#include "food_del_astar_planner/astar_planner.hpp"

// =============== Constructor ===============

AStarPlanner::AStarPlanner() : Node("astar_planner_node")
{
    // 1. Map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&AStarPlanner::mapCallback, this, _1)
    );

    // 2. AMCL Pose
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&AStarPlanner::amclPoseCallback, this, _1)
    );

    // 3. Goal Pose
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&AStarPlanner::goalPoseCallback, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Subscriber to /amcl_pose created");

    // path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

}

// =============== Callbacks ===============

// MAP callback
void AStarPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_)
{
    map_           = map_msg_;
    map_received_  = true;

    // ← save these so worldToMap() can use them
    map_origin_x_  = map_msg_->info.origin.position.x;
    map_origin_y_  = map_msg_->info.origin.position.y;
    map_resolution_ = map_msg_->info.resolution;

    RCLCPP_INFO(this->get_logger(), "Map received:");
    RCLCPP_INFO(this->get_logger(), "  width      : %d cells",    map_msg_->info.width);
    RCLCPP_INFO(this->get_logger(), "  height     : %d cells",    map_msg_->info.height);
    RCLCPP_INFO(this->get_logger(), "  resolution : %.3f m/cell", map_msg_->info.resolution);
    RCLCPP_INFO(this->get_logger(), "  origin x   : %.3f m",      map_msg_->info.origin.position.x);
    RCLCPP_INFO(this->get_logger(), "  origin y   : %.3f m",      map_msg_->info.origin.position.y);
    RCLCPP_INFO(this->get_logger(), "  total cells: %zu",         map_msg_->data.size());

    inflateMap();
}

// AMCL Pose callback
void AStarPlanner::amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_)
{
    // Extract the robot pose from the message
    robot_x_   = pose_msg_->pose.pose.position.x;
    robot_y_   = pose_msg_->pose.pose.position.y;
    robot_quaternion_ = tf2::Quaternion(
        pose_msg_->pose.pose.orientation.x,
        pose_msg_->pose.pose.orientation.y,
        pose_msg_->pose.pose.orientation.z,
        pose_msg_->pose.pose.orientation.w
    );
    robot_yaw_ = tf2::getYaw(robot_quaternion_);
    robot_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
    "Robot pose received — x: %.3f m  y: %.3f m  yaw: %.1f deg",
    robot_x_, robot_y_, robot_yaw_ * (180.0 / M_PI));
}

// Goal Pose Callback
void AStarPlanner::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_)
{
    goal_x_ = goal_msg_->pose.position.x;
    goal_y_ = goal_msg_->pose.position.y;
    goal_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
    "Goal pose received — x: %.3f m  y: %.3f m",
    goal_x_, goal_y_);

    // kick off A* algorithm
    runAStar();
}

// =============== Helper Functions ===============
AStarPlanner::Cell AStarPlanner::worldToMap(double wx, double wy)
{
    Cell cell;
    cell.col = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    cell.row = static_cast<int>((wy - map_origin_y_) / map_resolution_);
    return cell;
}

int AStarPlanner::cellToIndex(const Cell & cell)
{
    return cell.row * static_cast<int>(map_->info.width) + cell.col;
}

bool AStarPlanner::isValid(const Cell & cell)
{
    return cell.row >= 0 &&
           cell.col >= 0 &&
           cell.row < static_cast<int>(map_->info.height) &&
           cell.col < static_cast<int>(map_->info.width);
}

bool AStarPlanner::isObstacle(const Cell & cell)
{
    int index = cellToIndex(cell);
    int cost  = static_cast<int>(inflated_map_[index]);  // ← use inflated map
    return cost > 0 || cost < 0;  // anything non-zero is blocked
}

void AStarPlanner::inflateMap()
{
    // start with a copy of the original map
    int width  = static_cast<int>(map_->info.width);
    int height = static_cast<int>(map_->info.height);
    inflated_map_ = map_->data;

    // loop through every cell
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {

            int index = row * width + col;

            // only inflate cells that are obstacles
            if (static_cast<int>(map_->data[index]) == 100) {

                // spread outward by inflation_radius_ cells
                for (int dr = -inflation_radius_; dr <= inflation_radius_; dr++) {
                    for (int dc = -inflation_radius_; dc <= inflation_radius_; dc++) {

                        int nr = row + dr;
                        int nc = col + dc;

                        // check bounds
                        if (nr < 0 || nc < 0 || nr >= height || nc >= width) {
                            continue;
                        }

                        // only inflate free cells (don't overwrite existing obstacles)
                        int ni = nr * width + nc;
                        if (static_cast<int>(inflated_map_[ni]) == 0) {
                            inflated_map_[ni] = 50;  // mark as inflated zone
                        }
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Map inflated with radius %d cells", inflation_radius_);
}

// =============== A* Algorithm ===============
void AStarPlanner::runAStar()
{    
    RCLCPP_INFO(this->get_logger(), "Running A* algorithm...");

    // 1 Get the start and goal cells
    Cell start_cell = worldToMap(robot_x_, robot_y_);
    Cell goal_cell = worldToMap(goal_x_, goal_y_);

    RCLCPP_INFO(this->get_logger(), "Start cell: (row: %d, col: %d)", start_cell.row, start_cell.col);
    RCLCPP_INFO(this->get_logger(), "Goal cell: (row: %d, col: %d)", goal_cell.row, goal_cell.col);

    // debug — sample cost values around the start cell
    RCLCPP_INFO(this->get_logger(), "Cost values around start cell:");
    for (int dr = -2; dr <= 2; dr++) {
        for (int dc = -2; dc <= 2; dc++) {
            Cell c;
            c.row = start_cell.row + dr;
            c.col = start_cell.col + dc;
            if (isValid(c)) {
                int cost = static_cast<int>(map_->data[cellToIndex(c)]);
                if (cost > 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "  cell (%d,%d) cost = %d",
                        c.row, c.col, cost);
                }
            }
        }
    }
    // Step 2 — validate start and goal
    if (!isValid(start_cell)) {
        RCLCPP_WARN(this->get_logger(), "Start cell is outside the map!");
        return;
    }
    if (!isValid(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Goal cell is outside the map!");
        return;
    }
    if (isObstacle(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Goal cell is inside an obstacle!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Start and goal are valid!");

    // Step 3 — create the open list (min-heap, lowest f first)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;

    // Step 4 - put the start node in the open list
    AStarNode start_node;
    start_node.cell = start_cell;
    start_node.g    = 0.0;
    start_node.h    = std::sqrt(
                        std::pow(start_cell.row - goal_cell.row, 2) +
                        std::pow(start_cell.col - goal_cell.col, 2));
    start_node.f    = start_node.g + start_node.h;
    open_list.push(start_node);

    RCLCPP_INFO(this->get_logger(), "Open list initialized with start node.");
    RCLCPP_INFO(this->get_logger(), "Open list size: %zu", open_list.size());

    // Step 5 - create a closed set to put that cells that already explored
    std::unordered_set<Cell, CellHash> closed_set;
    // a collection of Cells that uses CellHash to store and find them instantly, with no duplicates allowed

    RCLCPP_INFO(this->get_logger(), "Closed set initialized.");

    // Step 5b — came_from map
    // for each cell, remember which cell we came from
    std::unordered_map<Cell, Cell, CellHash> came_from;

    // Step 6 - main A* loop
    while (!open_list.empty()) {
        
        // take the node with the lowest f from the open list
        AStarNode current_node = open_list.top(); // returns the lowest f node
        open_list.pop(); // removes the lowest f node from the open list

        // Step 6a — check if we reached the goal
        if (current_node.cell == goal_cell) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Reconstructing path...");

            // walk backwards from goal to start using came_from
            std::vector<Cell> path;
            Cell current = goal_cell;

            while (!(current == start_cell)) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_cell);

            // reverse so path goes start → goal
            std::reverse(path.begin(), path.end());

            RCLCPP_INFO(this->get_logger(), "Path found! %zu cells", path.size());

            // print first 5 cells of the path
            for (int i = 0; i < std::min((int)path.size(), 5); i++) {
                RCLCPP_INFO(this->get_logger(),
                    "  path[%d] = (row: %d, col: %d)", i, path[i].row, path[i].col);
            }

            // Step 8 — convert path cells to world coordinates and publish
            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp    = this->now();
            ros_path.header.frame_id = "map";

            for (const auto & cell : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp    = this->now();
                pose.header.frame_id = "map";

                // convert grid cell back to world coordinates
                // +0.5 to center the pose in the middle of the cell
                pose.pose.position.x = (cell.col + 0.5) * map_resolution_ + map_origin_x_;
                pose.pose.position.y = (cell.row + 0.5) * map_resolution_ + map_origin_y_;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;

                ros_path.poses.push_back(pose);
            }

            path_pub_->publish(ros_path);

            RCLCPP_INFO(this->get_logger(),
                "Path published on /planned_path — %zu waypoints", ros_path.poses.size());

            return;
        }

        // Step 6b — skip if already explored
        if (closed_set.count(current_node.cell)) {
            continue;
        }

        // Step 6c — mark as explored
        closed_set.insert(current_node.cell);

        RCLCPP_INFO(this->get_logger(),
            "Exploring cell: (row: %d, col: %d)  g: %.2f  h: %.2f  f: %.2f",
            current_node.cell.row, current_node.cell.col,
            current_node.g, current_node.h, current_node.f);

        // Step 7 — expand neighbors (8 directions)
        const std::vector<std::pair<int,int>> directions = {
            {-1,  0},  // North
            { 1,  0},  // South
            { 0,  1},  // East
            { 0, -1},  // West
            {-1, -1},  // North-West
            {-1,  1},  // North-East
            { 1, -1},  // South-West
            { 1,  1},  // South-East
        };

        for (const auto & [dr, dc] : directions)
        {
            Cell neighbor;
            neighbor.row = current_node.cell.row + dr;
            neighbor.col = current_node.cell.col + dc;

            // skip if outside the map
            if (!isValid(neighbor)) { continue; }

            // skip if obstacle
            if (isObstacle(neighbor)) { continue; }

            // skip if already explored
            if (closed_set.count(neighbor)) { continue; }

            // calculate move cost
            double move_cost;
            if (dr != 0 && dc != 0) {
                move_cost = 1.414;  // diagonal
            } else {
                move_cost = 1.0;    // straight
            }

            // calculate g, h, f
            double g = current_node.g + move_cost;
            double h = std::sqrt(
                std::pow(neighbor.row - goal_cell.row, 2) +
                std::pow(neighbor.col - goal_cell.col, 2));
            double f = g + h;

            // push neighbor into open list
            AStarNode neighbor_node;
            neighbor_node.cell = neighbor;
            neighbor_node.g    = g;
            neighbor_node.h    = h;
            neighbor_node.f    = f;
            open_list.push(neighbor_node);

            came_from[neighbor] = current_node.cell;  
        }
    }

    RCLCPP_WARN(this->get_logger(), "No path found!");
}

// ── main ────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}