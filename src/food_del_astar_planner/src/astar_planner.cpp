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
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
}

// =============== Callbacks ===============

// MAP callback
void AStarPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_)
{
    map_            = map_msg_;
    map_received_   = true;
    map_origin_x_   = map_msg_->info.origin.position.x;
    map_origin_y_   = map_msg_->info.origin.position.y;
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
    int cost  = static_cast<int>(inflated_map_[index]);
    return cost > 0 || cost < 0;  // anything non-zero is blocked
}

void AStarPlanner::inflateMap()
{
    int width  = static_cast<int>(map_->info.width);
    int height = static_cast<int>(map_->info.height);
    inflated_map_ = map_->data;

    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {

            int index = row * width + col;

            if (static_cast<int>(map_->data[index]) == 100) {

                for (int dr = -inflation_radius_; dr <= inflation_radius_; dr++) {
                    for (int dc = -inflation_radius_; dc <= inflation_radius_; dc++) {

                        int nr = row + dr;
                        int nc = col + dc;

                        if (nr < 0 || nc < 0 || nr >= height || nc >= width) {
                            continue;
                        }

                        int ni = nr * width + nc;
                        if (static_cast<int>(inflated_map_[ni]) == 0) {
                            inflated_map_[ni] = 50;
                        }
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(),
        "Map inflated with radius %d cells", inflation_radius_);
}

// =============== Line-of-Sight Check (Bresenham) ===============

bool AStarPlanner::hasLineOfSight(const Cell & from, const Cell & to)
{
    int r0 = from.row, c0 = from.col;
    int r1 = to.row,   c1 = to.col;

    int dr = std::abs(r1 - r0);
    int dc = std::abs(c1 - c0);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = dr - dc;

    while (true) {
        Cell c;
        c.row = r0;
        c.col = c0;

        if (!isValid(c) || isObstacle(c)) return false;

        if (r0 == r1 && c0 == c1) break;

        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r0 += sr; }
        if (e2 <  dr) { err += dr; c0 += sc; }
    }

    return true;
}

// =============== Path Smoothing ===============

std::vector<AStarPlanner::Cell> AStarPlanner::smoothPath(
    const std::vector<Cell> & path)
{
    if (path.size() <= 2) return path;

    std::vector<Cell> smoothed;
    smoothed.push_back(path[0]);  // always keep start

    int anchor = 0;

    while (anchor < (int)path.size() - 1) {
        int furthest = anchor + 1;

        // find the furthest cell reachable in a straight line from anchor
        for (int i = anchor + 2; i < (int)path.size(); i++) {
            if (hasLineOfSight(path[anchor], path[i])) {
                furthest = i;
            }
        }

        smoothed.push_back(path[furthest]);
        anchor = furthest;
    }

    return smoothed;
}

// =============== A* Algorithm ===============

void AStarPlanner::runAStar()
{
    RCLCPP_INFO(this->get_logger(), "Running A* algorithm...");

    // Step 1 — get start and goal cells
    Cell start_cell = worldToMap(robot_x_, robot_y_);
    Cell goal_cell  = worldToMap(goal_x_,  goal_y_);

    RCLCPP_INFO(this->get_logger(), "Start cell: (row: %d, col: %d)", start_cell.row, start_cell.col);
    RCLCPP_INFO(this->get_logger(), "Goal cell:  (row: %d, col: %d)", goal_cell.row,  goal_cell.col);

    // Step 2 — validate start and goal
    if (!isValid(start_cell)) {
        RCLCPP_WARN(this->get_logger(), "Start cell is outside the map!");
        return;
    }
    if (!isValid(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Goal cell is outside the map!");
        return;
    }
    if (isObstacle(start_cell)) {
        RCLCPP_WARN(this->get_logger(), "Start cell is inside an obstacle!");
        return;
    }
    if (isObstacle(goal_cell)) {
        RCLCPP_WARN(this->get_logger(), "Goal cell is inside an obstacle!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Start and goal are valid!");

    // Step 3 — open list (min-heap by f)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;

    // Step 4 — g_score map: best known cost to reach each cell
    std::unordered_map<Cell, double, CellHash> g_score;
    g_score[start_cell] = 0.0;

    // Step 5 — came_from map
    std::unordered_map<Cell, Cell, CellHash> came_from;

    // Step 6 — closed set
    std::unordered_set<Cell, CellHash> closed_set;

    // Step 7 — push start node
    AStarNode start_node;
    start_node.cell = start_cell;
    start_node.g    = 0.0;
    start_node.h    = std::sqrt(
                        std::pow(start_cell.row - goal_cell.row, 2) +
                        std::pow(start_cell.col - goal_cell.col, 2));
    start_node.f    = start_node.g + start_node.h;
    open_list.push(start_node);

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

    // Step 8 — main A* loop
    while (!open_list.empty()) {

        AStarNode current_node = open_list.top();
        open_list.pop();

        // reached goal?
        if (current_node.cell == goal_cell) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Reconstructing path...");

            // reconstruct raw path
            std::vector<Cell> path;
            Cell current = goal_cell;

            while (!(current == start_cell)) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_cell);
            std::reverse(path.begin(), path.end());

            RCLCPP_INFO(this->get_logger(), "Raw path: %zu cells", path.size());

            // smooth the path
            std::vector<Cell> smooth = smoothPath(path);
            RCLCPP_INFO(this->get_logger(), "Smoothed path: %zu waypoints", smooth.size());

            // publish
            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp    = this->now();
            ros_path.header.frame_id = "map";

            for (const auto & cell : smooth) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp    = this->now();
                pose.header.frame_id = "map";

                pose.pose.position.x = (cell.col + 0.5) * map_resolution_ + map_origin_x_;
                pose.pose.position.y = (cell.row + 0.5) * map_resolution_ + map_origin_y_;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;

                ros_path.poses.push_back(pose);
            }

            path_pub_->publish(ros_path);

            RCLCPP_INFO(this->get_logger(),
                "Path published on /plan — %zu waypoints", ros_path.poses.size());

            return;
        }

        // skip if already fully explored
        if (closed_set.count(current_node.cell)) {
            continue;
        }
        closed_set.insert(current_node.cell);

        // expand neighbors
        for (const auto & [dr, dc] : directions) {

            Cell neighbor;
            neighbor.row = current_node.cell.row + dr;
            neighbor.col = current_node.cell.col + dc;

            if (!isValid(neighbor))        { continue; }
            if (isObstacle(neighbor))      { continue; }
            if (closed_set.count(neighbor)){ continue; }

            double move_cost = (dr != 0 && dc != 0) ? 1.414 : 1.0;
            double tentative_g = current_node.g + move_cost;

            // only proceed if this is a better path to neighbor
            if (g_score.count(neighbor) && tentative_g >= g_score[neighbor]) {
                continue;
            }

            // best path to neighbor so far — record it
            g_score[neighbor]   = tentative_g;
            came_from[neighbor] = current_node.cell;

            double h = std::sqrt(
                std::pow(neighbor.row - goal_cell.row, 2) +
                std::pow(neighbor.col - goal_cell.col, 2));

            AStarNode neighbor_node;
            neighbor_node.cell = neighbor;
            neighbor_node.g    = tentative_g;
            neighbor_node.h    = h;
            neighbor_node.f    = tentative_g + h;
            open_list.push(neighbor_node);
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