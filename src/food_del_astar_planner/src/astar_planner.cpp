#include "food_del_astar_planner/astar_planner.hpp"

// =============== Constructor ===============

AStarPlanner::AStarPlanner() : Node("astar_planner_node")
{
    // ── Owned global costmap ───────────────────────────────────────────────
    global_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap",
        std::string{get_namespace()},
        "global_costmap"
    );

    global_costmap_->configure();

    // Activate after 3s — gives TF (map → base_link) time to be ready
    activate_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            RCLCPP_INFO(this->get_logger(), "Activating global_costmap...");
            global_costmap_->activate();
            activate_timer_->cancel();
            costmap_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "global_costmap activated.");
        });

    // ── AMCL Pose ─────────────────────────────────────────────────────────
    amcl_pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&AStarPlanner::amclPoseCallback, this, _1));

    // ── Goal Pose ──────────────────────────────────────────────────────────
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&AStarPlanner::goalPoseCallback, this, _1));

    // ── Path publisher ─────────────────────────────────────────────────────
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

    RCLCPP_INFO(this->get_logger(),
        "AStarPlanner started — global_costmap owned internally. "
        "Waiting for /amcl_pose and /goal_pose.");
}

// =============== Destructor ===============

AStarPlanner::~AStarPlanner()
{
    if (global_costmap_) {
        global_costmap_->deactivate();
        global_costmap_->cleanup();
    }
}

// =============== Callbacks ===============

void AStarPlanner::amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_)
{
    robot_x_ = pose_msg_->pose.pose.position.x;
    robot_y_ = pose_msg_->pose.pose.position.y;
    robot_quaternion_ = tf2::Quaternion(
        pose_msg_->pose.pose.orientation.x,
        pose_msg_->pose.pose.orientation.y,
        pose_msg_->pose.pose.orientation.z,
        pose_msg_->pose.pose.orientation.w);
    robot_yaw_ = tf2::getYaw(robot_quaternion_);
    robot_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Robot pose — x: %.3f  y: %.3f  yaw: %.1f deg",
        robot_x_, robot_y_, robot_yaw_ * (180.0 / M_PI));
}

void AStarPlanner::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_)
{
    goal_x_ = goal_msg_->pose.position.x;
    goal_y_ = goal_msg_->pose.position.y;
    goal_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
        "Goal pose — x: %.3f  y: %.3f", goal_x_, goal_y_);

    runAStar();
}

// =============== Helper Functions ===============

AStarPlanner::Cell AStarPlanner::worldToMap(
    double wx, double wy, nav2_costmap_2d::Costmap2D * costmap)
{
    // Use nav2's built-in conversion — handles origin and resolution correctly
    uint32_t mx, my;
    Cell cell;
    if (costmap->worldToMap(wx, wy, mx, my)) {
        cell.col = static_cast<int>(mx);
        cell.row = static_cast<int>(my);
    } else {
        // Out of bounds — clamp to edges so isValid() catches it
        cell.col = -1;
        cell.row = -1;
    }
    return cell;
}

int AStarPlanner::cellToIndex(
    const Cell & cell, nav2_costmap_2d::Costmap2D * costmap)
{
    return cell.row * static_cast<int>(costmap->getSizeInCellsX()) + cell.col;
}

bool AStarPlanner::isValid(
    const Cell & cell, nav2_costmap_2d::Costmap2D * costmap)
{
    return cell.row >= 0 &&
           cell.col >= 0 &&
           cell.row < static_cast<int>(costmap->getSizeInCellsY()) &&
           cell.col < static_cast<int>(costmap->getSizeInCellsX());
}

bool AStarPlanner::isObstacle(
    const Cell & cell, nav2_costmap_2d::Costmap2D * costmap)
{
    uint8_t cost = costmap->getCharMap()[cellToIndex(cell, costmap)];
    return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
           cost == nav2_costmap_2d::NO_INFORMATION;
}

// =============== Line-of-Sight (Bresenham) ===============

bool AStarPlanner::lineOfSight(
    const Cell & a, const Cell & b, nav2_costmap_2d::Costmap2D * costmap)
{
    int r0 = a.row, c0 = a.col;
    int r1 = b.row, c1 = b.col;

    int dr = std::abs(r1 - r0);
    int dc = std::abs(c1 - c0);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = dr - dc;

    while (true) {
        Cell cur{r0, c0};
        if (!isValid(cur, costmap) || isObstacle(cur, costmap)) {
            return false;
        }
        if (r0 == r1 && c0 == c1) break;

        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r0 += sr; }
        if (e2 <  dr) { err += dr; c0 += sc; }
    }
    return true;
}

// =============== Path Smoothing (String Pulling) ===============

std::vector<AStarPlanner::Cell> AStarPlanner::smoothPath(
    const std::vector<Cell> & path, nav2_costmap_2d::Costmap2D * costmap)
{
    if (path.size() <= 2) return path;

    std::vector<Cell> smoothed;
    smoothed.push_back(path.front());

    size_t anchor = 0;

    while (anchor < path.size() - 1) {
        // Find the furthest cell we can reach in line-of-sight from anchor
        size_t furthest = anchor + 1;
        for (size_t i = anchor + 2; i < path.size(); i++) {
            if (lineOfSight(path[anchor], path[i], costmap)) {
                furthest = i;
            } else {
                break;
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
    if (!costmap_ready_) {
        RCLCPP_WARN(this->get_logger(), "Global costmap not ready yet — aborting.");
        return;
    }
    if (!robot_pose_received_) {
        RCLCPP_WARN(this->get_logger(), "No robot pose yet — aborting.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Running A*...");

    auto * costmap2d = global_costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap2d->getMutex()));

    if (costmap2d->getSizeInCellsX() == 0 || costmap2d->getSizeInCellsY() == 0) {
        RCLCPP_WARN(this->get_logger(), "Costmap is empty — aborting.");
        return;
    }

    // Step 1 — world → grid using nav2's built-in worldToMap
    Cell start_cell = worldToMap(robot_x_, robot_y_, costmap2d);
    Cell goal_cell  = worldToMap(goal_x_,  goal_y_,  costmap2d);

    RCLCPP_INFO(this->get_logger(),
        "Start cell: (row %d, col %d)  world: (%.2f, %.2f)",
        start_cell.row, start_cell.col, robot_x_, robot_y_);
    RCLCPP_INFO(this->get_logger(),
        "Goal cell:  (row %d, col %d)  world: (%.2f, %.2f)",
        goal_cell.row, goal_cell.col, goal_x_, goal_y_);

    // Step 2 — validate
    if (!isValid(start_cell, costmap2d)) {
        RCLCPP_WARN(this->get_logger(), "Start is outside the costmap!");
        return;
    }
    if (!isValid(goal_cell, costmap2d)) {
        RCLCPP_WARN(this->get_logger(), "Goal is outside the costmap!");
        return;
    }
    if (isObstacle(goal_cell, costmap2d)) {
        RCLCPP_WARN(this->get_logger(), "Goal is inside an obstacle!");
        return;
    }

    // Step 3 — open list (min-heap on f)
    std::priority_queue<AStarNode,
                        std::vector<AStarNode>,
                        std::greater<AStarNode>> open_list;

    AStarNode start_node;
    start_node.cell = start_cell;
    start_node.g    = 0.0;
    start_node.h    = std::hypot(start_cell.row - goal_cell.row,
                                  start_cell.col - goal_cell.col);
    start_node.f    = start_node.h;
    open_list.push(start_node);

    std::unordered_set<Cell, CellHash> closed_set;
    std::unordered_map<Cell, Cell, CellHash> came_from;

    const std::vector<std::pair<int,int>> directions = {
        {-1,  0}, { 1,  0}, { 0,  1}, { 0, -1},
        {-1, -1}, {-1,  1}, { 1, -1}, { 1,  1},
    };

    // Step 4 — main loop
    while (!open_list.empty()) {

        AStarNode current = open_list.top();
        open_list.pop();

        if (current.cell == goal_cell) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Reconstructing path...");

            // Reconstruct raw path
            std::vector<Cell> raw_path;
            Cell c = goal_cell;
            while (!(c == start_cell)) {
                raw_path.push_back(c);
                c = came_from[c];
            }
            raw_path.push_back(start_cell);
            std::reverse(raw_path.begin(), raw_path.end());

            // Smooth path — removes zigzag via line-of-sight string pulling
            std::vector<Cell> smooth = smoothPath(raw_path, costmap2d);

            RCLCPP_INFO(this->get_logger(),
                "Raw path: %zu waypoints → Smoothed: %zu waypoints",
                raw_path.size(), smooth.size());

            // Unlock before publishing
            lock.unlock();

            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp    = this->now();
            ros_path.header.frame_id = "map";

            const double res = costmap2d->getResolution();
            const double ox  = costmap2d->getOriginX();
            const double oy  = costmap2d->getOriginY();

            for (size_t i = 0; i < smooth.size(); i++) {
                const auto & cell = smooth[i];
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp    = this->now();
                pose.header.frame_id = "map";

                // Cell center in world coordinates
                pose.pose.position.x = (cell.col + 0.5) * res + ox;
                pose.pose.position.y = (cell.row + 0.5) * res + oy;
                pose.pose.position.z = 0.0;

                // Compute heading toward next waypoint
                if (i + 1 < smooth.size()) {
                    double dx = (smooth[i+1].col - cell.col) * res;
                    double dy = (smooth[i+1].row - cell.row) * res;
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                } else {
                    pose.pose.orientation.w = 1.0;
                }

                ros_path.poses.push_back(pose);
            }

            path_pub_->publish(ros_path);
            RCLCPP_INFO(this->get_logger(),
                "Path published on /plan — %zu waypoints", ros_path.poses.size());
            return;
        }

        if (closed_set.count(current.cell)) { continue; }
        closed_set.insert(current.cell);

        for (const auto & [dr, dc] : directions) {
            Cell neighbor;
            neighbor.row = current.cell.row + dr;
            neighbor.col = current.cell.col + dc;

            if (!isValid(neighbor, costmap2d))    { continue; }
            if (isObstacle(neighbor, costmap2d))  { continue; }
            if (closed_set.count(neighbor))        { continue; }

            double move_cost = (dr != 0 && dc != 0) ? 1.414 : 1.0;
            double g = current.g + move_cost;
            double h = std::hypot(neighbor.row - goal_cell.row,
                                  neighbor.col - goal_cell.col);

            AStarNode nb;
            nb.cell = neighbor;
            nb.g    = g;
            nb.h    = h;
            nb.f    = g + h;
            open_list.push(nb);

            if (!came_from.count(neighbor)) {
                came_from[neighbor] = current.cell;
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "No path found!");
}

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AStarPlanner>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 2);

    executor.add_node(node);
    executor.add_node(node->get_global_costmap()->get_node_base_interface());

    executor.spin();
    rclcpp::shutdown();
    return 0;
}