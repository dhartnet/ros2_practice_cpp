#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robots/aStar.hpp"
#include "robots/sensors.hpp"
#include "robots/map.hpp"
#include "robots/planner.hpp"
#include "robots/state.hpp"

// Color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

class Robot : public rclcpp::Node {
    public:
        // Constructor - ClassName() : ParentClass("parent_args"), member1_(init_val), member2_(init_val) {
        // constructor body
        // }
        Robot() : Node("robot_node") {
            RCLCPP_INFO(this->get_logger(), GREEN "Robot node started!" RESET);

            // publish path to rviz for visualization
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);

            // Spin up support classes
            // create ptr to sensor object - pass this node to them
            sensors_ = std::make_unique<Sensors>(this);
            map_ = std::make_unique<Map>(this, 20, 20, 0.25);
            astar_ = std::make_unique<AStar>(1.5);

            map_->addObstacleRect(2, 2, 1, 1);
            map_->addObstacleRect(7, 7, 1, 1);
            map_->addObstacleCircle(12, 12, 2);

            // Create a timer to periodically run controlLoop in the timer callback
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Robot::controlLoop, this));
        }

    private:

        void publishPath(const std::vector<State>& path) {
            nav_msgs::msg::Path msg;
            msg.header.frame_id = "odom";
            msg.header.stamp = this->get_clock()->now();

            for (const auto& state : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = state.x;
                pose.pose.position.y = state.y;
                msg.poses.push_back(pose);
            }
            path_pub_->publish(msg);
        }

        void controlLoop() {
            RCLCPP_INFO(this->get_logger(), 
            CYAN "Pos: (%.2f, %.2f)" RESET, sensors_->getX(), sensors_->getY());

            // Get current position and compute path to goal if not path exists or if goal has changed
            if (!path_found_) {
                State current = {sensors_->getX(), sensors_->getY()};
                current_path_ = astar_->computePath(current, goal_, *map_);
            
                if (!current_path_.empty()) {
                    RCLCPP_INFO(this->get_logger(), GREEN "Path found!" RESET);
                    path_found_ = true;
                }
                else {
                    RCLCPP_WARN(this->get_logger(), RED "No path found!" RESET);
                }
            }

            publishPath(current_path_);

            map_->visualize();
        }

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        // pointers to keep them alive out of local scope of constructor
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<Sensors> sensors_;
        std::unique_ptr<Map> map_;
        std::unique_ptr<AStar> astar_;

        State goal_ = {4.0, 4.0}; // goal in world coordinates (meters)

        bool path_found_ = false;
        std::vector<State> current_path_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
};