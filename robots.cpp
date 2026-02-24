#include <rclcpp/rclcpp.hpp>
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
            RCLCPP_INFO(this->get_logger(), GREEN "Robot node started!");

            // Spin up support classes
            // create ptr to sensor object - pass this node to them
            sensors_ = std::make_unique<Sensors>(this);
            map_ = std::make_unique<Map>(this, 20, 20, 0.25);

            map_->addObstacleRect(2, 2, 1, 1);
            map_->addObstacleRect(7, 7, 1, 1);
            map_->addObstacleCircle(12, 12, 2);

            // Create a timer to periodically run controlLoop in the timer callback
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Robot::controlLoop, this));
        }

    private:
        void controlLoop() {
            RCLCPP_INFO(this->get_logger(), 
            CYAN "Pos: (%.2f, %.2f)", sensors_->getX(), sensors_->getY());
            map_->visualize();
        }

        // pointers to keep them alive out of local scope of constructor
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<Sensors> sensors_;
        std::unique_ptr<Map> map_;
        // std::unique_ptr<Planner> planner_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
};