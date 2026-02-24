#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "robots/state.hpp"

class Sensors {
  public:
    Sensors(rclcpp::Node* node) {
      // Subscriber for odometry made when given class object: sub - msg type - topic name - queue size - callback function
            odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 1,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    x_ = msg->pose.pose.position.x;
                    y_ = msg->pose.pose.position.y;
                });
                // lambda callback: [current class object] (smart pointer to message) { callback body }
    }
    
    double getX() const {return x_;}
    double getY() const {return y_;}
    
  private:
    // pointer for subscription to odometry topic - keeps it alive out of local scope of constructor
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Initial values for position, will be updated in callback
    double x_ = 0.0;
    double y_ = 0.0;
};