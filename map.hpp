#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "robots/state.hpp"
#include <vector>

class Map {
  public:
    Map(rclcpp::Node* node, int width, int height, double resolution) 
    : width_(width), height_(height), resolution_(resolution) {

      // Intitialize the occupancy grid with free cells
      grid_.resize(width_ * height_, 0);

      // Publisher to RViz to visualize the map
      map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

      RCLCPP_INFO(node->get_logger(), "Map initialized: %dx%d at %.2f m/cell", width_, height_, resolution_);
    }

    void addObstaclePoint(int x, int y) {
      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_[y * width_ + x] = 100; // Mark cells as occupied
      }
      else {
        RCLCPP_WARN(rclcpp::get_logger("Map"), "Attempted to add obstacle out of bounds: (%d, %d)", x, y);
      }
    }

    // Rectangle centered on a cell
    void addObstacleRect(int center_row, int center_col, int half_height, int half_width) {
        for (int r = center_row - half_height; r <= center_row + half_height; r++) {
            for (int c = center_col - half_width; c <= center_col + half_width; c++) {
                // Bounds check so we don't go outside the map
                if (r >= 0 && r < height_ && c >= 0 && c < width_) {
                    grid_[r * width_ + c] = 100;
                }
            }
        }
    }

    // Circle centered on a cell
    void addObstacleCircle(int center_row, int center_col, int radius) {
        for (int r = center_row - radius; r <= center_row + radius; r++) {
            for (int c = center_col - radius; c <= center_col + radius; c++) {
                // Check if the cell is within the circle                
                if (r >= 0 && r < height_ && c >= 0 && c < width_) {
                    int dr = r - center_row;
                    int dc = c - center_col;
                    if (dr * dr + dc * dc <= radius * radius) {
                        grid_[r * width_ + c] = 100;
                    }
                }
            }
        }
    }

    // World coords (meters) → grid cell
    std::pair<int,int> worldToGrid(double x, double y) const {
        int col = static_cast<int>(x / resolution_);
        int row = static_cast<int>(y / resolution_);
        return {row, col};
    }

    // Grid cell → world coords (meters)
    State gridToWorld(int row, int col) const {
        return {col * resolution_, row * resolution_};
    }

    void visualize() {
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.frame_id = "odom";
      msg.info.width = width_;
      msg.info.height = height_;
      msg.info.resolution = resolution_;
      msg.info.origin.position.x = 0.0;
      msg.info.origin.position.y = 0.0;
      msg.data = grid_;
      map_pub_->publish(msg);
    }

    bool outOfBounds(int x, int y) const {
      return x < 0 || x >= width_ || y < 0 || y >= height_;
    }
    
    bool inObstacle(int x, int y) const {
      return !outOfBounds(x, y) && grid_[y * width_ + x] == 100;
    }

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }

  private:
    int width_;
    int height_;
    double resolution_;

    std::vector<int8_t> grid_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  };