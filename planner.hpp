#pragma once
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "robots/map.hpp"
#include "robots/state.hpp"

class Planner {
  public:
    Planner() {
      moves_ = {{1,0}, {-1,0}, {0,1}, {0,-1},
                  {1,1}, {1,-1}, {-1,-1}, {-1,1}};
    }

    // deconstrctor - necessary for base class with virtual functions to ensure proper cleanup of derived classes
    virtual ~Planner() = default;
    virtual std::vector<State> computePath(const State& start, const State& goal, const Map& map) = 0;

    bool hasReachedGoal(const State& current, const State& goal, double threshold = 0.1) {
        double dx = current.x - goal.x;
        double dy = current.y - goal.y;
        return std::hypot(dx, dy) < threshold;
    }

    // Calculate path length
    double pathLength(const std::vector<State>& path) {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); i++) {
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            length += std::hypot(dx, dy);
        }
        return length;
    }

    // virtual function to print stats about the pathfinding result, can be overridden by derived classes for custom behavior
    virtual void printStats(const std::vector<State>& path, double time_ms, size_t nodes_explored) {
        RCLCPP_INFO(rclcpp::get_logger("planner"),
        "Path length: %.2f m | Time: %.2f ms | Nodes explored: %zu",
        pathLength(path), time_ms, nodes_explored);
    }

  protected:
    std::vector<std::pair<int,int>> moves_;

  };