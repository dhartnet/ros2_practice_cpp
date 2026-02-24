#pragma once
#include <vector> // for returned path
#include <queue> // priorty queue for open set
#include <unordered_map> // parent and g score tracking
#include <unordered_set> // closed/visitied set
#include <cmath> // for heuristic calculation std::hypot
#include <chrono> // timing
#include <algorithm>  // for std::reverse
#include "rclcpp/rclcpp.hpp"
#include "robots/planner.hpp"

// inherit from Planner base class and implement computePath method
class AStar : public Planner {
  public:
    AStar() : Planner() {} // calls parent constructor - doesn't add anything

    std::vector<State> computePath(const State& start, const State& goal, const Map& map) override {
      // add logic
      //time start
      // add variables for open set (priority queue), closed set (unordered set), parent tracking (unordered map), g score tracking (unordered map)?
      // need a weigfhted aspect
      // init queue, gscore, parent
      // while open set not empty
        // pop lowest fscore node from open set
        // if node is goal, reconstruct path and return
        // add node to closed set
        // for each neighbor of node
          // if neighbor in closed set, skip
          // calculate tentative g score
          // if neighbor not in open set or tentative g score < existing g score
            // update parent and g score
            // if neighbor not in open set, add to open set with f score
      // if we exhaust open set without finding goal, return empty path or throw exception
      return {};
    }
};