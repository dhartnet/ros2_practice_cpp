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

// hash function to allow using pairs as keys in unordered maps/sets for parent tracking, closed set, and g score tracking
struct PairHash {
    size_t operator()(const std::pair<int,int>& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second) << 1;
    }
};

// inherit from Planner base class and implement computePath method
class AStar : public Planner {
  public:
    AStar(double weight = 1.0) : Planner(), weight_(weight) {} // calls parent constructor - doesn't add anything
    
    // function for computing path to goal
    std::vector<State> computePath(const State& start, const State& goal, const Map& map) override {

      // Start timer
      auto t_start = std::chrono::high_resolution_clock::now();

      auto start_node = map.worldToGrid(start.x, start.y);
      auto goal_node = map.worldToGrid(goal.x, goal.y);

      // Validate start and goal
      if (map.outOfBounds(start_node.first, start_node.second) || 
          map.inObstacle(start_node.first, start_node.second)) {
          RCLCPP_WARN(rclcpp::get_logger("astar"), "Start is invalid!");
          return {};
      }

      if (map.outOfBounds(goal_node.first, goal_node.second) || 
          map.inObstacle(goal_node.first, goal_node.second)) {
          RCLCPP_WARN(rclcpp::get_logger("astar"), "Goal is invalid!");
          return {};
      }

      // open set priority queue - basically heap where lowest f score has highest priority - stores (f score, (x, y)) pairs
      std:: priority_queue<std::pair<double, std::pair<int,int>>, // what the element in queue looks like - (f score, (x, y))
                          std::vector<std::pair<double, std::pair<int,int>>>, // what the underlaying layout looks like - almost always vector of same layout as above
                          std::greater<std::pair<double, std::pair<int,int>>>> open_set_; // how it compares values - grater means lowest f score has highest priority

      // closed set to track visited nodes - stores (x, y) pairs
      std::unordered_set<std::pair<int,int>, PairHash> closed_set_; // (value, how to hash value)

      // g score tracking - stores (x, y) pairs as keys and g score as value
      std::unordered_map<std::pair<int,int>, double, PairHash> g_; // key, value, how to hash key

      // parent tracking to reconstruct path - stores val (x, y) pairs as keys and parent (x, y) pairs as values
      std::unordered_map<std::pair<int,int>, std::pair<int,int>, PairHash> parent_;

      // final path - reserve some space
      std:: vector<State> path;
      path.reserve((map.getWidth() * map.getHeight()) / 10);

      // initialize sets
      open_set_.push(std::make_pair(0.0, start_node)); // (f, pair)
      g_[start_node] = 0.0;
      parent_[start_node] = start_node; // for looping until parent == start
      
      // start loop
      while (!open_set_.empty()) {

        // get next point, pop point from queue, and add to closed set
        auto current = open_set_.top().second;
        open_set_.pop();
        closed_set_.insert(current);

        // check if at goal - backtrack and return path
        if (current == goal_node) {

          // make path vector and loop through parent map starting with current node
          auto current_node = current;
          while (current_node != start_node) {
            path.push_back(map.gridToWorld(current_node.first, current_node.second));
            current_node = parent_[current_node];
          }

          // add start and reverse path so its start to goal
          path.push_back(start);
          std::reverse(path.begin(), path.end());

          // Stop timer, print stats, return path vector
          auto t_end = std::chrono::high_resolution_clock::now();
          double time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
          printStats(path, time_ms, closed_set_.size());
          return path;
        }

        // loop through possible moves if not at goal and keep going
        for (const auto& move : moves_) {

          // make move
          std::pair<int,int> neighbor = {current.first + move.first, current.second + move.second};

          // check bounds of map, in obstacle, or already visited
          if (!map.outOfBounds(neighbor.first, neighbor.second) 
              && !map.inObstacle(neighbor.first, neighbor.second)
              && closed_set_.find(neighbor) == closed_set_.end()) {
                
                // calc temp_g score for neighbor
                double temp_g = g_[current] + std::hypot(neighbor.first - current.first, neighbor.second - current.second);

                // if neighbor not in g score map or temp_g is better than existing g score, update and push to queues
                if (g_.find(neighbor) == g_.end() || temp_g < g_.at(neighbor)) {
                  g_[neighbor] = temp_g;
                  double f= temp_g + weight_ * std::hypot(neighbor.first - goal_node.first, neighbor.second - goal_node.second);
                  open_set_.push(std::make_pair(f, neighbor));
                  parent_[neighbor] = current;
                }
        }


      }
      // if we exhaust open set without finding goal, return empty path or throw exception
  }

  RCLCPP_WARN(rclcpp::get_logger("astar"), "No path found!");
  return {};
  }

  private:

    // weight
    double weight_;
    
};