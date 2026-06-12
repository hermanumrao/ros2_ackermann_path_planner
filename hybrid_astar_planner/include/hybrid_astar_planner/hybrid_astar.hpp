#pragma once

#include "hybrid_astar_planner/collision_checker.hpp"
#include "hybrid_astar_planner/map_processor.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <vector>

struct HybridNode {
  double x;
  double y;
  double theta;

  double g_cost;
  double h_cost;

  HybridNode *parent;

  double totalCost() const { return g_cost + h_cost; }
};

struct CompareNodes {
  bool operator()(const HybridNode *a, const HybridNode *b) {
    return a->totalCost() > b->totalCost();
  }
};

struct StateKey {
  int x;
  int y;
  int theta;

  bool operator==(const StateKey &other) const {
    return x == other.x && y == other.y && theta == other.theta;
  }
};

struct StateKeyHasher {
  std::size_t operator()(const StateKey &key) const {
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^
           (std::hash<int>()(key.theta) << 2);
  }
};

class HybridAStar {
public:
  HybridAStar();

  void setMapProcessor(const MapProcessor *map_processor);

  void setCollisionChecker(const CollisionChecker *collision_checker);

  bool plan(const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            nav_msgs::msg::Path &path);
  int thetaToBin(double theta) const;

  StateKey nodeToKey(HybridNode *node) const;

private:
  bool isGoal(const HybridNode *node, double goal_x, double goal_y);

  double heuristic(double x, double y, double gx, double gy);

  void reconstructPath(HybridNode *goal_node, nav_msgs::msg::Path &path);

  std::vector<HybridNode *> generateNeighbors(HybridNode *current);

  const MapProcessor *map_processor_;
  const CollisionChecker *collision_checker_;

  int theta_bins_ = 72;

  double step_size_ = 0.3;

  double wheelbase_ = 0.5;

  double max_steering_angle_ = 25.0 * 3.14159265358979323846 / 180.0;

  double goal_tolerance_ = 0.5;
};
