#include "hybrid_astar_planner/hybrid_astar.hpp"

#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>

HybridAStar::HybridAStar() {
  map_processor_ = nullptr;
  collision_checker_ = nullptr;
}

void HybridAStar::setMapProcessor(const MapProcessor *map_processor) {
  map_processor_ = map_processor;
}

void HybridAStar::setCollisionChecker(
    const CollisionChecker *collision_checker) {
  collision_checker_ = collision_checker;
}

double HybridAStar::heuristic(double x, double y, double gx, double gy) {
  return std::hypot(gx - x, gy - y);
}

bool HybridAStar::isGoal(const HybridNode *node, double goal_x, double goal_y) {
  double distance = std::hypot(goal_x - node->x, goal_y - node->y);

  return distance < goal_tolerance_;
}

std::vector<HybridNode *> HybridAStar::generateNeighbors(HybridNode *current) {
  std::vector<HybridNode *> neighbors;

  std::vector<double> steering_angles = {-max_steering_angle_, 0.0,
                                         max_steering_angle_};

  for (double steering : steering_angles) {
    double new_theta =
        current->theta + (step_size_ / wheelbase_) * std::tan(steering);

    double new_x = current->x + step_size_ * std::cos(current->theta);

    double new_y = current->y + step_size_ * std::sin(current->theta);

    bool valid = collision_checker_->isStateValid(new_x, new_y, new_theta);

    if (!valid) {
      continue;
    }

    HybridNode *node = new HybridNode();

    node->x = new_x;
    node->y = new_y;
    node->theta = new_theta;

    node->g_cost = current->g_cost + step_size_;

    node->parent = current;

    neighbors.push_back(node);
  }

  return neighbors;
}

void HybridAStar::reconstructPath(HybridNode *goal_node,
                                  nav_msgs::msg::Path &path) {
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  HybridNode *current = goal_node;

  while (current != nullptr) {
    geometry_msgs::msg::PoseStamped pose;

    pose.pose.position.x = current->x;
    pose.pose.position.y = current->y;

    tf2::Quaternion q;
    q.setRPY(0, 0, current->theta);

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    poses.push_back(pose);

    current = current->parent;
  }

  std::reverse(poses.begin(), poses.end());

  path.poses = poses;
}

int HybridAStar::thetaToBin(double theta) const {
  while (theta < 0) {
    theta += 2.0 * M_PI;
  }

  while (theta >= 2.0 * M_PI) {
    theta -= 2.0 * M_PI;
  }

  double bin_size = 2.0 * M_PI / theta_bins_;

  return static_cast<int>(theta / bin_size);
}

bool HybridAStar::plan(const geometry_msgs::msg::PoseStamped &start,
                       const geometry_msgs::msg::PoseStamped &goal,
                       nav_msgs::msg::Path &path) {
  double start_theta = tf2::getYaw(start.pose.orientation);

  HybridNode *start_node = new HybridNode();

  start_node->x = start.pose.position.x;

  start_node->y = start.pose.position.y;

  start_node->theta = start_theta;

  start_node->g_cost = 0.0;

  start_node->h_cost = heuristic(start_node->x, start_node->y,
                                 goal.pose.position.x, goal.pose.position.y);

  start_node->parent = nullptr;

  std::priority_queue<HybridNode *, std::vector<HybridNode *>, CompareNodes>
      open_list;

  open_list.push(start_node);

  int iterations = 0;

  while (!open_list.empty()) {
    iterations++;

    HybridNode *current = open_list.top();

    open_list.pop();

    if (isGoal(current, goal.pose.position.x, goal.pose.position.y)) {
      reconstructPath(current, path);

      return true;
    }

    auto neighbors = generateNeighbors(current);

    for (auto neighbor : neighbors) {
      neighbor->h_cost = heuristic(neighbor->x, neighbor->y,
                                   goal.pose.position.x, goal.pose.position.y);

      open_list.push(neighbor);
    }

    if (iterations > 50000) {
      break;
    }
  }

  return false;
}

StateKey HybridAStar::nodeToKey(HybridNode *node) const {
  int mx, my;

  map_processor_->worldToMap(node->x, node->y, mx, my);

  StateKey key;

  key.x = mx;
  key.y = my;
  key.theta = thetaToBin(node->theta);

  return key;
}
