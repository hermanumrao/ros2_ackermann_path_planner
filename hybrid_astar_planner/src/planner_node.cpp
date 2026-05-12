#include "hybrid_astar_planner/node.hpp"

PlannerNode::PlannerNode() : Node("hybrid_astar_planner") {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map2d", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  initial_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10,
          std::bind(&PlannerNode::initialPoseCallback, this,
                    std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PlannerNode::goalPoseCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

  inflated_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_map", 10);

  RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner Started");
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;

  map_processor_.setMap(*msg);
  inflated_map_pub_->publish(map_processor_.getInflatedMap());
  has_map_ = true;

  RCLCPP_INFO(this->get_logger(), "Map received");
}

void PlannerNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  start_pose_.header = msg->header;
  start_pose_.pose = msg->pose.pose;

  has_start_ = true;

  RCLCPP_INFO(this->get_logger(), "Start pose received");

  tryPlan();
}

void PlannerNode::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_pose_ = *msg;

  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "Goal pose received");

  tryPlan();
}

void PlannerNode::tryPlan() {
  if (!has_map_ || !has_start_ || !has_goal_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Planning path...");

  int mx, my;

  bool success = map_processor_.worldToMap(start_pose_.pose.position.x,
                                           start_pose_.pose.position.y, mx, my);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Start pose outside map");

    return;
  }

  bool occupied = map_processor_.isOccupied(mx, my);

  RCLCPP_INFO(this->get_logger(), "Start Cell: (%d, %d), occupied=%d", mx, my,
              occupied);
}
