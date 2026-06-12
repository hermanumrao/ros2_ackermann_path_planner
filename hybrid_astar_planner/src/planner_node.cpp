#include "hybrid_astar_planner/node.hpp"

PlannerNode::PlannerNode() : Node("hybrid_astar_planner") {
  collision_checker_.setMapProcessor(&map_processor_);

  planner_.setMapProcessor(&map_processor_);

  planner_.setCollisionChecker(&collision_checker_);

  /*
   * Map
   */
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map2d", 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  /*
   * Start pose from odom_map
   */
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_map", 50,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  /*
   * Goal
   */
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PlannerNode::goalPoseCallback, this, std::placeholders::_1));

  /*
   * Path publisher
   */
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

  /*
   * Debug map
   */
  inflated_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_map", 10);

  RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner Started");
}

/*
 * Map
 */
void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;

  map_processor_.setMap(*msg);

  inflated_map_pub_->publish(map_processor_.getInflatedMap());

  has_map_ = true;

  RCLCPP_INFO(this->get_logger(), "Map received");
}

/*
 * Start pose from odom
 */
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  start_pose_.header = msg->header;

  start_pose_.pose = msg->pose.pose;

  has_start_ = true;

  RCLCPP_INFO(this->get_logger(), "Start pose updated");

  tryPlan();
}

/*
 * Goal
 */
void PlannerNode::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_pose_ = *msg;

  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "Goal received");

  tryPlan();
}

/*
 * Planner
 */
void PlannerNode::tryPlan() {
  if (!has_map_ || !has_start_ || !has_goal_) {
    return;
  }

  nav_msgs::msg::Path path;

  path.header.frame_id = "map";

  bool success = planner_.plan(start_pose_, goal_pose_, path);

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Path found");

    path_pub_->publish(path);
  } else {
    RCLCPP_WARN(this->get_logger(), "Path planning failed");
  }
}
