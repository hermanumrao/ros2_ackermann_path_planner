#include "hybrid_astar_planner/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PlannerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
