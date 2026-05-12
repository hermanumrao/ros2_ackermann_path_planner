#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>

class MapProcessor {
public:
  MapProcessor();

  void setMap(const nav_msgs::msg::OccupancyGrid &map);

  bool isOccupied(int x, int y) const;

  bool worldToMap(double wx, double wy, int &mx, int &my) const;

  void mapToWorld(int mx, int my, double &wx, double &wy) const;

  const nav_msgs::msg::OccupancyGrid &getInflatedMap() const;

private:
  void filterSparseObstacles();
  void inflateObstacles();

  nav_msgs::msg::OccupancyGrid raw_map_;
  nav_msgs::msg::OccupancyGrid inflated_map_;

  bool has_map_ = false;

  double inflation_radius_ = 0.5;
};
