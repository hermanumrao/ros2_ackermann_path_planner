#pragma once

#include "hybrid_astar_planner/map_processor.hpp"

class CollisionChecker {
public:
  CollisionChecker();

  void setMapProcessor(const MapProcessor *map_processor);

  bool isStateValid(double x, double y, double theta) const;

private:
  const MapProcessor *map_processor_;

  double robot_radius_ = 0.5;
};
