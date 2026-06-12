#include "hybrid_astar_planner/collision_checker.hpp"

#include <cmath>

CollisionChecker::CollisionChecker() { map_processor_ = nullptr; }

void CollisionChecker::setMapProcessor(const MapProcessor *map_processor) {
  map_processor_ = map_processor;
}

bool CollisionChecker::isStateValid(double x, double y, double theta) const {
  (void)theta;

  if (map_processor_ == nullptr) {
    return false;
  }

  const double sample_step = 0.05;

  for (double dx = -robot_radius_; dx <= robot_radius_; dx += sample_step) {
    for (double dy = -robot_radius_; dy <= robot_radius_; dy += sample_step) {
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance > robot_radius_) {
        continue;
      }

      double wx = x + dx;
      double wy = y + dy;

      int mx, my;

      bool inside = map_processor_->worldToMap(wx, wy, mx, my);

      if (!inside) {
        return false;
      }

      if (map_processor_->isOccupied(mx, my)) {
        return false;
      }
    }
  }

  return true;
}
