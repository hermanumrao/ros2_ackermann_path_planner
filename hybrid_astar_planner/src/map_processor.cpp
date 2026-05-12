#include "hybrid_astar_planner/map_processor.hpp"

#include <cmath>

MapProcessor::MapProcessor() {}

void MapProcessor::setMap(const nav_msgs::msg::OccupancyGrid &map) {
  raw_map_ = map;
  inflated_map_ = map;

  has_map_ = true;
  filterSparseObstacles();
  inflateObstacles();
}

void MapProcessor::filterSparseObstacles() {
  int width = raw_map_.info.width;
  int height = raw_map_.info.height;

  std::vector<int8_t> filtered = raw_map_.data;

  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int index = y * width + x;

      if (raw_map_.data[index] < 50) {
        continue;
      }

      int occupied_neighbors = 0;

      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) {
            continue;
          }

          int nx = x + dx;
          int ny = y + dy;

          int nindex = ny * width + nx;

          if (raw_map_.data[nindex] > 50) {
            occupied_neighbors++;
          }
        }
      }

      if (occupied_neighbors < 2) {
        filtered[index] = 0;
      }
    }
  }

  raw_map_.data = filtered;
}

void MapProcessor::inflateObstacles() {
  int width = raw_map_.info.width;
  int height = raw_map_.info.height;

  double resolution = raw_map_.info.resolution;

  int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / resolution));

  std::vector<int8_t> inflated_data = raw_map_.data;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;

      if (raw_map_.data[index] > 50) {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
          for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
              continue;
            }

            double distance = std::sqrt(dx * dx + dy * dy) * resolution;

            if (distance <= inflation_radius_) {
              int new_index = ny * width + nx;

              inflated_data[new_index] = 100;
            }
          }
        }
      }
    }
  }

  inflated_map_.data = inflated_data;
}

bool MapProcessor::isOccupied(int x, int y) const {
  if (!has_map_) {
    return true;
  }

  int width = inflated_map_.info.width;
  int height = inflated_map_.info.height;

  if (x < 0 || y < 0 || x >= width || y >= height) {
    return true;
  }

  int index = y * width + x;

  return inflated_map_.data[index] > 50;
}

bool MapProcessor::worldToMap(double wx, double wy, int &mx, int &my) const {
  if (!has_map_) {
    return false;
  }

  double origin_x = inflated_map_.info.origin.position.x;

  double origin_y = inflated_map_.info.origin.position.y;

  double resolution = inflated_map_.info.resolution;

  mx = static_cast<int>((wx - origin_x) / resolution);

  my = static_cast<int>((wy - origin_y) / resolution);

  int width = inflated_map_.info.width;
  int height = inflated_map_.info.height;

  if (mx < 0 || my < 0 || mx >= width || my >= height) {
    return false;
  }

  return true;
}

void MapProcessor::mapToWorld(int mx, int my, double &wx, double &wy) const {
  double resolution = inflated_map_.info.resolution;

  double origin_x = inflated_map_.info.origin.position.x;

  double origin_y = inflated_map_.info.origin.position.y;

  wx = origin_x + (mx + 0.5) * resolution;

  wy = origin_y + (my + 0.5) * resolution;
}

const nav_msgs::msg::OccupancyGrid &MapProcessor::getInflatedMap() const {
  return inflated_map_;
}
