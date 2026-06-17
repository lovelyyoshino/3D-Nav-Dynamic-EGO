#include "nav3d/map/local_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav3d::map {

LocalGrid::LocalGrid(
  double resolution,
  const common::Point3D& min_bound,
  const common::Point3D& max_bound)
  : resolution_(resolution)
{
  if (resolution_ <= 0.0) {
    throw std::invalid_argument("LocalGrid resolution must be positive");
  }
  bounds_.min = min_bound;
  bounds_.max = max_bound;
  bounds_.valid = true;
}

void LocalGrid::markFree(const common::Point3D& p)
{
  setCell(p, CellState::Free);
}

void LocalGrid::markOccupied(const common::Point3D& p)
{
  setCell(p, CellState::Occupied);
}

void LocalGrid::markRayFreeAndOccupied(
  const common::Point3D& origin,
  const common::Point3D& endpoint)
{
  if (!isInBounds(origin) || !isInBounds(endpoint)) {
    throw std::out_of_range("LocalGrid ray endpoint is outside local bounds");
  }

  const GridIndex occupied_endpoint = worldToGrid(endpoint);
  const double length = common::distance(origin, endpoint);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (resolution_ * 0.5))));

  for (int step = 0; step < steps; ++step) {
    const double alpha = static_cast<double>(step) / static_cast<double>(steps);
    const common::Point3D sample = origin + (endpoint - origin) * alpha;
    if (!(worldToGrid(sample) == occupied_endpoint)) {
      setCell(sample, CellState::Free);
    }
  }

  setCell(endpoint, CellState::Occupied);
}

bool LocalGrid::hasObservation(const common::Point3D& p) const
{
  return cells_.find(worldToGrid(p)) != cells_.end();
}

CellState LocalGrid::getCellState(const common::Point3D& p) const
{
  const auto it = cells_.find(worldToGrid(p));
  if (it == cells_.end()) {
    return CellState::Unknown;
  }
  return it->second;
}

void LocalGrid::clear()
{
  cells_.clear();
}

bool LocalGrid::hasOccupiedCells() const
{
  for (const auto& [_, state] : cells_) {
    if (state == CellState::Occupied) {
      return true;
    }
  }
  return false;
}

const std::unordered_map<GridIndex, CellState, GridIndexHash>& LocalGrid::observedCells() const
{
  return cells_;
}

GridIndex LocalGrid::worldToGrid(const common::Point3D& p) const
{
  return {
    static_cast<int>(std::floor(p.x / resolution_ + 1e-9)),
    static_cast<int>(std::floor(p.y / resolution_ + 1e-9)),
    static_cast<int>(std::floor(p.z / resolution_ + 1e-9)),
  };
}

common::Point3D LocalGrid::gridToWorld(const GridIndex& idx) const
{
  return {
    static_cast<double>(idx.x) * resolution_,
    static_cast<double>(idx.y) * resolution_,
    static_cast<double>(idx.z) * resolution_,
  };
}

bool LocalGrid::isOccupied(const common::Point3D& p) const
{
  return getCellState(p) == CellState::Occupied;
}

bool LocalGrid::isFree(const common::Point3D& p) const
{
  return isInBounds(p) && getCellState(p) == CellState::Free;
}

double LocalGrid::getDistance(const common::Point3D& p) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto& [idx, state] : cells_) {
    if (state == CellState::Occupied) {
      best = std::min(best, common::distance(p, gridToWorld(idx)));
    }
  }
  return best;
}

bool LocalGrid::isInBounds(const common::Point3D& p) const
{
  const double eps = 1e-9;
  return p.x >= bounds_.min.x - eps && p.x <= bounds_.max.x + eps &&
         p.y >= bounds_.min.y - eps && p.y <= bounds_.max.y + eps &&
         p.z >= bounds_.min.z - eps && p.z <= bounds_.max.z + eps;
}

double LocalGrid::getResolution() const
{
  return resolution_;
}

common::BoundingBox LocalGrid::getBounds() const
{
  return bounds_;
}

void LocalGrid::setCell(const common::Point3D& p, CellState state)
{
  if (!isInBounds(p)) {
    throw std::out_of_range("LocalGrid observation is outside local bounds");
  }
  cells_[worldToGrid(p)] = state;
}

}  // namespace nav3d::map
