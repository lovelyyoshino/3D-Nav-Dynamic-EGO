#pragma once

#include "nav3d/planner/i_path_searcher.h"

namespace nav3d::planner {

class Jps3D final : public IPathSearcher {
public:
  SearchResult search(
    const map::IMap& map,
    const common::Point3D& start,
    const common::Point3D& goal,
    const SearchOptions& options) const override;
};

}  // namespace nav3d::planner
