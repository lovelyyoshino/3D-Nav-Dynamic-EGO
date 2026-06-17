#include "pcl_pcd_loader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <utility>

namespace nav3d::tools {

common::Result<map::PointCloud> PclPcdLoader::load(const std::string& path)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  const int status = pcl::io::loadPCDFile<pcl::PointXYZ>(path, pcl_cloud);
  if (status < 0) {
    return common::Result<map::PointCloud>::failure("failed to load PCD with PCL: " + path);
  }

  map::PointCloud cloud;
  cloud.points.reserve(pcl_cloud.points.size());
  for (const auto& point : pcl_cloud.points) {
    cloud.points.push_back({
      static_cast<double>(point.x),
      static_cast<double>(point.y),
      static_cast<double>(point.z),
    });
  }
  return common::Result<map::PointCloud>::success(std::move(cloud));
}

}  // namespace nav3d::tools
