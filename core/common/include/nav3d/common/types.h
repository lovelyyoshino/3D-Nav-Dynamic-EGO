#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <vector>

namespace nav3d::common {

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3D() = default;
  Point3D(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

  bool operator==(const Point3D& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const Point3D& other) const { return !(*this == other); }
};

inline Point3D operator+(const Point3D& a, const Point3D& b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Point3D operator-(const Point3D& a, const Point3D& b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Point3D operator*(const Point3D& p, double scale)
{
  return {p.x * scale, p.y * scale, p.z * scale};
}

inline Point3D operator/(const Point3D& p, double scale)
{
  return {p.x / scale, p.y / scale, p.z / scale};
}

inline double norm(const Point3D& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline double distance(const Point3D& a, const Point3D& b)
{
  return norm(a - b);
}

struct BoundingBox {
  Point3D min;
  Point3D max;
  bool valid = false;

  void expandToInclude(const Point3D& p)
  {
    if (!valid) {
      min = p;
      max = p;
      valid = true;
      return;
    }
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
  }
};

inline std::ostream& operator<<(std::ostream& os, const Point3D& p)
{
  os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return os;
}

using Path3D = std::vector<Point3D>;

}  // namespace nav3d::common
