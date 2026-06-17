#include "nav3d/map/pcd_loader.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace nav3d::map {
namespace {

struct PcdHeader {
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::size_t points = 0;
  std::string data;
};

std::vector<std::string> split(const std::string& line)
{
  std::istringstream input(line);
  std::vector<std::string> tokens;
  std::string token;
  while (input >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

template <typename T>
T readScalar(const char* data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

double parseBinaryValue(const char* data, int size, char type)
{
  if (type == 'F' && size == 4) {
    return static_cast<double>(readScalar<float>(data));
  }
  if (type == 'F' && size == 8) {
    return readScalar<double>(data);
  }
  if (type == 'I' && size == 4) {
    return static_cast<double>(readScalar<std::int32_t>(data));
  }
  if (type == 'U' && size == 4) {
    return static_cast<double>(readScalar<std::uint32_t>(data));
  }
  return 0.0;
}

}  // namespace

common::Result<PointCloud> PcdLoader::load(const std::string& path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return common::Result<PointCloud>::failure("failed to open PCD file: " + path);
  }

  PcdHeader header;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto tokens = split(line);
    if (tokens.empty()) {
      continue;
    }
    const std::string key = tokens.front();
    if (key == "FIELDS") {
      header.fields.assign(tokens.begin() + 1, tokens.end());
    } else if (key == "SIZE") {
      for (auto it = tokens.begin() + 1; it != tokens.end(); ++it) {
        header.sizes.push_back(std::stoi(*it));
      }
    } else if (key == "TYPE") {
      for (auto it = tokens.begin() + 1; it != tokens.end(); ++it) {
        header.types.push_back(it->empty() ? 'F' : (*it)[0]);
      }
    } else if (key == "COUNT") {
      for (auto it = tokens.begin() + 1; it != tokens.end(); ++it) {
        header.counts.push_back(std::stoi(*it));
      }
    } else if (key == "POINTS") {
      header.points = static_cast<std::size_t>(std::stoull(tokens.at(1)));
    } else if (key == "DATA") {
      header.data = tokens.at(1);
      break;
    }
  }

  if (header.fields.empty() || header.points == 0 || header.data.empty()) {
    return common::Result<PointCloud>::failure("invalid or incomplete PCD header: " + path);
  }
  if (header.counts.empty()) {
    header.counts.assign(header.fields.size(), 1);
  }
  if (header.sizes.size() != header.fields.size() || header.types.size() != header.fields.size() ||
      header.counts.size() != header.fields.size()) {
    return common::Result<PointCloud>::failure("PCD field metadata length mismatch: " + path);
  }

  std::unordered_map<std::string, std::size_t> field_index;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    field_index[header.fields[i]] = i;
  }
  if (field_index.find("x") == field_index.end() || field_index.find("y") == field_index.end() ||
      field_index.find("z") == field_index.end()) {
    return common::Result<PointCloud>::failure("PCD must contain x y z fields: " + path);
  }

  PointCloud cloud;
  cloud.points.reserve(header.points);

  if (header.data == "ascii") {
    for (std::size_t i = 0; i < header.points && std::getline(file, line); ++i) {
      const auto tokens = split(line);
      if (tokens.size() < header.fields.size()) {
        return common::Result<PointCloud>::failure("PCD ascii row has too few fields: " + path);
      }
      cloud.points.push_back({
        std::stod(tokens[field_index["x"]]),
        std::stod(tokens[field_index["y"]]),
        std::stod(tokens[field_index["z"]]),
      });
    }
    return common::Result<PointCloud>::success(std::move(cloud));
  }

  if (header.data != "binary") {
    return common::Result<PointCloud>::failure("unsupported PCD DATA mode: " + header.data);
  }

  std::vector<std::size_t> offsets(header.fields.size(), 0);
  std::size_t stride = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    offsets[i] = stride;
    stride += static_cast<std::size_t>(header.sizes[i] * header.counts[i]);
  }

  std::vector<char> row(stride);
  for (std::size_t i = 0; i < header.points; ++i) {
    file.read(row.data(), static_cast<std::streamsize>(row.size()));
    if (!file) {
      return common::Result<PointCloud>::failure("PCD binary payload ended early: " + path);
    }
    const auto read_field = [&](const std::string& name) {
      const std::size_t idx = field_index.at(name);
      return parseBinaryValue(row.data() + offsets[idx], header.sizes[idx], header.types[idx]);
    };
    cloud.points.push_back({read_field("x"), read_field("y"), read_field("z")});
  }

  return common::Result<PointCloud>::success(std::move(cloud));
}

}  // namespace nav3d::map
