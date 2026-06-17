#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace nav3d::common {

template <typename T>
class Result {
public:
  static Result success(T value) { return Result(std::move(value)); }
  static Result failure(std::string error) { return Result(std::move(error)); }

  bool ok() const { return ok_; }

  const T& value() const
  {
    if (!ok_) {
      throw std::logic_error("attempted to read failed Result value");
    }
    return value_;
  }

  T& value()
  {
    if (!ok_) {
      throw std::logic_error("attempted to read failed Result value");
    }
    return value_;
  }

  const std::string& error() const { return error_; }

private:
  explicit Result(T value) : ok_(true), value_(std::move(value)) {}
  explicit Result(std::string error) : ok_(false), error_(std::move(error)) {}

  bool ok_ = false;
  T value_{};
  std::string error_;
};

}  // namespace nav3d::common
