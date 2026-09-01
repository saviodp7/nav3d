#pragma once

#include <stdexcept>
#include <string>

namespace nav3d_core {

class SmootherException : public std::runtime_error {
  public:
    explicit SmootherException(const std::string& description) : std::runtime_error(description) {}
};

class InvalidSmoother : public SmootherException {
  public:
    explicit InvalidSmoother(const std::string& description) : SmootherException(description) {}
};

class InvalidPath : public SmootherException {
  public:
    explicit InvalidPath(const std::string& description) : SmootherException(description) {}
};

class SmootherTimedOut : public SmootherException {
  public:
    explicit SmootherTimedOut(const std::string& description) : SmootherException(description) {}
};

class SmoothedPathInCollision : public SmootherException {
  public:
    explicit SmoothedPathInCollision(const std::string& description) : SmootherException(description) {}
};

class FailedToSmoothPath : public SmootherException {
  public:
    explicit FailedToSmoothPath(const std::string& description) : SmootherException(description) {}
};

} // namespace nav3d_core
