#pragma once

#include <stdexcept>
#include <string>

namespace nav3d_core {

class PlannerException : public std::runtime_error {
  public:
    explicit PlannerException(const std::string& description) : std::runtime_error(description) {}
};

class InvalidPlanner : public PlannerException {
  public:
    explicit InvalidPlanner(const std::string& description) : PlannerException(description) {}
};

class StartOccupied : public PlannerException {
  public:
    explicit StartOccupied(const std::string& description) : PlannerException(description) {}
};

class GoalOccupied : public PlannerException {
  public:
    explicit GoalOccupied(const std::string& description) : PlannerException(description) {}
};

class StartOutsideMapBounds : public PlannerException {
  public:
    explicit StartOutsideMapBounds(const std::string& description) : PlannerException(description) {}
};

class GoalOutsideMapBounds : public PlannerException {
  public:
    explicit GoalOutsideMapBounds(const std::string& description) : PlannerException(description) {}
};

class NoValidPathCouldBeFound : public PlannerException {
  public:
    explicit NoValidPathCouldBeFound(const std::string& description) : PlannerException(description) {}
};

class PlannerTimedOut : public PlannerException {
  public:
    explicit PlannerTimedOut(const std::string& description) : PlannerException(description) {}
};

class PlannerTFError : public PlannerException {
  public:
    explicit PlannerTFError(const std::string& description) : PlannerException(description) {}
};

class NoViapointsGiven : public PlannerException {
  public:
    explicit NoViapointsGiven(const std::string& description) : PlannerException(description) {}
};

class PlannerCancelled : public PlannerException {
  public:
    explicit PlannerCancelled(const std::string& description) : PlannerException(description) {}
};

} // namespace nav3d_core
