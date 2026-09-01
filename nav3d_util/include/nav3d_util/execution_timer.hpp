#pragma once

#include <chrono>

namespace nav3d_util {
class ExecutionTimer {
  public:
    using Clock = std::chrono::high_resolution_clock;
    using nanoseconds = std::chrono::nanoseconds;

    void start() noexcept { start_ = Clock::now(); }
    void end() noexcept { end_ = Clock::now(); }

    [[nodiscard]] nanoseconds elapsed_time() const noexcept { return end_ - start_; }

    [[nodiscard]] double elapsed_time_in_seconds() const noexcept {
        return std::chrono::duration<double>(end_ - start_).count();
    }

  private:
    Clock::time_point start_{};
    Clock::time_point end_{};
};
} // namespace nav3d_util
