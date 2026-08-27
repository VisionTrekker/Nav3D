#pragma once

#include <algorithm>
#include <cmath>

namespace robot_rl_simulation {

struct SafetyLimits {
  double max_linear_x{1.0};
  double max_linear_y{0.3};
  double max_angular_z{1.0};

  [[nodiscard]] bool is_valid() const {
    return std::isfinite(max_linear_x) && max_linear_x >= 0.0 &&
           std::isfinite(max_linear_y) && max_linear_y >= 0.0 &&
           std::isfinite(max_angular_z) && max_angular_z >= 0.0;
  }
};

struct Command {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

inline Command sanitize_command(const Command &command, const SafetyLimits &limits) {
  if (!limits.is_valid()) {
    return {};
  }

  const auto finite_or_zero = [](double value) {
    return std::isfinite(value) ? value : 0.0;
  };
  return {
    std::clamp(finite_or_zero(command.linear_x), -limits.max_linear_x, limits.max_linear_x),
    std::clamp(finite_or_zero(command.linear_y), -limits.max_linear_y, limits.max_linear_y),
    std::clamp(finite_or_zero(command.angular_z), -limits.max_angular_z, limits.max_angular_z),
  };
}

}  // namespace robot_rl_simulation
