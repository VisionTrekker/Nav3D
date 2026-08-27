#include <gtest/gtest.h>

#include <limits>

#include "robot_rl_simulation/safety_filter.hpp"

namespace robot_rl_simulation {

TEST(SafetyFilter, ClampsConfiguredCommandLimits) {
  const auto result = sanitize_command({2.0, -1.0, 3.0}, SafetyLimits{});
  EXPECT_DOUBLE_EQ(result.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result.linear_y, -0.3);
  EXPECT_DOUBLE_EQ(result.angular_z, 1.0);
}

TEST(SafetyFilter, HonorsCustomLimits) {
  const auto result = sanitize_command({0.8, -0.4, 0.5}, {0.5, 0.2, 0.25});
  EXPECT_DOUBLE_EQ(result.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.linear_y, -0.2);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.25);
}

TEST(SafetyFilter, RejectsInvalidLimits) {
  const auto result = sanitize_command({0.5, 0.1, 0.25},
                                       {-1.0, 0.3, std::numeric_limits<double>::infinity()});
  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}
TEST(SafetyFilter, NonFiniteInputBecomesZero) {
  const auto result = sanitize_command(
      {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()},
      SafetyLimits{});
  EXPECT_DOUBLE_EQ(result.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(result.angular_z, 0.0);
}

}  // namespace robot_rl_simulation
