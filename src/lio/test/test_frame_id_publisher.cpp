/*
 * Frame-id boundary-adapter meta-test.
 *
 * Verifies that publish.cpp no longer contains the old Elevator-LIO internal
 * frame_id string literals, and that the new nav3d public names are compiled
 * into the binary.
 *
 * Strategy:
 *   Part A — Source inspection: grep publish.cpp for old frame names
 *            (old names appear as string literals, not identifiers)
 *   Part B — Binary inspection: `strings` the binary for new frame names
 *
 * Before patch: "world","IMU","body","lidar" in publish.cpp source  -> FAIL
 * After  patch: "odom","base_link","lidar_frame" in binary         -> PASS
 */

#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

bool GrepFile(const std::string& path, const std::string& pattern, std::string& matched_lines) {
  std::string cmd = "grep -nE '" + pattern + "' '" + path + "' 2>/dev/null";
  char buf[1024];
  FILE* fp = popen(cmd.c_str(), "r");
  if (!fp) return false;
  matched_lines.clear();
  while (std::fgets(buf, sizeof(buf), fp)) matched_lines += buf;
  pclose(fp);
  return !matched_lines.empty();
}

bool StringsContain(const std::string& bin, const std::string& needle) {
  std::string cmd = "strings '" + bin + "' | grep -F '" + needle + "' > /dev/null 2>&1";
  return std::system(cmd.c_str()) == 0;
}

}  // namespace

// Part A: publish.cpp must NOT contain the old Elevator-LIO internal names
// as string literals assigned to frame_id fields.
TEST(FrameIdPublisher, NoOldFrameIdLiteralsInPublishCpp) {
  const std::string src = "/media/lenovo/disk/planner_ws/src/Nav3D/src/lio/src/node/publish.cpp";

  std::string out;

  // "world" as a frame_id string literal — must not appear after patch
  EXPECT_FALSE(GrepFile(src, R"(\.frame_id\s*=\s*"world")", out))
      << "publish.cpp still has: frame_id = \"world\"";

  // "IMU" as a frame_id string literal
  EXPECT_FALSE(GrepFile(src, R"(\.frame_id\s*=\s*"IMU")", out))
      << "publish.cpp still has: frame_id = \"IMU\"";

  // "body" as a frame_id string literal
  EXPECT_FALSE(GrepFile(src, R"(\.frame_id\s*=\s*"body")", out))
      << "publish.cpp still has: frame_id = \"body\"";

  // "lidar" as a frame_id string literal
  EXPECT_FALSE(GrepFile(src, R"(\.frame_id\s*=\s*"lidar")", out))
      << "publish.cpp still has: frame_id = \"lidar\"";
}

// Part B: The compiled binary must contain the new nav3d public names.
TEST(FrameIdPublisher, NewFrameIdConstantsInBinary) {
  const std::string bin = "/media/lenovo/disk/planner_ws/build/lio/lio";

  // "odom" — FRAME_PARENT_ID value
  EXPECT_TRUE(StringsContain(bin, "odom"))
      << "\"odom\" not found in lio binary — is FRAME_PARENT_ID defined?";

  // "base_link" — FRAME_BODY_ID / FRAME_IMU_ID value
  EXPECT_TRUE(StringsContain(bin, "base_link"))
      << "\"base_link\" not found in lio binary — is FRAME_BODY_ID/IMU_ID defined?";

  // "lidar_frame" — FRAME_LIDAR_ID value
  EXPECT_TRUE(StringsContain(bin, "lidar_frame"))
      << "\"lidar_frame\" not found in lio binary — is FRAME_LIDAR_ID defined?";
}
