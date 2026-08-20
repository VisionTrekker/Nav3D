#include <gtest/gtest.h>
#include "scan_context_loop/SCManager.h"

TEST(SCManager, MakeAndDetectReturnsValidLoopID) {
  SCManager sc;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = 1000; cloud.height = 1; cloud.is_dense = true;
  for (size_t i = 0; i < 1000; ++i) {
    pcl::PointXYZI p; p.x = i*0.01f; p.y = 0; p.z = 0; p.intensity = 100;
    cloud.push_back(p);
  }
  // Insert 31+ identical scans so the loop detector has enough history
  // to bypass its NUM_EXCLUDE_RECENT guard and exercise the detection path.
  for (int i = 0; i < 35; ++i) {
    sc.makeAndSaveScancontextAndKeys(cloud);
  }
  int loop_id = -1; float yaw_diff = 0;
  sc.detectLoopClosureID(34, loop_id, yaw_diff);
  EXPECT_GE(loop_id, 0);
}
