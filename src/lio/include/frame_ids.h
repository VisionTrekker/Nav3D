/*
 * frame_ids.h — shared frame ID macros for the nav3d boundary adapter.
 *
 * All frame_id string literals in src/node/*.cpp must resolve to these
 * macros so that the nav3d public frame convention (odom / base_link /
 * lidar_frame) can be changed in one place.
 *
 * TF chain: map → odom → base_link → lidar_frame
 */
#ifndef FRAME_IDS_H
#define FRAME_IDS_H

#define FRAME_PARENT_ID "odom"
#define FRAME_IMU_ID    "base_link"
#define FRAME_BODY_ID   "base_link"
#define FRAME_LIDAR_ID  "lidar_frame"

#endif  // FRAME_IDS_H
