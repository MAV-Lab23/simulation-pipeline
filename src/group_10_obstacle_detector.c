#include "group_10_obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>

#ifdef GROUP_10_OPENCV
#include "opencv_wrapper.h"
#endif

#include "drone.h"

#define PRINT(string,...) fprintf(stderr, "[detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#ifndef GROUP_10_OBSTACLE_DETECTOR_FPS
///< Default FPS (zero means run at camera fps)
#define GROUP_10_OBSTACLE_DETECTOR_FPS 0
#endif

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID 50
#endif

struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id);

struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  //PRINT("Got image from drone (width: %d, height: %d) \n", img->w, img->h);
#ifdef GROUP_10_OPENCV
  if (img->type == IMAGE_YUV422) {
    DroneState state = getDroneState();
    // Call OpenCV (C++ from paparazzi C function)
    opencv_wrapper((char*)img->buf, img->w, img->h, state);
  }
#endif
  return img;
}

#define OBSTACLE_COUNT 5

static float obstacles_array[OBSTACLE_COUNT][2] = {
  {  1.5f, -2.5f },
  { -1.8f, -3.4f },
  {  0.6f,  0.7f },
  { -1.8f,  0.5f },
  {  2.8f,  2.5f }
};

static void addTestObstacles() {
  // Assume all known obstacles start from the ground and go infinitely up.
  const float OBSTACLE_Z = 0;

  // DEBUG / Navigation testing: Send known obstacles via Abi.
  for (size_t i = 0; i < OBSTACLE_COUNT; i++)
  {
    Vector3f obstacle_pos = { obstacles_array[i][0], obstacles_array[i][1], OBSTACLE_Z };

    Vector3f obstacle_rot_pos = rotateObstaclePos(obstacle_pos);

    Vector2i obstacle_grid_pos = getObstacleGridPos(obstacle_rot_pos);

    if (!validVectorInt(obstacle_grid_pos)) continue;

    PRINT("ADDING OBSTACLE WITH GRID POS: (%i, %i)\n", obstacle_grid_pos.x, obstacle_grid_pos.y);

    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, obstacle_grid_pos.x, obstacle_grid_pos.y);
  }
}

void group_10_obstacle_detector_init(void) {
#ifdef GROUP_10_OBSTACLE_DETECTOR_CAMERA
  cv_add_to_device(&GROUP_10_OBSTACLE_DETECTOR_CAMERA, obstacle_detector, GROUP_10_OBSTACLE_DETECTOR_FPS, 0);
#endif
}

void group_10_obstacle_detector_periodic(void) {
  static bool added_obstacles = false;
  if (!added_obstacles) {
    addTestObstacles();
    added_obstacles = true;
  }
}
