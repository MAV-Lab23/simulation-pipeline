#include "group_10_obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>

#include "opencv_wrapper.h"
#include "obstacle.h"

#ifndef PRINT
#define PRINT(string,...) fprintf(stderr, "[detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

// (zero means run at camera fps)
#ifndef GROUP_10_OBSTACLE_DETECTOR_FPS
#define GROUP_10_OBSTACLE_DETECTOR_FPS 0 
#endif

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID 50
#endif

#define OBSTACLE_COUNT 5

// Predefined obstacles for debugging purposes.
static float obstacles_array[OBSTACLE_COUNT][2] = {
  {  1.5f, -2.5f },
  { -1.8f, -3.4f },
  {  0.6f,  0.7f },
  { -1.8f,  0.5f },
  {  2.8f,  2.5f }
};

struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id);
struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    parseImage((char*)img->buf, img->w, img->h);
  }
  return img;
}

static void addTestObstacles() {
  // DEBUG / Navigation testing: Send known obstacles via Abi.
  for (size_t i = 0; i < OBSTACLE_COUNT; i++) {
    int x, y;
    transformObstacleToGrid(obstacles_array[i][0], obstacles_array[i][1], &x, &y);
    if (x == INVALID_POINT || y == INVALID_POINT) continue;
    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, (float)x, (float)y);
  }
}

void group_10_obstacle_detector_init(void) {
#ifdef GROUP_10_OBSTACLE_DETECTOR_CAMERA
  cv_add_to_device(&GROUP_10_OBSTACLE_DETECTOR_CAMERA, obstacle_detector, GROUP_10_OBSTACLE_DETECTOR_FPS, 0);
#endif
}

void group_10_obstacle_detector_periodic(void) {
  // Avoider.c abi bind is called after group_10_obstacle_detector_init so this is what I did.
  // Perhaps the start="" flag of the module would have been better here?
  // static bool added_obstacles = false;
  // if (!added_obstacles) {
  //   addTestObstacles();
  //   added_obstacles = true;
  // }
}
