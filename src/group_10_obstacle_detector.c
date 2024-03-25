#include "group_10_obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>

#ifdef GROUP_10_OPENCV
#include "opencv_wrapper.h"
#endif

#include "drone.h"

#define PRINT(string,...) fprintf(stderr, "[obstacle_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#ifndef GROUP_10_OBSTACLE_DETECTOR_FPS
#define GROUP_10_OBSTACLE_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
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

void group_10_obstacle_detector_init(void) {
#ifdef GROUP_10_OBSTACLE_DETECTOR_CAMERA
  cv_add_to_device(&GROUP_10_OBSTACLE_DETECTOR_CAMERA, obstacle_detector, GROUP_10_OBSTACLE_DETECTOR_FPS, 0);
#endif
}

void group_10_obstacle_detector_periodic(void) {}
