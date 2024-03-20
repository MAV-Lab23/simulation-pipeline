#include "group_10_obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>

#define PRINT(string,...) fprintf(stderr, "[obstacle_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#ifndef GROUP_10_OBSTACLE_DETECTOR_FPS
#define GROUP_10_OBSTACLE_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID ABI_BROADCAST
#endif


struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id);

struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  /*
    TODO: PROCESS img with OpenCV
  */

  // TODO: Determine obstacle coordinates.
  int32_t x = 69;
  int32_t y = 70;

  // TODO: Add OBSTACLE_POINT_DETECTION to paparazzi/conf/abi.xml
  AbiSendMsgGROUP_10_OBSTACLE_POINT_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, x, y);

  return img;
}

void group_10_obstacle_detector_init(void)
{
#ifdef GROUP_10_OBSTACLE_DETECTOR_CAMERA
  cv_add_to_device(&GROUP_10_OBSTACLE_DETECTOR_CAMERA, obstacle_detector, GROUP_10_OBSTACLE_DETECTOR_FPS, 0);
#endif
}

void group_10_obstacle_detector_periodic(void)
{

}