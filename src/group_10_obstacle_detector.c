#include "group_10_obstacle_detector.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>

#ifdef GROUP_10_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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

struct image_t *obstacle_detector(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  /*
    TODO: PROCESS img with OpenCV
  */
  //PRINT("Got image from drone (width: %d, height: %d) \n", img->w, img->h);

#ifdef GROUP_10_OPENCV
  cv::Mat mat;
  
  PRINT("Made matrix with size: %d \n", mat.size);
#endif
  
  DroneState drone_state = getDroneState();

  // TODO: Determine obstacle coordinates and set them here.
  float x = drone_state.optitrack_pos.x;
  float y = drone_state.optitrack_pos.y;

  // Passing obstacle coordinates to other thread via Abi.
  AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, x, y);

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
