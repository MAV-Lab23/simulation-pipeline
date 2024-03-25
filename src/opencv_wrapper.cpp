#include "opencv_wrapper.h"

#define PRINT(string,...) fprintf(stderr, "[obstacle_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

#include "modules/core/abi.h"
#include "image_processing.h"

int opencv_wrapper(char *img, int width, int height, const DroneState& state) {
  // Convert paparazzi image into OpenCV image.
  cv::Mat M(width, height, CV_8UC3, img);

  cv::cvtColor(M, M, COLOR_YUV2BGR_UYVY);

  std::vector<cv::Point2f> points = processImageForObjects(M);
  std::vector<cv::Point> grid_points = getGridPoints(M.size(), state, points, true, true, true);

  for (size_t i = 0; i < grid_points.size(); ++i) {
    // TODO: Check that this works from a cpp file.
    // Passing obstacle coordinates to other thread via Abi.
    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, grid_points[i].x, grid_points[i].y);
  }

  colorbgr_opencv_to_yuv422(M, img, width, height);

  return 0;
}