#include "opencv_wrapper.h"

#define PRINT(string,...) fprintf(stderr, "[opencv_wrapper->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

#include "modules/core/abi.h"
#include "image_processing.h"

int convertYUV422toBGR(const char* yuv_data, int width, int height, cv::Mat& bgr_image) {

  // Check for valid input
  if (!yuv_data || width <= 0 || height <= 0) {
    return -1;
  }

  // Create a Mat from the YUV422 char pointer
  cv::Mat yuv_image(height, width, CV_8UC2, (void*)yuv_data);

  // Separate Y, U, and V channels (assuming U and V are subsampled by 2 horizontally)
  cv::Mat Y = yuv_image;
  cv::Mat U(height, width / 2, CV_8UC1, yuv_image.ptr<uchar>() + 1);
  cv::Mat V(height, width / 2, CV_8UC1, yuv_image.ptr<uchar>() + width);

  // Upsample U and V channels (nearest neighbor is a simple option)
  cv::resize(U, U, cv::Size(width, height), cv::INTER_NEAREST);
  cv::resize(V, V, cv::Size(width, height), cv::INTER_NEAREST);

  // Convert YUV to BGR
  cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_UYVY);  // Assuming YUYV order

  return 0;
}

int opencv_wrapper(char *img, int width, int height, const DroneState state) {
  // Convert paparazzi image into OpenCV image.
  cv::Mat M(width, height, CV_8UC2, img);
  
  cv::cvtColor(M, M, CV_YUV2BGR_Y422);

  // TODO: Fix this. Somehow processImageForObjects or getGridPoints is returning an empty array.
  // I suspect it has something to do with the format of M being incorrect (should be BGR).
  std::vector<cv::Point2f> points = processImageForObjects(M);
  std::vector<cv::Point> grid_points = getGridPoints(M.size(), state, points, true, true, true);

  for (size_t i = 0; i < grid_points.size(); ++i) {
    // TODO: Check that this works from a cpp file.
    // Passing obstacle coordinates to other thread via Abi.
    //PRINT("(%d, %d),", grid_points[i].x, grid_points[i].y);
    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, grid_points[i].x, grid_points[i].y);
  }
  //PRINT("\n");

  //cv::Mat isolatedFloor, mask;
	//isolateGreenFloor(M, isolatedFloor, mask);

  cv::cvtColor(M, M, CV_BGR2YUV);

  coloryuv_opencv_to_yuv422(M, img, width, height);

  return 0;
}