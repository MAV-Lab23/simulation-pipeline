#include "opencv_wrapper.h"

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std.h"
#include "modules/core/abi.h"

#include "transform.h"
#include "navigation.h"
#include "drone.h"
#include "image_processing.h"

// FOR GRID VISUALIZATION
#include "draw.h"

int image_process_loops = 0;
float probabilities[GRID_LENGTH] = { 0 };
int timers[GRID_LENGTH] = { 0 };

#define UNDISTORT true
#define SUBTRACT_TIMERS true

#ifndef PRINT
#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

void addNavigationObstacle(int x, int y) {
	// Convert 2D to 1D coordinate.
  addGridElement(x + GRID_WIDTH * y);
  printGridElement(x, y);
}

float getNavigationHeading() {
  cv::Mat grid;
  DroneState state = getDroneState();
  return updateNavigation(state, grid, false, SUBTRACT_TIMERS);
}

static cv::Mat convertImage(char* img, int width, int height) {
  cv::Mat M(height, width, CV_8UC2, img);
  cv::Mat image;
  cv::cvtColor(M, image, CV_YUV2BGR_Y422);
  cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
  return image;
} 

void parseImage(char* img, int width, int height) {
  const DroneState state = getDroneState();
  cv::Mat image = convertImage(img, width, height);

  bool draw_outputs = WRITE_REALTIME_PROCESSING_IMAGES && video_capture_record_video;
  
  cv::Mat grid;
  std::vector<cv::Point> grid_points = detectObstacles(image, state, NULL, UNDISTORT, {}, draw_outputs, &grid);

  // Send obstacle grid points to avoider.c via abi.
  for (const cv::Point& gp : grid_points) {
    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, (float)gp.x, (float)gp.y);
  }

  if (draw_outputs) {
    // Show real time processed video.
    writeImage(image, "processed_images");
    // Show real time grid.
    writeImage(grid, "grid_images");
  }

  // Conversion back to YUV422 not required due to writeImage.
  // Convert back to YUV422 and put it in place of the original image
  //colorbgr_opencv_to_yuv422(image, img, width, height);
  image_process_loops++;
}