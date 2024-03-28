#include "opencv_wrapper.h"

#ifdef GROUP_10_OPENCV

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std.h"
#include "modules/core/abi.h"

#include "image_processing.h"

// FOR GRID VISUALIZATION
#include "navigation.h"
#include "draw.h"

int image_process_loops = 0;
float probabilities[GRID_LENGTH] = { 0 };
int timers[GRID_LENGTH] = { 0 };

#ifndef PRINT
#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

int parseImage(char *img, int width, int height, const DroneState state)
{
  //PRINT("5Got image with size %i, %i \n", width, height);
  // Convert paparazzi image into OpenCV image.
  cv::Mat M(height, width, CV_8UC2, img);
  cv::Mat image;

  // Color image example
  // Convert the image to an OpenCV Mat
  cv::cvtColor(M, image, CV_YUV2BGR_Y422);

  cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);

  // TODO: Fix this. Somehow processImageForObjects or getGridPoints is returning an empty array.
  // I suspect it has something to do with the format of M being incorrect (should be BGR).
  std::vector<cv::Point2f> points = processImageForObjects(image);

  std::vector<cv::Point> u_grid_points = getGridPoints(state, points, M.size(), true, false, false);

  for (size_t i = 0; i < u_grid_points.size(); ++i) {
    //PRINT("Found grid point %i, %i \n", u_grid_points[i].x, u_grid_points[i].y);
    // TODO: Check that this works from a cpp file.
    // Passing obstacle coordinates to other thread via Abi.
    // PRINT("(%d, %d),", u_grid_points[i].x, u_grid_points[i].y);
    AbiSendMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, u_grid_points[i].x, u_grid_points[i].y);
  }

  if (WRITE_REALTIME_PROCESSING_IMAGES && video_capture_record_video) {
    int point_radius = 2;
    for (size_t i = 0; i < points.size(); i++) {
      //PRINT("Drawing point to image: %i, %i \n", (int)points[i].x, (int)points[i].y);
      cv::circle(image, {(int)points[i].x, (int)points[i].y}, point_radius, cv::Scalar(255, 0, 0), -1);
    }

    // Show real time processed video.
    writeImage(image, "processed_images");
  }

  if (WRITE_REALTIME_PROCESSING_IMAGES && video_capture_record_video) {
    cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);
    grid.setTo(cv::Scalar(255, 255, 255));

    drawCarpet(grid);
    //drawObstacles(grid, obstacles);
    
    drawDrone(grid, state);

    int point_radius = 2;
    for (size_t i = 0; i < u_grid_points.size(); i++) {
      cv::circle(grid, { (int)u_grid_points[i].x, (int)u_grid_points[i].y }, point_radius, cv::Scalar(255, 128, 128), -1);
    }

    for (int j = 0; j < GRID_SIZE.y; j++)
    {
      int offset = j * GRID_SIZE.x;
          for (int i = 0; i < GRID_SIZE.x; i++)
          {
        int index = i + offset;
        int timer = getTimer(index);
              if (timer > 0) {
                  cv::circle(grid, cv::Point(i, j), 1, cv::Scalar(0, 0, 255), -1);
              }
          }
    }

    // Show real time grid.
    writeImage(grid, "grid_images");
  }

  // Not needed due to imwrite.
  // Convert back to YUV422 and put it in place of the original image
  //colorbgr_opencv_to_yuv422(image, img, width, height);
  image_process_loops++;
  return 0;
}

#endif
