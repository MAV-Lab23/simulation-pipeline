#include "opencv_wrapper.h"

#define PRINT(string,...) fprintf(stderr, "[obstacle_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

using namespace cv;
using namespace std;

struct contour_estimation cont_est;
struct contour_threshold cont_thres;

RNG rng(12345);


cv::Mat isolateGreenFloor(const cv::Mat& image, cv::Mat& isolatedFloor, cv::Mat& mask) {
	// Convert image to HSV
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

	// Define range of green color in HSV
	// Threshold the HSV image to get only green color.
	cv::inRange(hsvImage, cv::Scalar(20, 0, 0), cv::Scalar(80, 255, 220), mask);

	// Erode and dilate to remove noise
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

	// Bitwise-AND mask and original image
	cv::bitwise_and(image, image, isolatedFloor, mask);

	return mask;
}

int opencv_wrapper(char *img, int width, int height) {
  cv::Mat M(width, height, CV_8UC2, img);
  
  cvtColor(M, M, CV_YUV2RGB_Y422);
  cvtColor(M, M, CV_RGB2YUV);

	cv::Mat isolatedFloor, mask;
  isolateGreenFloor(M, isolatedFloor, mask);

  colorbgr_opencv_to_yuv422(isolatedFloor, img, width, height);

  return 0;
}

/*
int opencv_wrapper(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(width, height, CV_8UC2, img); // original
  Mat image, edge_image, thresh_image;

  // convert UYVY in paparazzi to YUV in opencv
  cvtColor(M, M, CV_YUV2RGB_Y422);
  cvtColor(M, M, CV_RGB2YUV);

  // Threshold all values within the indicted YUV values.
  inRange(M, cv::Scalar(20, 0, 0), cv::Scalar(80, 255, 220), thresh_image);

  /// Find contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  edge_image = thresh_image;
  int edgeThresh = 35;
  Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
  findContours(edge_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  // Get the moments
  vector<Moments> mu(contours.size());
  for (unsigned int i = 0; i < contours.size(); i++) {
    mu[i] = moments(contours[i], false);
  }

  //  Get the mass centers:
  vector<Point2f> mc(contours.size());
  for (unsigned int i = 0; i < contours.size(); i++) {
    mc[i] = Point2f(mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00);
  }

  /// Draw contours
  Mat drawing = Mat::zeros(edge_image.size(), CV_8UC3);
  for (unsigned int i = 0; i < contours.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    circle(drawing, mc[i], 4, color, -1, 8, 0);
  }

  // Find Largest Contour
  int largest_contour_index = 0;
  int largest_area = 0;
  Rect bounding_rect;

  // iterate through each contour.
  for (unsigned int i = 0; i < contours.size(); i++) {
    //  Find the area of contour
    double a = contourArea(contours[i], false);
    if (a > largest_area) {
      largest_area = a;
      // Store the index of largest contour
      largest_contour_index = i;
      // Find the bounding rectangle for biggest contour
      bounding_rect = boundingRect(contours[i]);
    }
  }
  Scalar color(255, 255, 255);
  // Draw the contour and rectangle
  drawContours(M, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy);

  rectangle(M, bounding_rect,  Scalar(0, 255, 0), 2, 8, 0);

  // some figure can cause there are no largest circles, in this case, do not draw circle
  circle(M, mc[largest_contour_index], 4, Scalar(0, 255, 0), -1, 8, 0);
  Point2f rect_center(bounding_rect.x + bounding_rect.width / 2 , bounding_rect.y + bounding_rect.height / 2);
  circle(image, rect_center, 4, Scalar(0, 0, 255), -1, 8, 0);

  // Convert back to YUV422, and put it in place of the original image
  colorbgr_opencv_to_yuv422(M, img, width, height);

  float contour_distance_est;
  //estimate the distance in X, Y and Z direction
  float area = bounding_rect.width * bounding_rect.height;
  if (area > 28000.) {
    contour_distance_est = 0.1;
  }
  if ((area > 16000.) && (area < 28000.)) {
    contour_distance_est = 0.5;
  }
  if ((area > 11000.) && (area < 16000.)) {
    contour_distance_est = 1;
  }
  if ((area > 3000.) && (area < 11000.)) {
    contour_distance_est = 1.5;
  }
  if (area < 3000.) {
    contour_distance_est = 2.0;
  }
  cont_est.contour_d_x = contour_distance_est;
  float Im_center_w = width / 2.;
  float Im_center_h = height / 2.;
  float real_size = 1.; // real size of the object
  cont_est.contour_d_y = -(rect_center.x - Im_center_w) * real_size / float(bounding_rect.width); // right hand
  cont_est.contour_d_z = -(rect_center.y - Im_center_h) * real_size / float(bounding_rect.height); // point downwards
  return 0;
}
*/