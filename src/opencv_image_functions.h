#ifndef OPENCV_IMAGE_FUNCTIONS_H
#define OPENCV_IMAGE_FUNCTIONS_H

#ifdef GROUP_10_OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * Converts cv::Mat with three channels to a YUV422 image.
 * Note that the rgb function first converts to YUV, and then to YUV422 making
 * this function slower than coloryuv_opencv_to_yuv422.
 */
void colorbgr_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

/**
 * Converts cv::Mat with three channels YUV to a YUV422 image.
 */
void coloryuv_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

/**
 * Converts cv::Mat with one to a YUV422 image.
 * The U and V channels are set to 127.
 */
void grayscale_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

//void uyvy_opencv_to_yuv_opencv(cv::Mat image, cv::Mat image_in, int width, int height);

void yuv_opencv_to_yuv422(cv::Mat image, char *img, int width, int height);

#endif

#endif
