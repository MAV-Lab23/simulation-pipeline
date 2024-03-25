/*
 * Copyright (C) 2016 Roland Meertens
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/opencv_image_functions.cpp
 *
 * A small library with functions to convert between the Paparazzi used YUV422 arrays
 * and the opencv image functions.
 */

#include "opencv_image_functions.h"

#define PRINT(string,...) fprintf(stderr, "[obstacle_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

// YUV in opencv convert to YUV on Bebop
void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b &c = image.at<cv::Vec3b>(row, col);

      // Set image buffer values
      int i = row * width + col;
      img[2 * i + 1] = c[0]; // y;
      img[2 * i] = col % 2 ? c[1] : c[2]; // u or v
    }
  }
}

/*
void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height)
{
//Turn the opencv RGB colored image back in a YUV colored image for the drone
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      // Extract pixel color from image
      cv::Vec3b c = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_m1 = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_p1 = image_in.at<cv::Vec3b>(row, col);
      if (col > 0) {
        c_m1 = image_in.at<cv::Vec3b>(row, col - 1);
      }
      if (col < width) {
        c_p1 = image_in.at<cv::Vec3b>(row, col + 1);
      }
      image.at<cv::Vec3b>(row, col)[0] = c[1] ;
      image.at<cv::Vec3b>(row, col)[1] = col % 2 ? c[0] : c_m1[0];
      image.at<cv::Vec3b>(row, col)[2] = col % 2 ? c_p1[0] : c[0];

    }
  }
}
*/

void coloryuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 3);

  int nRows = image.rows;
  int nCols = image.cols;

  int byte_index = 0;
  for(int r = 0; r < nRows; ++r) {
    for(int c = 0; c < nCols; ++c) {
      Vec3b yuv = image.at<Vec3b>(r, c);
      if((byte_index % 4) == 0) {
        img[byte_index++] = yuv.val[1]; // U
      } else {
        img[byte_index++] = yuv.val[2]; // V
      }
      img[byte_index++] = yuv.val[0]; // Y
    }
  }
}

void colorbgr_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  // Convert to YUV color space
  cvtColor(image, image, COLOR_BGR2YUV);
  // then call the to color function
  coloryuv_opencv_to_yuv422(image, img, width, height);
}


void grayscale_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  CV_Assert(image.depth() == CV_8U);
  CV_Assert(image.channels() == 1);

  int n_rows = image.rows;
  int n_cols = image.cols;

  // If the image is one block in memory we can iterate over it all at once!
  if (image.isContinuous()) {
    n_cols *= n_rows;
    n_rows = 1;
  }

  // Iterate over the image, setting only the Y value
  // and setting U and V to 127
  int i, j;
  uchar *p;
  int index_img = 0;
  for (i = 0; i < n_rows; ++i) {
    p = image.ptr<uchar>(i);
    for (j = 0; j < n_cols; j++) {
      img[index_img++] = 127;
      img[index_img++] = p[j];


    }
  }
}
