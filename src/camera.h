#ifndef GROUP_10_CAMERA_H
#define GROUP_10_CAMERA_H

#include <opencv2/core/mat.hpp>

#include "constants.h"

// Camera parameters

static const cv::Point2f FOCAL_LENGTH = { 328.88704246f, 328.23230449f }; // millimeters
static const cv::Point2f DISTORTION_CENTER = { 261.99543697f, 218.7788551f }; // pixels from top left

static const cv::Mat CAMERA_MATRIX = (cv::Mat_<float>(3, 3) << 
    FOCAL_LENGTH.x,              0, DISTORTION_CENTER.x,
                 0, FOCAL_LENGTH.y, DISTORTION_CENTER.y,
                 0,              0,                   1);

static const cv::Mat DISTORTION_COEFFS = (cv::Mat_<float>(4, 1) << 
    -0.34929476, 0.16049764, -0.00051511, 0.00109877 /*, -0.0425097*/);

static const cv::Point2f CAMERA_FOV = { DEG_TO_RAD(125.0), DEG_TO_RAD(85.0) }; // radians

// static const cv::Point CAMERA_FOV = {
//     2 * atan2(IMAGE_SIZE.x, 2 * FOCAL_LENGTH.x),
//     2 * atan2(IMAGE_SIZE.y, 2 * FOCAL_LENGTH.y)
// }; // radians

static const cv::Point2f HALF_CAMERA_FOV = {
    CAMERA_FOV.x / 2.0f, CAMERA_FOV.y / 2.0f
}; // radians

#define CAMERA_TILT 18.9f // degrees

#endif