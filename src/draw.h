#pragma once

#ifndef IN_PAPARAZZI

#include <opencv2/opencv.hpp>

void initDrawingWindows() {
    cv::namedWindow("Image");
    cv::namedWindow("Final");
    cv::namedWindow("Floor");
    cv::namedWindow("Filtered");
    cv::namedWindow("Grid");
    cv::namedWindow("Intermediate1");
    cv::namedWindow("Intermediate2");
    cv::namedWindow("Intermediate3");

    cv::moveWindow("Image", 0, 0);
    cv::moveWindow("Final", 0, 275);
    cv::moveWindow("Floor", 530, 0);
    cv::moveWindow("Filtered", 530, 275);
    cv::moveWindow("Grid", 530 * 2, 0);
    cv::moveWindow("Intermediate1", 0, 275 * 2 + 9);
    cv::moveWindow("Intermediate2", 530, 275 * 2 + 9);
    cv::moveWindow("Intermediate3", 530 * 2, 275 * 2 + 9);
}

void destroyDrawingWindows() {
    cv::destroyAllWindows();
}

#endif