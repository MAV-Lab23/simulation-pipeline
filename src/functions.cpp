#include "functions.h"

#include <opencv2/highgui.hpp>
#include <iostream>

void show(const cv::Mat &img){
	cv::imshow("1", img);
	std::cout << "OpenCV works" << std::endl;
}