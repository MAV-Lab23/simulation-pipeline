#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main() {
	cv::Mat a = cv::Mat::eye(640, 640, CV_8UC1)*255;
	cv::imshow("1", a);
	cv::waitKey(0);
	printf("The program is working!\n");
	return 0;
}