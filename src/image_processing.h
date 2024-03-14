#pragma once

#include <types.h>
#include <constants.h>
#include <utility.h>
#include <opencv2/core/mat.hpp>


#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;

cv::Mat WeightedCombine(const cv::Mat& addition, const cv::Mat& storage, float addition_weight) {
	assert(addition_weight >= 0.0f && addition_weight <= 1.0f);

	float storage_weight = 1.0f - addition_weight;

	return addition_weight * addition + storage_weight * storage;
}


Mat isolateGreenFloor(Mat image, Mat& isolatedFloor, Mat& mask) {
	// Convert image to HSV
	Mat hsvImage;
	cvtColor(image, hsvImage, COLOR_BGR2HSV);

	// Define range of green color in HSV
	Scalar lowerGreen = Scalar(20, 0, 0);
	Scalar upperGreen = Scalar(80, 255, 175);

	// Threshold the HSV image to get only green colors
	inRange(hsvImage, lowerGreen, upperGreen, mask);

	// Erode and dilate to remove noise
	erode(mask, mask, Mat(), Point(-1, -1), 2);
	dilate(mask, mask, Mat(), Point(-1, -1), 2);

	// Bitwise-AND mask and original image
	bitwise_and(image, image, isolatedFloor, mask);

	return mask
}

void detectFloorBorder(Mat processedImage, vector<int>& floorBorder) {
	for (int x = 0; x < processedImage.cols; ++x) {
		for (int y = processedImage.rows - 1; y >= 0; --y) {
			if (processedImage.at<uchar>(y, x) > 0) { // Assuming binary image
				floorBorder[x] = y;
				break;
			}
		}
	}
}

void detectObjectPositions(vector<int>& floorBorder, vector<Point>& objectPositions, int maxSlopeChange = 10, int minPosChange = 10) {
	vector<int> slopes;
	for (size_t i = 1; i < floorBorder.size(); i++) {
		slopes.push_back(floorBorder[i] - floorBorder[i - 1]); // Calculate slope
	}

	for (size_t i = 1; i < slopes.size(); i++) {
		if (abs(slopes[i] - slopes[i - 1]) > maxSlopeChange) {
			if (objectPositions.empty() || int(i - objectPositions.back().x) > minPosChange) {
				objectPositions.push_back(Point(i, floorBorder[i]));
			}
		}
	}
}

void calculateDistancesToObjects(Point imageCenter, vector<Point>& objectPositions, vector<int>& floorBorder, vector<pair<Point, double>>& distances) {
	for (size_t i = 0; i < objectPositions.size(); ++i) {
		Point objectPoint = Point(objectPositions[i].x, floorBorder[objectPositions[i].x]);
		double distance = sqrt(pow(imageCenter.x - objectPoint.x, 2) + pow(imageCenter.y - objectPoint.y, 2));
		distances.push_back(make_pair(objectPoint, distance));
	}
}

void mergeCloseLines(vector<Vec4i>& lines, vector<Vec4i>& mergedLines, int mergeThreshold = 10) {
	vector<bool> merged(lines.size(), false);
	for (size_t i = 0; i < lines.size(); ++i) {
		if (merged[i]) continue;

		Vec4i& line1 = lines[i];
		// Averages of start and end points
		int avgX1 = line1[0], avgY1 = line1[1], avgX2 = line1[2], avgY2 = line1[3];
		int count = 1;

		for (size_t j = i + 1; j < lines.size(); ++j) {
			if (merged[j]) continue;

			Vec4i& line2 = lines[j];
			// Check if lines are close enough to merge
			if ((abs(line1[0] - line2[0]) < mergeThreshold && abs(line1[1] - line2[1]) < mergeThreshold) ||
				(abs(line1[2] - line2[2]) < mergeThreshold && abs(line1[3] - line2[3]) < mergeThreshold)) {
				avgX1 += line2[0];
				avgY1 += line2[1];
				avgX2 += line2[2];
				avgY2 += line2[3];
				count++;
				merged[j] = true;
			}
		}

		mergedLines.push_back(Vec4i(avgX1 / count, avgY1 / count, avgX2 / count, avgY2 / count));
		merged[i] = true;
	}
}

void smoothFloorBorder(vector<int>& floorBorder, vector<int>& smoothedBorder, int maxJump = 20) {
	smoothedBorder = floorBorder; // Copy original floor border
	for (size_t i = 1; i < floorBorder.size() - 1; i++) {
		// Check for sharp jumps compared to neighbors
		if (abs(floorBorder[i] - floorBorder[i - 1]) > maxJump && abs(floorBorder[i] - floorBorder[i + 1]) > maxJump) {
			smoothedBorder[i] = (smoothedBorder[i - 1] + smoothedBorder[i + 1]) / 2; // Average of neighbors
		}
	}
}
