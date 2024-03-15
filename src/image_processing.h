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

using namespace std;

cv::Mat WeightedCombine(const cv::Mat& addition, const cv::Mat& storage, float addition_weight) {
	assert(addition_weight >= 0.0f && addition_weight <= 1.0f);

	float storage_weight = 1.0f - addition_weight;

	return addition_weight * addition + storage_weight * storage;
}

cv::Mat isolateGreenFloor(const cv::Mat& image, cv::Mat& isolatedFloor, cv::Mat& mask) {
	// Convert image to HSV
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

	// Define range of green color in HSV
	cv::Scalar lowerGreen = cv::Scalar(20, 0, 0);
	cv::Scalar upperGreen = cv::Scalar(80, 255, 175);

	// Threshold the HSV image to get only green colors
	cv::inRange(hsvImage, lowerGreen, upperGreen, mask);

	// Erode and dilate to remove noise
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

	// Bitwise-AND mask and original image
	cv::bitwise_and(image, image, isolatedFloor, mask);

	return mask;
}
void detectFloorBorder(const cv::Mat& processedImage, std::vector<int>& floorBorder) {
	for (int x = 0; x < processedImage.cols; ++x) {
		for (int y = processedImage.rows - 1; y >= 0; --y) {
			if (processedImage.at<uchar>(y, x) > 0) { // Assuming binary image
				floorBorder[x] = y;
				break;
			}
		}
	}
}

void smoothFloorBorder(std::vector<int>& floorBorder, std::vector<int>& smoothedBorder, int maxJump = 20) {
	smoothedBorder = floorBorder; // Copy original floor border
	for (size_t i = 1; i < floorBorder.size() - 1; i++) {
		// Check for sharp jumps compared to neighbors
		if (abs(floorBorder[i] - floorBorder[i - 1]) > maxJump && abs(floorBorder[i] - floorBorder[i + 1]) > maxJump) {
			smoothedBorder[i] = (smoothedBorder[i - 1] + smoothedBorder[i + 1]) / 2; // Average of neighbors
		}
	}
}

void detectObjectPositions(std::vector<int>& floorBorder, std::vector<cv::Point>& objectPositions, int maxSlopeChange = 10, int minPosChange = 10) {
	std::vector<int> slopes;
	for (size_t i = 1; i < floorBorder.size(); i++) {
		slopes.push_back(floorBorder[i] - floorBorder[i - 1]); // Calculate slope
	}

	for (size_t i = 1; i < slopes.size(); i++) {
		if (abs(slopes[i] - slopes[i - 1]) > maxSlopeChange) {
			if (objectPositions.empty() || int(i - objectPositions.back().x) > minPosChange) {
				objectPositions.push_back(cv::Point(i, floorBorder[i]));
			}
		}
	}
}

void calculateDistancesToObjects(cv::Point imageCenter, std::vector<cv::Point>& objectPositions, std::vector<int>& floorBorder, std::vector<std::pair<cv::Point, double>>& distances) {
	for (size_t i = 0; i < objectPositions.size(); ++i) {
		cv::Point objectPoint = cv::Point(objectPositions[i].x, floorBorder[objectPositions[i].x]);
		double distance = sqrt(pow(imageCenter.x - objectPoint.x, 2) + pow(imageCenter.y - objectPoint.y, 2));
		distances.push_back(std::make_pair(objectPoint, distance));
	}
}

void detectObjectDistances(const cv::Mat& image, const cv::Mat& mask, std::vector<std::pair<cv::Point, double>>& outputDistances) {
	// Assuming the floor border is detected from the bottom of the image
	int height = image.rows;
	int width = image.cols;
	cv::Point bottomCenter(width / 2, height - 1);

	// Kernel for morphological operations
	int kernelSize = 5;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

	// Process the mask to detect edges and dilate
	cv::Mat processedMask;
	cv::Canny(mask, processedMask, 100, 200);
	cv::dilate(processedMask, processedMask, kernel, cv::Point(-1, -1), 2);

	// Detect and smooth the floor border
	std::vector<int> floorBorder(width, 0);
	std::vector<int> smoothedBorder(width, 0);
	detectFloorBorder(processedMask, floorBorder);
	smoothFloorBorder(floorBorder, smoothedBorder);

	// Detect object positions along the floor border
	std::vector<cv::Point> objectPositions;
	detectObjectPositions(smoothedBorder, objectPositions);

	// Calculate distances from bottom center to each object position
	std::vector<std::pair<cv::Point, double>> distances;
	calculateDistancesToObjects(bottomCenter, objectPositions, smoothedBorder, distances);

	// Set the output
	outputDistances = distances;
}


void mergeCloseLines(const std::vector<cv::Vec4i>& lines, std::vector<cv::Vec4i>& mergedLines, int mergeThreshold = 10) {
	std::vector<bool> merged(lines.size(), false);
	for (size_t i = 0; i < lines.size(); ++i) {
		if (merged[i]) continue;

		const cv::Vec4i& line1 = lines[i];
		// Averages of start and end points
		int avgX1 = line1[0], avgY1 = line1[1], avgX2 = line1[2], avgY2 = line1[3];
		int count = 1;

		for (size_t j = i + 1; j < lines.size(); ++j) {
			if (merged[j]) continue;

			const cv::Vec4i& line2 = lines[j];
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

		mergedLines.push_back(cv::Vec4i(avgX1 / count, avgY1 / count, avgX2 / count, avgY2 / count));
		merged[i] = true;
	}
}

std::vector<cv::Point> processImageForObjects(const cv::Mat& inputImage) {
	// Ensure the input image is not empty
	if (inputImage.empty()) {
		cerr << "Error: Input image is empty." << endl;
		return {};
	}

	// Prepare the image and mask for object distance detection
	cv::Mat isolatedFloor, mask;
	isolateGreenFloor(inputImage, isolatedFloor, mask); // Assume this function is implemented elsewhere

	// Clone the input image for annotation to preserve the original
	cv::Mat annotatedImage = inputImage.clone();

	// Detect object distances and update the original image (if necessary)
	// Assuming the floor border is detected from the bottom of the image
	int height = inputImage.rows;
	int width = inputImage.cols;
	cv::Point bottomCenter(width / 2, height - 1);

	// Kernel for morphological operations
	int kernelSize = 5;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));

	// Process the mask to detect edges and dilate
	cv::Mat processedMask;
	cv::Canny(mask, processedMask, 100, 200);
	cv::dilate(processedMask, processedMask, kernel, cv::Point(-1, -1), 2);

	// Detect and smooth the floor border
	std::vector<int> floorBorder(width, 0);
	std::vector<int> smoothedBorder(width, 0);
	detectFloorBorder(processedMask, floorBorder);
	smoothFloorBorder(floorBorder, smoothedBorder);

	// Detect object positions along the floor border
	std::vector<cv::Point> objectPositions;
	detectObjectPositions(smoothedBorder, objectPositions);

	// The function does not need to return the image itself;
	// objectDistances will contain the positions and distances of detected objects.

	// Optional: Visualize results on the image
	// ... (visualization code omitted for brevity) ...

	return objectPositions;
}
