#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "types.h"
#include "constants.h"
#include "utility.h"

using Contour = std::vector<cv::Point>;

cv::Mat weightedCombine(const cv::Mat& addition, const cv::Mat& storage, float addition_weight) {
	assert(addition_weight >= 0.0f && addition_weight <= 1.0f);

	float storage_weight = 1.0f - addition_weight;

	return addition_weight * addition + storage_weight * storage;
}

std::vector<cv::Point> clampContourY(const std::vector<cv::Point>& contour, int y_threshold) {
	std::vector<cv::Point> clamped_contour;
	for (const cv::Point& point : contour) {
		int clamped_y = std::min(point.y, y_threshold);
		clamped_contour.push_back(cv::Point(point.x, clamped_y));
	}
	return clamped_contour;
}

cv::Mat extractLargestContour(const cv::Mat& image, std::vector<std::vector<cv::Point>>& floor_cns, std::vector<std::vector<cv::Point>>& above_cns) {
	// Preprocessing (optional)
	// You might need additional preprocessing steps depending on your image characteristics

	cv::Mat single_channel;
	cv::cvtColor(image, single_channel, cv::COLOR_BGR2GRAY);

	// Create output image (same size and type as input)
	cv::Mat filtered_image = cv::Mat::zeros(single_channel.rows, single_channel.cols, CV_8UC1);

	// Find contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(single_channel, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::sort(contours.begin(), contours.end(), [](auto& c1, auto& c2) {
		return cv::contourArea(c1) > cv::contourArea(c2);
	});

	// Extract largest contour (if any)
	std::vector<std::vector<cv::Point>> largest_contour;
	
	int contour_draw_count = 10;

	int contour_area_threshold = 150;

	for (size_t i = 0; i < contour_draw_count; i++)
	{
		if (i < contours.size() && cv::contourArea(contours[i]) > contour_area_threshold) {
			largest_contour.push_back(contours[i]);
		}
	}

	int horizon_y_threshold = 10;
	int contour_above_horizon_area_threshold = 500;

	int horizon_reset_threshold = 5;
	static int loops_with_unreset_horizon = 0;

	std::vector<std::vector<cv::Point>> floor_contours;
	std::vector<std::vector<cv::Point>> above_contours;
	int highest_y_coordinate = 1000000;
	static int previous_highest_y = 1000000;
	if (largest_contour.size() > 0) {
		for (size_t j = 0; j < largest_contour[0].size(); j++)
		{
			if (largest_contour[0][j].y < highest_y_coordinate) {
				highest_y_coordinate = largest_contour[0][j].y;
				if (previous_highest_y == 1000000) {
					previous_highest_y = highest_y_coordinate;
				}
				else {
					if (previous_highest_y > highest_y_coordinate) {
						if (abs(previous_highest_y - highest_y_coordinate) < horizon_y_threshold) {
							previous_highest_y = highest_y_coordinate;
						}
						else {
							loops_with_unreset_horizon++;
						}
						if (loops_with_unreset_horizon >= horizon_reset_threshold) {
							previous_highest_y = highest_y_coordinate;
							loops_with_unreset_horizon = 0;
						}
					}
					else {
						previous_highest_y = highest_y_coordinate;
					}
				}
			}
		}

		floor_cns.push_back(largest_contour[0]);
		above_cns.push_back(largest_contour[0]);

		for (size_t i = 1; i < largest_contour.size(); i++)
		{
			//largest_contour[i] = clampContourY(largest_contour[i], previous_highest_y);
			bool checkarea = false;
			for (size_t j = 0; j < largest_contour[i].size(); j++)
			{
				if (largest_contour[i][j].y > previous_highest_y) {
					checkarea = true;
				}
			}
			if (checkarea) {
				if (cv::contourArea(largest_contour[i]) > contour_above_horizon_area_threshold) {
					above_cns.push_back(largest_contour[i]);
					floor_cns.push_back(largest_contour[i]);
				}
			}
			else {
				above_cns.push_back(largest_contour[i]);
				floor_cns.push_back(largest_contour[i]);
			}
		}
	}

	// Draw horizon line.
	//cv::line(filtered_image, cv::Point(0, previous_highest_y), cv::Point(filtered_image.cols, previous_highest_y), cv::Scalar(255), 2);

	cv::drawContours(filtered_image, floor_cns, -1, cv::Scalar(255), cv::FILLED);
	cv::drawContours(filtered_image, above_cns, -1, cv::Scalar(255), cv::FILLED);

	return filtered_image;
}

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


cv::Mat detectHarrisCorners(const cv::Mat& image, const std::vector<std::vector<cv::Point>>& contours) {

	// Harris corner detection parameters
	int blockSize = 2;
	int ksize = 3;
	double k = 0.04;

	// Calculate Harris corner response
	cv::Mat corner_response;
	cv::cornerHarris(image, corner_response, blockSize, ksize, k);

	// Threshold corner response
	cv::Mat corner_mask;
	double threshold = 0.1 * corner_response.at<double>(0, 0); // Adjust threshold as needed
	cv::threshold(corner_response, corner_mask, threshold, 255, cv::THRESH_BINARY);

	// Find contours of corner mask
	std::vector<std::vector<cv::Point>> corner_contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(corner_mask, corner_contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Filter corners based on proximity to original contours
	cv::Mat result_image = image.clone();
	for (const std::vector<cv::Point>& corner_contour : corner_contours) {
		for (const cv::Point& corner_point : corner_contour) {
			bool is_corner = false;
			for (const std::vector<cv::Point>& original_contour : contours) {
				if (cv::pointPolygonTest(original_contour, corner_point, false) >= 0) {
					is_corner = true;
					break;
				}
			}
			if (is_corner) {
				cv::circle(result_image, corner_point, 3, cv::Scalar(255), -1);  // Draw red circle at corner
			}
		}
	}

	return result_image;
}

cv::Vec4i getLongestLine(const std::vector<cv::Point>& contour) {
	// Input: Contour represented as a vector of points

	// Preprocessing (optional)
	// You might need to simplify the contour or remove noise depending on your needs

	// Length of longest line found so far
	double max_length = 0;
	cv::Vec4i longest_line(0, 0, 0, 0);  // (x1, y1, x2, y2) format for line endpoints

	// Iterate through all pairs of points in the contour
	for (int i = 0; i < contour.size() - 1; i++) {
		cv::Point pt1 = contour[i];
		for (int j = i + 1; j < contour.size(); j++) {
			cv::Point pt2 = contour[j];

			// Calculate line length using distance formula
			double line_length = sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2));

			// Update longest line if current line is longer
			if (line_length > max_length) {
				max_length = line_length;
				longest_line[0] = pt1.x;
				longest_line[1] = pt1.y;
				longest_line[2] = pt2.x;
				longest_line[3] = pt2.y;
			}
		}
	}

	return longest_line;
}

std::vector<cv::Point> smoothContour(const std::vector<cv::Point>& contour, double epsilon = 0.005) {
	// Input: Rough contour represented as a vector of points
	// epsilon: Approximation accuracy (adjust based on smoothness desired)

	// Apply polygonal approximation (Douglas-Peucker algorithm)
	std::vector<cv::Point> approx_contour;
	cv::approxPolyDP(contour, approx_contour, epsilon * cv::arcLength(contour, true), true);

	return approx_contour;
}


cv::RotatedRect minAreaRect(const std::vector<cv::Point>& contour) {
	// Find minimum area rectangle enclosing the contour
	return cv::minAreaRect(contour);
}

cv::Point getCircleCenter(const cv::RotatedRect& rect) {
	// Get the center of the rectangle
	return rect.center;
}

double getCircleRadius(const cv::RotatedRect& rect) {
	// Calculate the radius as the maximum distance from the center to any corner
	double half_width = rect.size.width / 2.0;
	double half_height = rect.size.height / 2.0;
	return std::sqrt(std::max(half_width * half_width, half_height * half_height));
}

double distanceToLine(cv::Point line_start, cv::Point line_end, cv::Point point)
{
	double dist_x = line_end.x - line_start.x;
	double dist_y = line_end.y - line_start.y;
	double normalLength = sqrt(dist_x * dist_x + dist_y * dist_y);
	if (normalLength == 0.0) return 0;
	double distance = (double)((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) / normalLength;
	return abs(distance);
}

std::vector<cv::Point> processImageForObjects(const cv::Mat& inputImage) {
	// Ensure the input image is not empty
	if (inputImage.empty()) {
		std::cerr << "Error: Input image is empty." << std::endl;
		return {};
	}

	// Prepare the image and mask for object distance detection
	cv::Mat isolatedFloor, mask;
	isolateGreenFloor(inputImage, isolatedFloor, mask); // Assume this function is implemented elsewhere
	std::vector<Contour> floor_cns;
	std::vector<Contour> above_cns;
	cv::imshow("Floor", isolatedFloor);
	cv::Mat filteredFloor = extractLargestContour(isolatedFloor, floor_cns, above_cns);
	cv::imshow("Filtered", filteredFloor);
	//cv::Mat res = detectHarrisCorners(filteredFloor, cns);


	// Clone the input image for annotation to preserve the original
	cv::Mat annotatedImage = inputImage.clone();

	cv::Mat contour_edges = filteredFloor.clone();

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
	cv::Canny(filteredFloor, processedMask, 100, 200);

	cv::imshow("Intermediate1", processedMask);


	if (floor_cns.size() > 0) {
		cv::Mat drawing = cv::Mat::zeros(contour_edges.size(), CV_8UC1);
		for (size_t i = 0; i < floor_cns.size(); i++)
		{
			Contour border_counter = floor_cns[i];

			std::vector<cv::Point> hull;
			cv::convexHull(border_counter, hull, true); // Clockwise orientation, return points

			// Draw the original contour and convex hull (optional)
			//cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, border_counter), 0, cv::Scalar(255), 2);
			cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, hull), 0, cv::Scalar(255), 2);
		}
		for (size_t i = 0; i < above_cns.size(); i++)
		{

			// Find the minimum area rectangle
			cv::RotatedRect min_area_rect = minAreaRect(above_cns[i]);
			// Get circle center and radius based on the rectangle
			cv::Point circle_center = getCircleCenter(min_area_rect);
			double circle_radius = getCircleRadius(min_area_rect);
			cv::circle(drawing, circle_center, round(circle_radius), cv::Scalar(255), 2);  // Red for circle
		}
		cv::imshow("Intermediate2", drawing);

		std::vector<cv::Point> hull;
		cv::convexHull(floor_cns[0], hull, true); // Clockwise orientation, return points

		int distance_threshold = 20;
		int dist_thresh2 = distance_threshold * distance_threshold;

		std::vector<cv::Point> non_hull_points;

		cv::Mat hull_outliers = inputImage;//cv::Mat::zeros(isolatedFloor.size(), CV_8UC3);

		/*for (auto& hull_point : hull)
		{
			cv::circle(hull_outliers, hull_point, 1, cv::Scalar(255, 0, 0), 2);
		}*/

		for (auto& pointC : floor_cns[0]) {
	
			bool skip = false;
			for (size_t i = 0; i < hull.size() - 1; ++i) {
				cv::Point pointA = hull[i];
				cv::Point pointB = hull[i + 1];

				if (distanceToLine(pointA, pointB, pointC) < distance_threshold) {
					skip = true;
					break;
				}
			}
			if (!skip) {
				non_hull_points.push_back(pointC);
			}
		}

		for (int i = 0; i < non_hull_points.size(); i++) {
			cv::circle(hull_outliers, non_hull_points[i], 1, cv::Scalar(0, 0, 255), 2);
		}

		//cv::dilate(dilated, dilated, kernel, cv::Point(-1, -1), 2);

		cv::imshow("Intermediate3", hull_outliers);
	}

	// Detect and smooth the floor border
	std::vector<int> floorBorder(width, 0);
	std::vector<int> smoothedBorder(width, 0);
	detectFloorBorder(processedMask, floorBorder);
	smoothFloorBorder(floorBorder, smoothedBorder);

	// Detect object positions along the floor border
	std::vector<cv::Point> objectPositions;
	detectObjectPositions(smoothedBorder, objectPositions);

	return objectPositions;
}
