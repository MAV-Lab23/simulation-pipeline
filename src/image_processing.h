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

/*
// Function to filter points based on neighboring points within a radius
cv::Mat filterPoints(const cv::Mat& points) {
	// Input: points - Mat of size (N, 1, CV_32F) where N is the number of points and each row represents a point (x, y)
	// Output: filtered_points - Mat of same size and type as points containing only points near big clumps

	// Parameters (adjust these based on your data)
	int min_neighbors = 10;  // Minimum number of neighbors for a clump
	float search_radius = 10.0f;  // Radius to search for neighbors

	// Allocate memory for neighbor indices and distances
	int* neighbors = (int*)malloc(100 * sizeof(int));
	float* distances = (float*)malloc(100 * sizeof(float));

	// Create output point container
	cv::Mat filtered_points = cv::Mat::zeros(points.rows, 1, CV_32F);

	// Iterate through each point
	for (int i = 0; i < points.rows; i++) {
		// Get current point
		cv::Vec2f current_point = points.at<cv::Vec2f>(i, 0);

		// Search for neighbors within radius (brute-force approach)
		int num_neighbors = 0;
		for (int j = 0; j < points.rows; j++) {
			if (i == j) {
				continue; // Skip itself
			}

			cv::Vec2f neighbor_point = points.at<cv::Vec2f>(j, 0);
			float distance = sqrtf(powf(current_point[0] - neighbor_point[0], 2.0f) + powf(current_point[1] - neighbor_point[1], 2.0f));
			if (distance <= search_radius) {
				neighbors[num_neighbors] = j;
				distances[num_neighbors] = distance;
				num_neighbors++;
			}
		}

		// Check if point has enough neighbors and copy it to the filtered set
		if (num_neighbors >= min_neighbors) {
			filtered_points.at<cv::Vec2f>(i, 0) = current_point;
		}
	}

	// Free allocated memory
	free(neighbors);
	free(distances);

	return filtered_points;
}
*/

using Contour = std::vector<cv::Point>;

cv::Mat WeightedCombine(const cv::Mat& addition, const cv::Mat& storage, float addition_weight) {
	assert(addition_weight >= 0.0f && addition_weight <= 1.0f);

	float storage_weight = 1.0f - addition_weight;

	return addition_weight * addition + storage_weight * storage;
}

// Function to filter noise in an image
cv::Mat filterNoise(const cv::Mat& image) {
	// Input: image - Grayscale image

	// Parameters (adjust these based on your noise characteristics)
	int blur_kernel_size = 3;  // Kernel size for blurring
	double threshold_value = 127;  // Threshold for removing low-intensity noise

	// 1. Apply blurring to reduce high-frequency noise (consider Gaussian blur for better results)
	cv::Mat blurred_image;
	cv::GaussianBlur(image, blurred_image, cv::Size(blur_kernel_size, blur_kernel_size), 0); // Adjust sigma based on noise

	// 2. Apply thresholding to remove low-intensity noise
	cv::Mat filtered_image;
	cv::threshold(blurred_image, filtered_image, threshold_value, 255, cv::THRESH_BINARY);

	// 3. (Optional) Morphological operations for further refinement (adjust based on noise)
	// cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// cv::erode(filtered_image, filtered_image, element);  // Remove small isolated noise spots
	// cv::dilate(filtered_image, filtered_image, element);  // Restore potential edge details

	return filtered_image;
}



// Function to check if a point is a core point
bool isCorePoint(const std::vector<cv::Point>& points, double eps, int min_pts, int point_idx, const std::vector<int>& labels) {
	int num_neighbors = 0;

	// Iterate through all points
	for (int i = 0; i < points.size(); i++) {
		if (point_idx != i && labels[i] != 0) {  // Exclude itself and noise points
			double distance = cv::norm(points[point_idx] - points[i]);
			if (distance <= eps) {
				num_neighbors++;
			}
		}
	}

	return num_neighbors >= min_pts;
}

// Function to recursively expand a cluster
void expandCluster(const std::vector<cv::Point>& points, double eps, int min_pts, int point_idx, int cluster_id, std::vector<int>& labels, int& current_cluster_size) {
	labels[point_idx] = cluster_id;  // Assign cluster ID
	current_cluster_size++;

	// Find neighbors within eps
	std::vector<int> neighbors;
	for (int i = 0; i < points.size(); i++) {
		if (point_idx != i && labels[i] == -1) {  // Unvisited points
			double distance = cv::norm(points[point_idx] - points[i]);
			if (distance <= eps) {
				neighbors.push_back(i);
			}
		}
	}

	// Recursively visit unvisited neighbors
	for (int neighbor_idx : neighbors) {
		if (labels[neighbor_idx] == -1) {
			expandCluster(points, eps, min_pts, neighbor_idx, cluster_id, labels, current_cluster_size);
		}
	}
}
// Function to implement DBSCAN clustering
std::vector<int> dbscan(const std::vector<cv::Point>& points, double eps, int min_pts, int& largest_cluster_size) {
	// Input:
	//  - points: Vector of Point structures representing data points
	//  - eps: Search radius for neighborhood definition
	//  - min_pts: Minimum number of neighbors to be considered a core point

	// Initialize variables
	int n_points = points.size();
	std::vector<int> labels(n_points, -1);  // -1: Unvisited, 0: Noise, 1+: Cluster ID
	int cluster_id = 0;

	// Iterate through each data point
	for (int i = 0; i < n_points; i++) {
		if (labels[i] == -1) {  // Unvisited point
			int current_cluster_size = 0;
			if (isCorePoint(points, eps, min_pts, i, labels)) {
				cluster_id++;
				expandCluster(points, eps, min_pts, i, cluster_id, labels, current_cluster_size);
				largest_cluster_size = std::max(largest_cluster_size, current_cluster_size);
			}
			else {
				labels[i] = 0;  // Mark as noise
			}
		}
	}

	return labels;
}

std::vector<cv::Point> clampContourY(const std::vector<cv::Point>& contour, int y_threshold) {
	std::vector<cv::Point> clamped_contour;
	for (const cv::Point& point : contour) {
		int clamped_y = std::min(point.y, y_threshold);
		clamped_contour.push_back(cv::Point(point.x, clamped_y));
	}
	return clamped_contour;
}

cv::Mat extractLargestContour(const cv::Mat & image, std::vector<std::vector<cv::Point>>& floor_cns, std::vector<std::vector<cv::Point>>& above_cns) {
	// Input: Grayscale image (assumed)

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


	int thickness = 2;
	int lineType = cv::LINE_8;

	//cv::line(filtered_image, cv::Point(0, previous_highest_y), cv::Point(filtered_image.cols, previous_highest_y), cv::Scalar(255), thickness, lineType);

	//std::vector<std::vector<cv::Point>> below_horizon;


	cv::drawContours(filtered_image, floor_cns, -1, cv::Scalar(255), cv::FILLED);  // White color
	cv::drawContours(filtered_image, above_cns, -1, cv::Scalar(255), cv::FILLED);  // White color

	return filtered_image;
}


// Function to filter isolated green dots
cv::Mat filterDots(const cv::Mat & image, int search_radius, int min_neighbors) {
	// Input:
	//  - image: Grayscale image (assumed black background with green dots)
	//  - search_radius: Radius to search for neighbors
	//  - min_neighbors: Minimum number of neighbors for a dot to be considered part of a clump

	// Check if image is grayscale (assuming black background)

	cv::Mat single_channel;
	cv::cvtColor(image, single_channel, cv::COLOR_BGR2GRAY);

	// Find non-zero elements (green dots)
	std::vector<cv::Point> dot_locations;
	cv::findNonZero(single_channel, dot_locations);

	// Create output image (same size and type as input)
	cv::Mat filtered_image = cv::Mat::zeros(single_channel.rows, single_channel.cols, CV_8UC1);

	
	// DBSCAN parameters
	double eps = 1.5;
	int min_pts = 2;

	int largest_cluster = 0;

	// Run DBSCAN
	//std::vector<int> labels = dbscan(dot_locations, eps, min_pts, largest_cluster);

	// Print cluster labels (optional)
	for (int i = 0; i < dot_locations.size(); i++) {
		//if (labels[i] > 0 && labels[i] == largest_cluster) {
		filtered_image.at<uchar>(dot_locations[i].y, dot_locations[i].x) = 255;
		//}
	}


	return filtered_image;
}

// Function to undistort points using the fisheye model
void undistortObjectPoints(const std::vector<cv::Point2f>& distortedPoints,
	std::vector<cv::Point2f>& undistortedPoints,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs) {
	// Ensure output vector is empty
	undistortedPoints.clear();

	// Check if input points are not empty
	if (!distortedPoints.empty()) {
		// Convert distorted points to the correct format
		std::vector<cv::Point2f> distortedPointsMat(distortedPoints.begin(), distortedPoints.end());

		// Undistort the points
		cv::fisheye::undistortPoints(distortedPointsMat, undistortedPoints, cameraMatrix, distCoeffs);

		// If you want to project points back to image space after undistortion (optional)
		// cv::fisheye::undistortPoints(distortedPointsMat, undistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
	}
	else {
		// Handle the case where there are no points to undistort
		std::cout << "No points to undistort." << std::endl;
	}
}

cv::Mat isolateGreenFloor(const cv::Mat& image, cv::Mat& isolatedFloor, cv::Mat& mask) {
	// Convert image to HSV
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

	cv::Mat mask1, mask2;

	// Define range of green color in HSV
	// Threshold the HSV image to get only green colors
	cv::inRange(hsvImage, cv::Scalar(20, 0, 0), cv::Scalar(80, 255, 220), mask);

	//cv::inRange(hsvImage, cv::Scalar(15, 150, 100), cv::Scalar(20, 255, 255), mask1);  // Detect yellow range
	//cv::bitwise_not(mask1 | mask2, mask1);  // Invert to exclude these ranges
	//cv::bitwise_and(mask1, mask, mask);  // Invert to exclude these ranges

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

	cv::Mat dilated;
	cv::dilate(processedMask, dilated, kernel, cv::Point(-1, -1), 2);

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
		//contour_edges = addRedLinesToContour(contour_edges, border_counter);

		// Draw or process the fitted trapezoid (example using bounding rectangle)
		//if (fitted_trapezoid.size.width > 0) {
		//	cv::Rect box = fitted_trapezoid.boundingRect();
		//	cv::rectangle(contour_edges, box, cv::Scalar(0, 255, 0), 2);  // Draw green rectangle around the fitted trapezoid
		//}

		//Contour smoothed_border_counter = smoothContour(border_counter);

		//cv::drawContours(contour_edges, smoothed_border_counter, -1, cv::Scalar(255), 5);  // White color

		//cv::Vec4i line = getLongestLine(border_counter);

		//int thickness = 2;
		//int lineType = cv::LINE_8;

		//cv::line(contour_edges, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255), thickness, lineType);


		cv::imshow("Intermediate2", drawing);
	}

	cv::imshow("Intermediate3", dilated);

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
