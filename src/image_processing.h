#ifndef GROUP_10_IMAGE_PROCESSING_H
#define GROUP_10_IMAGE_PROCESSING_H

#ifdef GROUP_10_OPENCV

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "drone.h"
#include "types.h"
#include "constants.h"
#include "utility.h"

using Contour = std::vector<cv::Point>;

// Roll pitch yaw
static Vector3f rotateVector(const Vector3f& u, const Vector3f& rpy) {
    // Conversion of Euler to rotation matrix.
    // a = yaw, b = pitch, y = roll
    float y = rpy.x;
    float b = rpy.y;
    float a = rpy.z;

	float cos_yaw = cos(a);
	float cos_pitch = cos(b);
	float cos_roll = cos(y);

	float sin_yaw = sin(a);
	float sin_pitch = sin(b);
	float sin_roll = sin(y);

    // https://en.wikipedia.org/wiki/Rotation_matrix#General_3D_rotations

	float R1 = cos_yaw * cos_pitch;
	float R2 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
	float R3 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;

	float R5 = sin_yaw * cos_pitch;
	float R6 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
	float a7 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;

	float R9 = -sin_pitch;
	float R10 = cos_pitch * sin_roll;
	float R11 = cos_pitch * cos_roll;

	Vector3f result;

	result.x = R1 * u.x + R2  * u.y + R3  * u.z;
	result.y = R5 * u.x + R6  * u.y + a7  * u.z;
	result.z = R9 * u.x + R10 * u.y + R11 * u.z;
	
	return result;
}

static Vector2f getGridPosition(
    const cv::Point2f cam_point,
    DroneState state,
    bool correct_longitude) {

    Vector2f point_norm;
	point_norm.x = normalizeValue(cam_point.x, 0.0f, (float)IMAGE_SIZE.x);
    point_norm.y = normalizeValue(cam_point.y, 0.0f, (float)IMAGE_SIZE.y);

    float longitude = (point_norm.x * 2 - 1) * HALF_CAMERA_FOV.x;
    float latitude =  (point_norm.y * 2 - 1) * HALF_CAMERA_FOV.y;

    if (correct_longitude) {
        // I noticed that FOV might be shifted or distorted with respect to the center of the drone slightly.
        float FOV_WIDTH_SHIFT_CORRECTION_FACTOR = degToRad(7.0);

        //longitude += FOV_WIDTH_SHIFT_CORRECTION_FACTOR;
    }

	float cos_lat = cos(latitude);
	float cos_lon = cos(longitude);

	Vector3f u;

	u.x = cos_lat * cos_lon;
	u.y = cos_lat * sin(longitude);
	u.z = cos_lon * sin(latitude);

	Vector3f opti_dir = rotateVector(u, state.optitrack_angle);

    float ground_height = 0;
    // Intersection parameter
    float t = -(ground_height + state.optitrack_pos.z) / opti_dir.z;

	Vector2f intersection = { INVALID_POINT_FLT, INVALID_POINT_FLT };

	if (t > 0) {
		return intersection;
	}

	//printf("t: %.3f \n", t);
	//printf("x: %.3f, y: %.3f \n", state.optitrack_pos.x,state.optitrack_pos.y);
	//printf("dirx: %.3f, diry: %.3f \n", opti_dir.x, opti_dir.y);
    intersection.x = state.optitrack_pos.x + t * opti_dir.x;
    intersection.y = state.optitrack_pos.y + t * opti_dir.y;
	//printf("intersection: (%.1f, %.1f)\n", intersection.x, intersection.y);

    return intersection;
}

void setDistortionMatrixes(cv::Mat& out_camera_matrix, cv::Mat& out_distortion_coeffs) {
    out_camera_matrix = (cv::Mat_<double>(3, 3) << 
            FOCAL_LENGTH.x,              0, DISTORTION_CENTER.x,
                        0, FOCAL_LENGTH.y,  DISTORTION_CENTER.y,
                        0,              0,                    1);
    out_distortion_coeffs = (cv::Mat_<double>(4, 1) << 
		DISTORTION_COEFFS[0], DISTORTION_COEFFS[1], DISTORTION_COEFFS[2], DISTORTION_COEFFS[3]);//, DISTORTION_COEFFS[4]);
}

cv::Mat undistortImage(const cv::Mat& input_image) {
	cv::Mat undistorted;
	cv::Mat camera_matrix;
	cv::Mat distortion_coeffs;
	setDistortionMatrixes(camera_matrix, distortion_coeffs);
    cv::undistort(input_image, undistorted, camera_matrix, distortion_coeffs, camera_matrix);
	return undistorted;
}

std::vector<cv::Point2f> undistortPoints(const std::vector<cv::Point2f>& distorted_points) {
	cv::Mat camera_matrix;
	cv::Mat distortion_coeffs;
	setDistortionMatrixes(camera_matrix, distortion_coeffs);
    std::vector<cv::Point2f> points;
    if (!distorted_points.empty()) {
        cv::undistortPoints(distorted_points, points, camera_matrix, distortion_coeffs, cv::noArray(), camera_matrix);
    }

    return points;
}

double distanceToLine(cv::Point line_start, cv::Point line_end, cv::Point point)
{
	double dist_x = line_end.x - line_start.x;
	double dist_y = line_end.y - line_start.y;
	double normalLength = sqrt(dist_x * dist_x + dist_y * dist_y);
	if (normalLength == 0.0) return 0;
	double distance = (double)(dist_x * (line_start.y - point.y) - (line_start.x - point.x) * dist_y) / normalLength;
	return abs(distance);
}

std::vector<cv::Point> clampContourY(const std::vector<cv::Point>& contour, int y_threshold) {
	std::vector<cv::Point> clamped_contour;
	for (const cv::Point& point : contour) {
		int clamped_y = std::min(point.y, y_threshold);
		clamped_contour.push_back(cv::Point(point.x, clamped_y));
	}
	return clamped_contour;
}

cv::Mat extractLargestContour(const cv::Mat& image, float* current_horizon_y, std::vector<std::vector<cv::Point>>& floor_cns, std::vector<std::vector<cv::Point>>& above_cns) {
	// Preprocessing (optional)
	// You might need additional preprocessing steps depending on your image characteristics

	cv::Mat single_channel;
	cv::cvtColor(image, single_channel, cv::COLOR_BGR2GRAY);

	// Find contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(single_channel, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point>& c1, std::vector<cv::Point>& c2) {
		return cv::contourArea(c1) > cv::contourArea(c2);
	});

	// Extract largest contour (if any)
	std::vector<std::vector<cv::Point>> largest_contours;
	
	int contour_draw_count = 10;

	int contour_area_threshold = 150;

	for (size_t i = 0; i < contour_draw_count; i++)
	{
		if (i < contours.size() && cv::contourArea(contours[i]) > contour_area_threshold) {
			largest_contours.push_back(contours[i]);
		}
	}

	int horizon_y_threshold = 10;
	int contour_above_horizon_area_threshold = 500;

	int horizon_reset_threshold = 5;
	static int loops_with_unreset_horizon = 0;

	int highest_y_coordinate = INT_MAX;
	static int horizon_y = INT_MAX;
	if (largest_contours.size() > 0) {
		for (size_t j = 0; j < largest_contours[0].size(); j++)
		{
			if (largest_contours[0][j].y < highest_y_coordinate) {
				highest_y_coordinate = largest_contours[0][j].y;
				if (horizon_y == INT_MAX) {
					horizon_y = highest_y_coordinate;
				}
				else {
					if (horizon_y > highest_y_coordinate) {
						if (abs(horizon_y - highest_y_coordinate) < horizon_y_threshold) {
							horizon_y = highest_y_coordinate;
						}
						else {
							loops_with_unreset_horizon++;
						}
						if (loops_with_unreset_horizon >= horizon_reset_threshold) {
							horizon_y = highest_y_coordinate;
							loops_with_unreset_horizon = 0;
						}
					}
					else {
						horizon_y = highest_y_coordinate;
					}
				}
			}
		}

		floor_cns.push_back(largest_contours[0]);
		//above_cns.push_back(largest_contours[0]);

		for (size_t i = 1; i < largest_contours.size(); i++)
		{
			//largest_contours[i] = clampContourY(largest_contours[i], horizon_y);
			bool checkarea = false;
			for (size_t j = 0; j < largest_contours[i].size(); j++)
			{
				if (largest_contours[i][j].y > horizon_y) {
					checkarea = true;
				}
			}
			if (checkarea) {
				if (cv::contourArea(largest_contours[i]) > contour_above_horizon_area_threshold) {
					above_cns.push_back(largest_contours[i]);
				}
			} else {
				floor_cns.push_back(largest_contours[i]);
			}
		}
	}

	// Create output image (same size and type as input)
	cv::Mat filtered_image = cv::Mat::zeros(image.size(), CV_8UC1);
#ifndef IN_PAPARAZZI
	cv::Mat contour_image = cv::Mat::zeros(image.size(), CV_8UC1);
#endif

	if (floor_cns.size() > 0) {
		std::vector<std::vector<cv::Point>> cns = { floor_cns[0] };

		cv::drawContours(filtered_image, cns, -1, cv::Scalar(255), cv::FILLED);
#ifndef IN_PAPARAZZI
		cv::drawContours(contour_image, contours, -1, cv::Scalar(255), 1, cv::LINE_8);
#endif
	}

#ifndef IN_PAPARAZZI
	cv::imshow("Intermediate1", contour_image);
	//cv::drawContours(filtered_image, above_cns, -1, cv::Scalar(255), cv::FILLED);
#endif

	// Draw horizon line.
	bool draw_horizon = true;

	cv::Mat filtered_image_with_horizon;

	if (draw_horizon) {
		filtered_image.copyTo(filtered_image_with_horizon);
		cv::line(filtered_image_with_horizon, cv::Point(0, horizon_y), cv::Point(filtered_image_with_horizon.cols, horizon_y), cv::Scalar(230), 2);
		
		#ifndef IN_PAPARAZZI
		cv::imshow("Filtered", filtered_image_with_horizon);
		#endif
	} else {
		#ifndef IN_PAPARAZZI
		cv::imshow("Filtered", filtered_image);
		#endif
	}

	*current_horizon_y = horizon_y;

	return filtered_image;
}

cv::Mat isolateGreenFloor(const cv::Mat& image, cv::Mat& isolatedFloor, cv::Mat& mask) {
	// Convert image to HSV
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

	// Define range of green color in HSV
	// Threshold the HSV image to get only green color.
	if (IN_REAL_LIFE == 1) {
		cv::inRange(hsvImage, cv::Scalar(20, 0, 0), cv::Scalar(80, 255, 220), mask);
	} else {
		// Gazebo needs different color values due to simulated camera.
		cv::inRange(hsvImage, cv::Scalar(20, 100, 0), cv::Scalar(80, 255, 140), mask);
	}

	// Erode and dilate to remove noise
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

	// Bitwise-AND mask and original image
	cv::bitwise_and(image, image, isolatedFloor, mask);

	return mask;
}

#ifndef IN_PAPARAZZI

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

void detectObjectPositions(std::vector<int>& floorBorder, std::vector<cv::Point2f>& objectPositions, int maxSlopeChange = 10, int minPosChange = 10) {
	std::vector<int> slopes;
	for (size_t i = 1; i < floorBorder.size(); i++) {
		slopes.push_back(floorBorder[i] - floorBorder[i - 1]); // Calculate slope
	}

	for (size_t i = 1; i < slopes.size(); i++) {
		if (abs(slopes[i] - slopes[i - 1]) > maxSlopeChange) {
			if (objectPositions.empty() || int(i - objectPositions.back().x) > minPosChange) {
				objectPositions.push_back(cv::Point2f(i, floorBorder[i]));
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

#endif

std::vector<std::vector<cv::Point2f>> groupPoints(const std::vector<cv::Point2f>& points, double distance) {
  std::unordered_map<int, std::vector<cv::Point2f>> clusters;
  for (size_t i = 0; i < points.size(); ++i) {
    bool found_cluster = false;
    for (const std::pair<int, std::vector<cv::Point2f>>& pair : clusters) {
	  int cluster_id = pair.first;
	  const std::vector<cv::Point2f>& cluster = pair.second;
      for (const cv::Point2f& other_point : cluster) {
        double dist = std::sqrt(std::pow(points[i].x - other_point.x, 2) + std::pow(points[i].y - other_point.y, 2));
        if (dist <= distance) {
          clusters[cluster_id].push_back(points[i]);
          found_cluster = true;
          break;
        }
      }
      if (found_cluster) {
        break;
      }
    }
    if (!found_cluster) {
      clusters[i] = {points[i]};
    }
  }
  std::vector<std::vector<cv::Point2f>> result;
  for (const std::pair<int, std::vector<cv::Point2f>>& pair : clusters) {
	// Save cluster
    result.push_back(pair.second);
  }
  return result;
}

std::vector<cv::Point2f> processImageForObjects(const cv::Mat& inputImage) {
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
	
	float horizon_y = 0;

	cv::Mat filteredFloor = extractLargestContour(isolatedFloor, &horizon_y, floor_cns, above_cns);

	//cv::Mat res = detectHarrisCorners(filteredFloor, cns);

	// Clone the input image for annotation to preserve the original
	cv::Mat annotatedImage = inputImage.clone();

	cv::Mat contour_edges = filteredFloor.clone();

	// Detect object distances and update the original image (if necessary)
	// Assuming the floor border is detected from the bottom of the image
	int img_height = inputImage.rows;
	int img_width = inputImage.cols;

	std::vector<cv::Point2f> obstacle_base_points;
	std::vector<cv::Point2f> potential_obstacle_points;

	if (floor_cns.size() > 0) {
#ifndef IN_PAPARAZZI
		cv::Mat drawing = cv::Mat::zeros(contour_edges.size(), CV_8UC1);
		for (size_t i = 0; i < floor_cns.size(); i++) {
			Contour border_counter = floor_cns[i];

			std::vector<cv::Point> hull;
			cv::convexHull(border_counter, hull, true); // Clockwise orientation, return points

			// Draw the original contour and convex hull (optional)
			//cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, border_counter), 0, cv::Scalar(255), 2);
			cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, hull), 0, cv::Scalar(255), 2);
		}
		// Draw circles around the non floor hulls.
		for (size_t i = 0; i < above_cns.size(); i++) {
			// Find the minimum area rectangle
			cv::RotatedRect min_area_rect = minAreaRect(above_cns[i]);
			// Get circle center and radius based on the rectangle
			cv::Point circle_center = getCircleCenter(min_area_rect);
			double circle_radius = getCircleRadius(min_area_rect);
			cv::circle(drawing, circle_center, round(circle_radius), cv::Scalar(255), 2);  // Red for circle
		}
		
		cv::imshow("Intermediate2", drawing);
#endif

		// Get outlining hull of biggest chunk of the floor.
		std::vector<cv::Point> hull;
		cv::convexHull(floor_cns[0], hull, true);

		// Rejects points further than this from any hull line.
		int too_close_to_any_hull_line = 8;

		// Hull points within these ranges of edge of screen are considered
		// part of the edges of the screen (if both points are within, it is a "UNACCEPTABLE" hull line). 
		int hull_width_threshold = 5;
		int hull_height_threshold = 5;

		// Draw
		cv::Mat hull_outliers = inputImage;//cv::Mat::zeros(isolatedFloor.size(), CV_8UC3);

		// Reject points further than this distance from the closest "ACCEPTABLE" hull line.
		double too_close_to_closest_hull_line = 20;

		// Rejects point further than this distance from the bottom of the screen.
		// If the floor contour randomly jumps up, this will prevent that from being considered.
		double too_close_to_floor_line = 5;

		hull.push_back(hull[0]);

		for (cv::Point& pointC : floor_cns[0]) {
			// Shortest distance of point to a non edge hull line.
			double shortest_hull_line_distance = DBL_MAX;
	
			bool skip = false;

			for (size_t i = 0; i < hull.size() - 1; ++i) {
				cv::Point pointA = hull[i];
				cv::Point pointB = hull[i + 1];
				//cv::circle(isolatedFloor, pointA, 2, cv::Scalar(255, 255, 0), -2);
				//cv::circle(isolatedFloor, pointB, 2, cv::Scalar(255, 255, 0), -2);

				// SKIP "UNACCEPTABLE" HULL LINES (i.e. near bottom edges of screen).

				// Skip hull lines that go from near one left edge of the screen to near the left other edge.
				if (pointA.x > img_width - hull_width_threshold && pointB.x > img_width - hull_width_threshold) {
					continue;
				}

				// Skip hull lines that go from near one right edge of the screen to near the right other edge.
				if (pointA.x < hull_width_threshold && pointB.x < hull_width_threshold) {
					continue;
				}

				// Skip hull lines that go from near one bottom edge of the screen to near the bottom other edge.
				if (pointA.y > img_height - hull_height_threshold && pointB.y > img_height - hull_height_threshold) {
					continue;
				}

				// "ACCEPTABLE" HULL LINE.
				
				double dist_to_floor = distanceToLine({ 0, img_height }, { img_width, img_height }, pointC);

				double dist_to_hull_line = distanceToLine(pointA, pointB, pointC);

				if (dist_to_hull_line < shortest_hull_line_distance) {
					shortest_hull_line_distance = dist_to_hull_line;
				}

				cv::line(isolatedFloor, pointA, pointB, cv::Scalar(0, 0, 255), 2);

				if (dist_to_floor < too_close_to_floor_line) {
					skip = true;
					break;
				}
				// Point too close to one of the hull lines
				if (dist_to_hull_line < too_close_to_any_hull_line) {
					skip = true;
					break;
				}
				//cv::line(isolatedFloor, pointA, pointB, cv::Scalar(0, 0, 255), 2);
			}

			// Only points that were not too close to any of the contours and are below horizon (y is positive down).
			if (!skip && pointC.y > horizon_y) {
				// Only points that have a distance to their closest contour shorter than a threshold.
				// This makes sure contours which are distorted but part of the floor convex hull are skipped.
				// This also means that when approaching an object, as its bottom gets closer to the bottom
				// of the screen, it will no longer be detected (hence relying on some memory of where obstacles were).
				if (shortest_hull_line_distance > too_close_to_closest_hull_line) {
					potential_obstacle_points.push_back(pointC);
				}
				//cv::circle(isolatedFloor, pointC, 2, cv::Scalar(255, 0, 0), -2);
			}
		}

		// Any points within this many pixels of another point are considered as part of one obstacle.
		// This essentially depends on what the longest fully straight contour is.
		// If a contour is fully straight, it will only place points at the top and bottom,
		// which will make those points far apart and hence considered different obstacles bases.
		double obstacle_contour_grouping_distance = 30;
		float lowest_point_dist_threshold = 10;

		// Group all the potential points by their distance to the closest other point.
		std::vector<std::vector<cv::Point2f>> separateGroups = groupPoints(potential_obstacle_points, obstacle_contour_grouping_distance);

		// Get lowest points on the screen of each group of points.
		for (size_t i = 0; i < separateGroups.size(); i++)
		{
			cv::Point lowest_point = { 0, 0 }; // Top of screen.
			const std::vector<cv::Point2f>& group = separateGroups[i];

			if (group.size() == 0) continue;

			// Find lowest point of the individual grouped points.
			for (size_t j = 0; j < group.size(); j++)
			{
				cv::Point point = group[j];
				cv::circle(isolatedFloor, point, 2, cv::Scalar(128, 255, 255), -2);
				// Lower on the screen.
				if (point.y >= lowest_point.y) {
					lowest_point = point;
				}
			}

			// Add all points that are vertically near the lowest point (for objects with longer base).
			for (size_t j = 0; j < group.size(); j++)
			{
				cv::Point point = group[j];
				if (abs(point.y - lowest_point.y) < lowest_point_dist_threshold) {
					obstacle_base_points.push_back(point);
				}
			}
			// Add only the lowest point vertically of each group (this would consider objects with long horizontal bases as only their lowest point).
			//obstacle_base_points.push_back(lowest_point);
		}
	}

	#ifndef IN_PAPARAZZI
	cv::imshow("Floor", isolatedFloor);
	#endif

	// Detect and smooth the floor border
	// std::vector<int> floorBorder(img_width, 0);
	// std::vector<int> smoothedBorder(img_width, 0);
	// detectFloorBorder(processedMask, floorBorder);
	// smoothFloorBorder(floorBorder, smoothedBorder);
	// // Detect object positions along the floor border
	// std::vector<cv::Point2f> objectPositions;
	// detectObjectPositions(smoothedBorder, objectPositions);

	//return objectPositions;
	//return potential_obstacle_points;
	return obstacle_base_points;
}

std::vector<cv::Point> getGridPoints(
	const DroneState& state,
	std::vector<cv::Point2f> points,
	const cv::Size& img_size, 
	bool undistort,
	bool correct_pitch,
	bool correct_longitude) {
	if (undistort) {
		points = undistortPoints(points);
	}

	std::vector<cv::Point2f> check_points;

	for (size_t i = 0; i < points.size(); i++) {
		cv::Point2f point = points[i];
		if (point.x < 0 || point.x > img_size.width) continue;
		if (point.y < 0 || point.y > img_size.height) continue;
		check_points.push_back(point);
	}

	//printf("Pos: %.3f, %.3f, %.3f \n", state.optitrack_pos.x, state.optitrack_pos.y, state.optitrack_pos.z);
	//printf("Angles: %.3f, %.3f, %.3f \n", radToDeg(state.optitrack_angle.x), radToDeg(state.optitrack_angle.y), radToDeg(state.optitrack_angle.z));

    std::vector<cv::Point> grid_points;
    // Transform each point from camera view to bird's eye view (and relative to grid).
    for (size_t i = 0; i < check_points.size(); i++) {
		DroneState corrected = state;
		if (correct_pitch) {
			float pitch_correction = correctPitch(state.optitrack_ang_rates.y);
			corrected.optitrack_angle.y += pitch_correction;
		}
        Vector2f optitrack_point = getGridPosition(check_points[i], corrected, correct_longitude);

		if (!validVectorFloat(optitrack_point)) continue;

        Vector2i grid_point = optitrackCoordinateToGrid(optitrack_point);

        if (!validVectorInt(grid_point)) continue;

        grid_points.emplace_back(grid_point.x, grid_point.y);
    }

    return grid_points;
}

/*
std::vector<cv::Point2f> opticalFlow(const cv::Mat& frame, const std::vector<cv::Point2f>& new_points) {
	static cv::Mat prev_frame_gray;
	static cv::Mat frame_gray;
	static std::vector<cv::Point2f> p0;

	cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

	if (prev_frame_gray.empty()) {
		prev_frame_gray = frame_gray;
		return new_points;
	}

	if (new_points.size() == 0) return new_points;

	if (p0.size() == 0) {
		p0 = new_points;
	}

	std::vector<cv::Point2f> p1;
	
	// calculate optical flow
	std::vector<u_char> status;
	std::vector<float> err;
	cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
	cv::calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, p0, p1, status, err, cv::Size(15,15), 2, criteria);
	std::vector<cv::Point2f> good_new;

	for(uint i = 0; i < p0.size(); i++)
	{
		// Select good points
		if(status[i] == 1) {
			good_new.push_back(p1[i]);
			// draw the tracks
			//cv::line(mask,p1[i], p0[i], colors[i], 2);
			///cv::circle(frame, p1[i], 5, colors[i], -1);
		}
	}
	prev_frame_gray = frame_gray;
	p0 = good_new;
	return good_new;
}
*/

#endif

#endif