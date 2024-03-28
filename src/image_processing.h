#ifndef GROUP_10_IMAGE_PROCESSING_H
#define GROUP_10_IMAGE_PROCESSING_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "drone.h"
#include "constants.h"
#include "utility.h"
#include "transform.h"
#include "draw.h"
#include "camera.h"
#include "navigation.h"

#define DRAW_HORIZON_LINE true

cv::Mat undistortImage(const cv::Mat& input_image) {
	cv::Mat undistorted;
    cv::undistort(input_image, undistorted, CAMERA_MATRIX, DISTORTION_COEFFS, CAMERA_MATRIX);
	return undistorted;
}

std::vector<cv::Point2f> undistortPoints(const std::vector<cv::Point2f>& distorted_points) {
    std::vector<cv::Point2f> points;
    if (!distorted_points.empty()) {
        cv::undistortPoints(distorted_points, points, CAMERA_MATRIX, DISTORTION_COEFFS, cv::noArray(), CAMERA_MATRIX);
    }

    return points;
}

float distanceToLine(cv::Point line_start, cv::Point line_end, cv::Point point)
{
	float dist_x = line_end.x - line_start.x;
	float dist_y = line_end.y - line_start.y;
	float normalLength = sqrtf(dist_x * dist_x + dist_y * dist_y);
	if (normalLength == 0.0f) return 0;
	float distance = (float)(dist_x * (line_start.y - point.y) - (line_start.x - point.x) * dist_y) / normalLength;
	return fabsf(distance);
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
	cv::findContours(single_channel, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point>& c1, std::vector<cv::Point>& c2) {
		return cv::contourArea(c1) > cv::contourArea(c2);
	});

	// Extract largest contour (if any)
	std::vector<std::vector<cv::Point>> largest_contours;

	for (size_t i = 0; i < CONTOUR_SELECTION_COUNT; i++)
	{
		if (i < contours.size() && cv::contourArea(contours[i]) > CONTOUR_AREA_THRESHOLD) {
			largest_contours.push_back(contours[i]);
		}
	}

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
						if (abs(horizon_y - highest_y_coordinate) < HORIZON_Y_DIST_THRESHOLD) {
							horizon_y = highest_y_coordinate;
						}
						else {
							loops_with_unreset_horizon++;
						}
						if (loops_with_unreset_horizon >= LOOPS_BEFORE_HORIZON_RESET) {
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
				if (cv::contourArea(largest_contours[i]) > CONTOUR_ABOVE_HORIZON_AREA_THRESHOLD) {
					above_cns.push_back(largest_contours[i]);
				}
			} else {
				floor_cns.push_back(largest_contours[i]);
			}
		}
	}

	// Create output image (same size and type as input)
	cv::Mat filtered_image = cv::Mat::zeros(image.size(), CV_8UC1);
	cv::Mat contour_image;
#ifdef IN_PAPARAZZI
	if (WRITE_REALTIME_PROCESSING_IMAGES) {
		contour_image = cv::Mat::zeros(image.size(), CV_8UC1);
	}
#else
	contour_image = cv::Mat::zeros(image.size(), CV_8UC1);
#endif

	if (floor_cns.size() > 0) {
		std::vector<std::vector<cv::Point>> cns = { floor_cns[0] };

		cv::drawContours(filtered_image, cns, -1, cv::Scalar(255), cv::FILLED);
#ifdef IN_PAPARAZZI
		if (WRITE_REALTIME_PROCESSING_IMAGES) {
			cv::drawContours(contour_image, contours, -1, cv::Scalar(255), 1, cv::LINE_8);
		}
#else
		cv::drawContours(contour_image, contours, -1, cv::Scalar(255), 1, cv::LINE_8);
#endif
	}

	cv::Mat filtered_image_with_horizon;
	if (DRAW_HORIZON_LINE) {
		filtered_image.copyTo(filtered_image_with_horizon);
		cv::line(filtered_image_with_horizon, cv::Point(0, horizon_y), cv::Point(filtered_image_with_horizon.cols, horizon_y), cv::Scalar(230), 2);
	}

#ifdef IN_PAPARAZZI
	writeImage(DRAW_HORIZON_LINE ? filtered_image_with_horizon : filtered_image, "filtered_images");
	writeImage(contour_image, "contour_images");
#else
	cv::imshow("Contour", contour_image);
	cv::imshow("Filtered", DRAW_HORIZON_LINE ? filtered_image_with_horizon : filtered_image);
#endif

	*current_horizon_y = horizon_y;

	return filtered_image;
}

cv::Mat isolateGreenFloor(const cv::Mat& image, cv::Mat& isolatedFloor, cv::Mat& mask) {
	// Convert image to HSV
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

	if (IN_REAL_LIFE == 1) {
		cv::inRange(hsvImage,
		cv::Scalar(
			REAL_WORLD_GREEN_H_MIN,
			REAL_WORLD_GREEN_S_MIN,
			REAL_WORLD_GREEN_V_MIN),
		cv::Scalar(
			REAL_WORLD_GREEN_H_MAX,
			REAL_WORLD_GREEN_S_MAX,
			REAL_WORLD_GREEN_V_MAX), mask);
	} else {
		// Gazebo needs different color values due to simulated camera.
		cv::inRange(hsvImage,
		cv::Scalar(
			PAPARAZZI_GREEN_H_MIN,
			PAPARAZZI_GREEN_S_MIN,
			PAPARAZZI_GREEN_V_MIN),
		cv::Scalar(
			PAPARAZZI_GREEN_H_MAX,
			PAPARAZZI_GREEN_S_MAX,
			PAPARAZZI_GREEN_V_MAX), mask);
	}

	// Erode and dilate to remove noise
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

	// Bitwise-AND mask and original image
	cv::bitwise_and(image, image, isolatedFloor, mask);

	return mask;
}

std::vector<std::vector<cv::Point2f>> groupPoints(const std::vector<cv::Point2f>& points, float distance) {
  std::unordered_map<int, std::vector<cv::Point2f>> clusters;
  for (size_t i = 0; i < points.size(); ++i) {
    bool found_cluster = false;
    for (const std::pair<int, std::vector<cv::Point2f>>& pair : clusters) {
	  int cluster_id = pair.first;
	  const std::vector<cv::Point2f>& cluster = pair.second;
      for (const cv::Point2f& other_point : cluster) {
        float dist = sqrtf(std::pow(points[i].x - other_point.x, 2) + std::pow(points[i].y - other_point.y, 2));
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
	std::vector<std::vector<cv::Point>> floor_cns;
	std::vector<std::vector<cv::Point>> above_cns;
	
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
		// Assume floor is always biggest contour in view.
		// This may fail if a very large plant comes in close proximity of the drone.
		std::vector<cv::Point>& floor = floor_cns[0];

		cv::Mat drawing = cv::Mat::zeros(contour_edges.size(), CV_8UC1);
		for (size_t i = 0; i < floor_cns.size(); i++) {
			std::vector<cv::Point> border_counter = floor_cns[i];

			std::vector<cv::Point> hull;
			cv::convexHull(border_counter, hull, true); // Clockwise orientation, return points

			// Draw the original contour and convex hull (optional)
			//cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, border_counter), 0, cv::Scalar(255), 2);
			cv::drawContours(drawing, std::vector<std::vector<cv::Point>>(1, hull), 0, cv::Scalar(255), 2);
		}
#ifdef IN_PAPARAZZI
    	writeImage(drawing, "hulls");
#else
		cv::imshow("Hull", drawing);
#endif
		
		// Get outlining hull of biggest chunk of the floor.
		std::vector<cv::Point> hull;
		cv::convexHull(floor, hull, true);

		if (floor.size() > 0) {
			// Get rid of objects which are on the bottom edge of the floor contour (such as mats).
			// This is done by collapsing all the y values of the floor contour between the minimum and maximum floor x values.

			// Find min and max x coordinates of the floor (pixels which are at the bottom of the image).
			int left_extreme = img_width / 2 - 1;
			int right_extreme = img_width / 2 + 1;
			size_t left_extreme_index = 0;
			size_t right_extreme_index = floor.size() - 1;

			for (size_t i = 0; i < floor.size(); ++i) {
				const cv::Point& floor_point = floor[i];
				if (floor_point.x <= left_extreme) {
					left_extreme = floor_point.x;
					left_extreme_index = i;
				} else if (floor_point.x >= right_extreme) {
					right_extreme = floor_point.x;
					right_extreme_index = i;
				}
			}

			cv::circle(isolatedFloor, floor[left_extreme_index], 6, cv::Scalar(255, 255, 0), -2);
			cv::circle(isolatedFloor, floor[right_extreme_index], 6, cv::Scalar(255, 255, 0), -2);

			// I'm not sure if a situation exists where the right extreme is ever above the left but this makes sure of that.
			// Collapse the y values of the points between the two extremes. 
			if (left_extreme_index < right_extreme_index) {
				for (size_t i = left_extreme_index; i <= right_extreme_index; ++i) {
					floor[i].y = img_height;
				}
			}
		}

		// Draw
		cv::Mat hull_outliers = inputImage;//cv::Mat::zeros(isolatedFloor.size(), CV_8UC3);


		hull.push_back(hull[0]);

		for (const cv::Point& pointC : floor) {
			// Shortest distance of point to a non edge hull line.
			float shortest_hull_line_distance = DBL_MAX;
	
			bool skip = false;

			for (size_t i = 0; i < hull.size() - 1; ++i) {
				cv::Point pointA = hull[i];
				cv::Point pointB = hull[i + 1];
				//cv::circle(isolatedFloor, pointA, 2, cv::Scalar(255, 255, 0), -2);
				//cv::circle(isolatedFloor, pointB, 2, cv::Scalar(255, 255, 0), -2);

				// SKIP "UNACCEPTABLE" HULL LINES (i.e. near bottom edges of screen).

				// Skip hull lines that go from near one left edge of the screen to near the left other edge.
				if (pointA.x > img_width - HULL_WIDTH_THRESHOLD && pointB.x > img_width - HULL_WIDTH_THRESHOLD) {
					continue;
				}

				// Skip hull lines that go from near one right edge of the screen to near the right other edge.
				if (pointA.x < HULL_WIDTH_THRESHOLD && pointB.x < HULL_WIDTH_THRESHOLD) {
					continue;
				}

				// Skip hull lines that go from near one bottom edge of the screen to near the bottom other edge.
				if (pointA.y > img_height - HULL_HEIGHT_THRESHOLD && pointB.y > img_height - HULL_HEIGHT_THRESHOLD) {
					continue;
				}

				// "ACCEPTABLE" HULL LINE.
				
				float dist_to_floor = distanceToLine({ 0, img_height }, { img_width, img_height }, pointC);

				float dist_to_hull_line = distanceToLine(pointA, pointB, pointC);

				if (dist_to_hull_line < shortest_hull_line_distance) {
					shortest_hull_line_distance = dist_to_hull_line;
				}

				cv::line(isolatedFloor, pointA, pointB, cv::Scalar(0, 0, 255), 2);

				if (dist_to_floor < TOO_CLOSE_TO_FLOOR_LINE) {
					skip = true;
					break;
				}
				// Point too close to one of the hull lines
				if (dist_to_hull_line < TOO_CLOSE_TO_ANY_HULL_LINE) {
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
				if (shortest_hull_line_distance > TOO_CLOSE_TO_CLOSEST_HULL_LINE) {
					potential_obstacle_points.push_back(pointC);
				}
				//cv::circle(isolatedFloor, pointC, 2, cv::Scalar(255, 0, 0), -2);
			}
		}

		// Group all the potential points by their distance to the closest other point.
		std::vector<std::vector<cv::Point2f>> separateGroups = groupPoints(potential_obstacle_points, OBSTACLE_CONTOUR_GROUPING_DISTANCE);

		// Get lowest points on the screen of each group of points.
		for (const std::vector<cv::Point2f>& group : separateGroups)
		{
			cv::Point lowest_point = { 0, 0 }; // Top of screen.
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
				if (abs(point.y - lowest_point.y) < LOWEST_POINT_DISTANCE_THRESHOLD) {
					obstacle_base_points.push_back(point);
				}
			}
			// Add only the lowest point vertically of each group (this would consider objects with long horizontal bases as only their lowest point).
			//obstacle_base_points.push_back(lowest_point);
		}
	}

#ifdef IN_PAPARAZZI
	writeImage(isolatedFloor, "isolated_floors");
#else
	cv::imshow("Floor", isolatedFloor);
#endif

	return obstacle_base_points;
}

std::vector<cv::Point> getGridPoints(
	const DroneState& state,
	std::vector<cv::Point2f> points,
	const cv::Size& img_size, 
	bool undistort) {
	if (undistort) {
		points = undistortPoints(points);
	}
    std::vector<cv::Point> grid_points;
    // Transform each point from camera view to bird's eye view (and relative to grid).
    for (const cv::Point& p : points) {
		if (p.x < 0 || p.x > img_size.width) continue;
		if (p.y < 0 || p.y > img_size.height) continue;
		cv::Point2f birds_eye_point = cameraToOptitrackBirdsEye(img_size, state.pos, state.heading, p);
        cv::Point grid_point = optitrack2DToGrid(birds_eye_point);
        if (!validGridPoint(grid_point)) continue;
        grid_points.emplace_back(grid_point);
    }
    return grid_points;
}

std::vector<cv::Point> detectObstacles(cv::Mat& image, const DroneState& state, std::vector<cv::Point2f>* out_points = NULL, bool undistort = true, const std::vector<cv::Point>& obstacles = {}, bool draw_outputs = true, cv::Mat* out_grid = NULL) {
  std::vector<cv::Point2f> points = processImageForObjects(image);
  if (out_points != NULL) {
    *out_points = points;
  }
  std::vector<cv::Point> grid_points = getGridPoints(state, points, image.size(), undistort);
  if (draw_outputs) {
    cv::Mat grid = cv::Mat(GRID_WIDTH, GRID_HEIGHT, CV_8UC3);
    grid.setTo(cv::Scalar(255, 255, 255));

    drawCarpet(grid);
    drawObstacles(grid, obstacles);
    drawDrone(grid, state);

    for (const cv::Point& gp : grid_points) {
      cv::circle(grid, gp, 2, cv::Scalar(255, 128, 128), -1);
    }

    for (int j = 0; j < GRID_HEIGHT; j++) {
      int offset = j * GRID_WIDTH;
      for (int i = 0; i < GRID_WIDTH; i++) {
        int index = i + offset;
        int timer = getTimer(index);
        if (timer > 0) {
          cv::circle(grid, cv::Point(i, j), 1, cv::Scalar(0, 0, 255), -1);
        }
      }
    }

    for (const cv::Point& p : points) {
        cv::circle(image, p, 2, cv::Scalar(255, 0, 0), -1);
    }
    *out_grid = grid;
  }
  return grid_points;
}

#endif