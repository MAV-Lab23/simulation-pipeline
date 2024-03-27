#pragma once

#ifndef IN_PAPARAZZI

#include <opencv2/opencv.hpp>

#include "types.h"
#include "utility.h"

void initDrawingWindows() {
    cv::namedWindow("Image");
    cv::namedWindow("Floor");
    cv::namedWindow("Filtered");
    cv::namedWindow("Grid");
    cv::namedWindow("Intermediate1");
    cv::namedWindow("Intermediate2");
    cv::namedWindow("Undistorted");
    cv::namedWindow("Final");

    cv::moveWindow("Image", 0, 0);
    cv::moveWindow("Final", 0, 275);
    cv::moveWindow("Floor", 530, 0);
    cv::moveWindow("Undistorted", 530, 275);
    cv::moveWindow("Grid", 530 * 2, 0);
    cv::moveWindow("Intermediate1", 0, 275 * 2 + 9);
    cv::moveWindow("Intermediate2", 530, 275 * 2 + 9);
    cv::moveWindow("Filtered", 530 * 2, 275 * 2 + 9);
}

void destroyDrawingWindows() {
    cv::destroyAllWindows();
}

void drawObstacles(cv::Mat& out_grid, const std::vector<Obstacle>& obstacles) {
    int obstacle_radius = 5; // pixels
    cv::Scalar obstacle_color = { 0, 165, 255 }; // BGR
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        const Obstacle& o = obstacles[i];
        Vector2i obstacle_grid_pos = getObstacleGridPos(o.optitrack_pos);

        if (!validVectorInt(obstacle_grid_pos)) continue;

        cv::circle(out_grid, { obstacle_grid_pos.x, obstacle_grid_pos.y }, obstacle_radius, obstacle_color, -1);
    }
}

void drawGridPoints(cv::Mat& out_grid, const std::vector<cv::Point>& points, const std::vector<cv::Scalar>& colors) {
    int point_radius = 2;
    for (size_t i = 0; i < points.size(); i++) {
        cv::circle(out_grid, points[i], point_radius, colors[i], -1);
    }
}

void drawCarpet(cv::Mat& out_grid) {
    Vector2i top_left;
    Vector2i bottom_right;
    getCarpetCorners(&top_left, &bottom_right);
    cv::Scalar carpet_color = { 0, 250, 0 };
    cv::rectangle(out_grid, { top_left.x, top_left.y }, { bottom_right.x, bottom_right.y }, carpet_color, -1);
}

void drawHeading(cv::Mat& out_grid, const Vector2i& pos, float heading, float heading_length, const cv::Scalar& line_color, int thickness) {
    // If the drone is outside of the grid boundaries do not draw it.
    if (!validVectorInt(pos)) return;

    int x_dir = heading_length / METERS_PER_GRID_CELL.x * -cos(heading);
    int y_dir = heading_length / METERS_PER_GRID_CELL.y * -sin(heading);

    cv::Point end_pos;
    end_pos.x = (int)clamp(pos.x + x_dir, 0, GRID_SIZE.x);
    end_pos.y = (int)clamp(pos.y + y_dir, 0, GRID_SIZE.y);
        
    cv::Point d = { pos.x, pos.y };
    cv::line(out_grid, d, end_pos, line_color, thickness);
}

// Grid heading in radians.
void drawDrone(cv::Mat& out_grid, const DroneState& state) {
    Vector2i grid_pos = optitrackCoordinateToGrid({ state.optitrack_pos.x, state.optitrack_pos.y });
    int heading_length = 1;
    drawHeading(out_grid, grid_pos, state.optitrack_angle.z, heading_length, cv::Scalar(128, 128, 128), 2);
    cv::Point d = { grid_pos.x, grid_pos.y };
    // Drone point
    int drone_radius = 6;
    cv::Scalar drone_color = { 0, 0, 0 };
    cv::circle(out_grid, d, drone_radius, drone_color, -1);
}

void drawGrid(
    std::vector<cv::Point>& grid_points,
    const std::vector<cv::Scalar>& colors,
    const std::vector<Obstacle>& obstacles,
    const DroneState& state) {
    // OpenCV grid visualization.
    cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);
    grid.setTo(cv::Scalar(255, 255, 255));
    drawCarpet(grid);
    drawObstacles(grid, obstacles);
    drawGridPoints(grid, grid_points, colors);
    drawDrone(grid, state);
    cv::imshow("Grid", grid);
}

#endif