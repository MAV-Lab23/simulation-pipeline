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

        Vector2f pos;
        pos.x = normalizeValue(o.optitrack_pos.x, -(ARENA_SIZE.x + 2) / 2.0, (ARENA_SIZE.x + 2) / 2.0);
        pos.y = normalizeValue(o.optitrack_pos.y, -(ARENA_SIZE.y + 2) / 2.0, (ARENA_SIZE.y + 2) / 2.0);

        Vector2i obstacle_grid_pos = { (int)(pos.x * GRID_SIZE.x), (int)(pos.y * GRID_SIZE.y) };




        if (!validVector(obstacle_grid_pos)) continue;

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
    // Calculate carpet start and end points.
    // Top left
    int tl_x = (ARENA_SIZE.x - CARPET_SIZE.x) / (2 * ARENA_SIZE.x) * GRID_SIZE.x;
    int tl_y = (ARENA_SIZE.y - CARPET_SIZE.y) / (2 * ARENA_SIZE.y) * GRID_SIZE.y;
    // Bottom right
    int br_x = tl_x + CARPET_SIZE.x / ARENA_SIZE.x * GRID_SIZE.x;
    int br_y = tl_y + CARPET_SIZE.y / ARENA_SIZE.y * GRID_SIZE.y;

    cv::Scalar carpet_color = { 0, 250, 0 };
    cv::rectangle(out_grid, { tl_x, tl_y }, { br_x, br_y }, carpet_color, -1);
}

// Grid heading in radians.
void drawDrone(cv::Mat& out_grid, const DroneState& state) {
    Vector2i grid_pos = optitrackCoordinateToGrid({ state.optitrack_pos.x, state.optitrack_pos.y });
    //Vector2i grid_pos_cam = optitrackCoordinateToGrid({ state.optitrack_pos.x + x_cam, state.optitrack_pos.y + y_cam });

    // If the drone is outside of the grid boundaries do not draw it.
    if (!validVector(grid_pos)) return;

    int heading_length = 30;

    float heading = state.optitrack_angle.z;

    int x_dir = heading_length * -cos(heading);
    int y_dir = heading_length * -sin(heading);

    cv::Point end_pos;
    end_pos.x = (int)clamp(grid_pos.x + x_dir, 0, GRID_SIZE.x);
    end_pos.y = (int)clamp(grid_pos.y + y_dir, 0, GRID_SIZE.y);
        
    // Drone point
    int drone_radius = 6;
    cv::Scalar drone_color = { 0, 0, 0 };

    cv::Point d = { grid_pos.x, grid_pos.y };
    //cv::Point c = { grid_pos_cam.x, grid_pos_cam.y };

    cv::circle(out_grid, d, drone_radius, drone_color, -1);
    //cv::circle(out_grid, c, drone_radius, cv::Scalar(128), -1);
    cv::line(out_grid, d, end_pos, cv::Scalar(128), 1);
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