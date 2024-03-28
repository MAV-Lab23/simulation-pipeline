#ifndef GROUP_10_DRAW_H
#define GROUP_10_DRAW_H

#include <string>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#include <opencv2/opencv.hpp>

#include "drone.h"
#include "transform.h"
#include "utility.h"

#ifdef IN_PAPARAZZI

#include "modules/computer_vision/video_capture.h"

#ifndef PRINT
#define PRINT(string,...) fprintf(stderr, "[writeImage->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

// Change this to false if you wish to not record vision processed images.
#define WRITE_REALTIME_PROCESSING_IMAGES true 

#ifndef VIDEO_CAPTURE_PATH
#define VIDEO_CAPTURE_PATH /data/ftp/internal_000/images
#endif

extern int image_process_loops;

void writeImage(const cv::Mat& image, const std::string& sub_directory) {
    if (WRITE_REALTIME_PROCESSING_IMAGES && video_capture_record_video) {
        std::string directory = std::string(STRINGIFY(VIDEO_CAPTURE_PATH)) + std::string("/../") + sub_directory + std::string("/");
        if (access(directory.c_str(), F_OK)) {
            char save_dir_cmd[266]; // write 10b + [0:256]
            sprintf(save_dir_cmd, "mkdir -p %s", directory.c_str());
            if (system(save_dir_cmd) != 0) {
                printf("[video_capture] Could not create sub directory for processed images: %s.\n", sub_directory.c_str());
                return;
            }
        }
        std::string image_path = directory + std::to_string(image_process_loops) + std::string(".jpg");
        cv::imwrite(image_path, image);
    }
}

#else

#define WRITE_REALTIME_PROCESSING_IMAGES false 

void initDrawingWindows() {
    cv::namedWindow("Image");
    cv::namedWindow("Floor");
    cv::namedWindow("Filtered");
    cv::namedWindow("Grid");
    cv::namedWindow("Contour");
    cv::namedWindow("Hull");
    cv::namedWindow("Undistorted");
    cv::namedWindow("Final");

    cv::moveWindow("Image", 0, 0);
    cv::moveWindow("Final", 0, 275);
    cv::moveWindow("Floor", 530, 0);
    cv::moveWindow("Undistorted", 530, 275);
    cv::moveWindow("Grid", 530 * 2, 0);
    cv::moveWindow("Contour", 0, 275 * 2 + 9);
    cv::moveWindow("Hull", 530, 275 * 2 + 9);
    cv::moveWindow("Filtered", 530 * 2, 275 * 2 + 9);
}

void destroyDrawingWindows() {
    cv::destroyAllWindows();
}

#endif

void drawObstacles(cv::Mat& out_grid, const std::vector<cv::Point>& obstacles) {
    int obstacle_radius = 5; // pixels
    cv::Scalar obstacle_color = { 0, 165, 255 }; // BGR
    for (const cv::Point& obstacle : obstacles) {
        if (!validGridPoint(obstacle)) continue;
        cv::circle(out_grid, obstacle, obstacle_radius, obstacle_color, -1);
    }
}

void drawCarpet(cv::Mat& out_grid) {
    cv::Point top_left;
    cv::Point bottom_right;
    getCarpetCornerGridPoints(&top_left, &bottom_right);
    cv::Scalar carpet_color = { 0, 250, 0 };
    cv::rectangle(out_grid, top_left, bottom_right, carpet_color, -1);
}

void drawHeading(cv::Mat& out_grid, const cv::Point& pos,
    float heading, float heading_length, const cv::Scalar& line_color, int thickness) {
    // If the drone is outside of the grid boundaries do not draw it.
    if (!validGridPoint(pos)) return;

    int x_dir = heading_length / METERS_PER_GRID_CELL_X * -cos(heading);
    int y_dir = heading_length / METERS_PER_GRID_CELL_Y * -sin(heading);

    cv::Point end_pos;
    end_pos.x = (int)clamp(pos.x + x_dir, 0, GRID_WIDTH);
    end_pos.y = (int)clamp(pos.y + y_dir, 0, GRID_HEIGHT);
        
    cv::line(out_grid, pos, end_pos, line_color, thickness);
}

// Grid heading in radians.
void drawDrone(cv::Mat& out_grid, const DroneState& state) {
    cv::Point grid_pos = optitrack3DToGrid(state.pos);
    int heading_length = 1;
    drawHeading(out_grid, grid_pos, state.heading.z, heading_length, cv::Scalar(128, 128, 128), 2);
    // Drone point
    int drone_radius = 6;
    cv::Scalar drone_color = { 0, 0, 0 };
    cv::circle(out_grid, grid_pos, drone_radius, drone_color, -1);
}

#endif