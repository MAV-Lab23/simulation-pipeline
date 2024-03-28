// DO NOT FORGET TO SET THIS WHEN TESTING WITH REAL WORLD DATA
#define IN_REAL_LIFE 0 // 0 = false, 1 = true 

#include "image_processing.h"
#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "constants.h"
#include "navigation.h"
#include "file.h"

float probabilities[GRID_LENGTH] = { 0 };
int timers[GRID_LENGTH] = { 0 };

#define UNDISTORT true

void parseImage(cv::Mat& image, DroneState& state, const std::vector<cv::Point>& obstacles) {
    // Process a copy of the image so the original can be displayed later.
    cv::Mat original_img;
    image.copyTo(original_img);

    cv::Mat grid;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point> grid_points = detectObstacles(image, state, &points, true, obstacles, true, &grid);

    // Add identified points to grid and set their timers.
    for (const cv::Point& gp : grid_points) {
	    // Convert 2D to 1D coordinate.
        addGridElement(gp.x + GRID_WIDTH * gp.y);
    }

    float best_heading = updateNavigation(state, grid, true, true);

    cv::Mat undistorted = undistortImage(original_img);
    std::vector<cv::Point2f> u_points = undistortPoints(points);
    
    for (const cv::Point& up : u_points) {
        cv::circle(undistorted, up, 2, cv::Scalar(255, 0, 0), -1);
    }

    // Display original and final images as well as grid.
    cv::imshow("Image", original_img);
    cv::imshow("Final", image);
    cv::imshow("Undistorted", undistorted);
    cv::imshow("Grid", grid);

    // Pause before going to next frame.
    cv::waitKey(0);

    if (cv::waitKey(30) == 27) return; // Wait for 'esc' key press to exit
}

int main() {

    initDrawingWindows();

    bool old_data_files = false;

    std::vector<std::pair<cv::Mat, DroneState>> drone_data;
    std::vector<cv::Point> obstacles;

    if (old_data_files) {
        // Directory paths relative to src directory.
        drone_data = getDroneData(
            "../images/run1/",
            "run1.csv",
            "../data/",
            "../cache/",
            NED
        );
    } else {
        // Directory path relative to src directory.
        // 20240326-081515 // 1.25 zoom
        // 20240324-020231 // 1.0 zoom
        // 20240327-230719 // 1.0 zoom
        const char* drone_images_directory = "../images/20240324-020231/";
        // I wish bebop had above C++ 11 for auto :)
        std::pair<std::vector<std::pair<cv::Mat, DroneState>>, std::vector<cv::Point>> pair = getDroneDataNew(drone_images_directory, NED);
        drone_data = pair.first;
        obstacles = pair.second;
    }

    for (std::pair<cv::Mat, DroneState>& pair : drone_data) {
        // img, state, obstacles.
        parseImage(pair.first, pair.second, obstacles);
    }

    cv::waitKey(0);

    destroyDrawingWindows();

    std::cin.get();

    return 0;
}