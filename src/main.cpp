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
        if (gp.x >= 0 && gp.x < GRID_WIDTH && gp.y >= 0 && gp.y < GRID_HEIGHT) { 
            // Convert 2D to 1D coordinate.
            addGridElement(gp.x + GRID_WIDTH * gp.y);
        }
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
        std::string real_life_bush = "../images/real_life/run1/";
        std::string real_life_low_flying = "../images/real_life/run3/";
        std::string real_life_flying = "../images/real_life/run4/";

        std::string dir = real_life_bush;

        drone_data = getDroneData(
            dir,
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
        std::string orange_poles_1 = "../images/orange_poles/20240324-020231/";
        std::string floor_mats_one = "../images/floor_mats/20240327-230719/";
        std::string floor_mats_many = "../images/floor_mats/20240328-094836/";
        std::string metal_plate_collision_and_success = "../images/metal_plates/20240328-014412/";
        std::string best_run = "../images/mix/20240328-135410/";

        std::string dir = best_run;//orange_poles_1;
        // I wish bebop had above C++ 11 for auto :)
        std::pair<std::vector<std::pair<cv::Mat, DroneState>>, std::vector<cv::Point>> pair = getDroneDataNew(dir, NED);
        drone_data = pair.first;
        // TODO: Obstacle positions / rotations need to be fixed.
        //obstacles = pair.second;
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
