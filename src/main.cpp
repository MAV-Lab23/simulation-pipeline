#define IN_REAL_LIFE 0 // 0 = false, 1 = true 

#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "constants.h"
#include "image_processing.h"

void loop(cv::Mat& in_img, DroneState& state, const std::vector<Obstacle>& obstacles) {
    // Process a copy of the image so the original can be displayed later.
    cv::Mat original_img;
    in_img.copyTo(original_img);

    std::vector<cv::Point2f> points;
    std::vector<cv::Scalar> colors;

    bool use_test_points = false;
    
    if (use_test_points) {
        // Required to draw the intermediate screen but not used otherwise.
        processImageForObjects(in_img);
        fixDroneState(state);
        points = { { 520 / 2, 240 / 2 } /* center */, 
                   { 520 / 2 + 520 / 4, 240 / 2 } /* right */,
                   { 520 / 2 - 520 / 4, 240 / 2 } /* left */,
                   { 520 / 2, 240 / 2 + 240 / 4 } /* bottom */,
                   { 520 / 2, 240 / 2 - 240 / 4 } /* top */ };
        colors = {
            cv::Scalar(0, 0, 0) /* black */,
            cv::Scalar(255, 0, 0) /* blue */,
            cv::Scalar(19, 69, 139) /* brown */,
            cv::Scalar(214, 112, 218) /* purple */,
            cv::Scalar(0, 0, 255) /* red */,
        };
        assert(colors.size() == points.size());
    } else {
        points = processImageForObjects(in_img);
        colors.resize(points.size(), { 128, 128, 128 });
    }

    // change later part of this if you want to change these bools for non test points.
    bool undistort = !use_test_points && true;
    bool correct_pitch = !use_test_points && true;
    bool correct_longitude = !use_test_points && true;
    
    std::vector<cv::Point> grid_points = getGridPoints(
        in_img.size(),
        state,
        points,
        undistort,
        correct_pitch,
        correct_longitude);

    drawGrid(grid_points, colors, obstacles, state);

    // Display original and final images.
    cv::imshow("Image", original_img);
    cv::imshow("Final", in_img);

    // Pause before going to next frame.
    cv::waitKey(0);

    if (cv::waitKey(30) == 27) return; // Wait for 'esc' key press to exit
}

int main() {

    initDrawingWindows();

    bool old_data_files = false;

    std::vector<std::pair<cv::Mat, DroneState>> drone_data;
    std::vector<Obstacle> obstacles;

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
        const char* drone_images_directory = "../images/20240324-020231/";
        auto pair = getDroneDataNew(drone_images_directory, NED);
        drone_data = pair.first;
        obstacles = pair.second;
    }

    for (auto& [img, state] : drone_data) {
        loop(img, state, obstacles);
    }

    cv::waitKey(0);

    destroyDrawingWindows();

    std::cin.get();

    return 0;
}