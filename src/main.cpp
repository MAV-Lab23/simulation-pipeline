// DO NOT FORGET TO SET THIS WHEN TESTING WITH REAL WORLD DATA 
#define IN_REAL_LIFE 0 // 0 = false, 1 = true 

#define GROUP_10_OPENCV 1

#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "constants.h"
#include "image_processing.h"
#include "navigation.h"

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
        //points = opticalFlow(in_img, points);
        colors.resize(points.size(), { 128, 128, 128 });
    }

    std::vector<cv::Point2f> ref_points;
    ref_points = {
                   { 0 + 1, 240 - 1 }, /* bottom left */
                   { 520 / 2, 240 - 1 }, /* bottom center */
                   { 520 - 1, 240 - 1 }, /* bottom right */

                   { 520 / 2 - 10, 240 }, /* bottom center */
                   { 520 / 2 - 20, 240 }, /* bottom center */
                   { 520 / 2 - 30, 240 }, /* bottom center */
                   { 520 / 2 - 40, 240 }, /* bottom center */
                   { 520 / 2 - 50, 240 }, /* bottom center */
                   { 520 / 2 + 10, 240 }, /* bottom center */
                   { 520 / 2 + 20, 240 }, /* bottom center */
                   { 520 / 2 + 30, 240 }, /* bottom center */
                   { 520 / 2 + 40, 240 }, /* bottom center */
                   { 520 / 2 + 50, 240 } /* bottom center */
                   
                //    { 0 + 1, 0 + 1 }, /* top left */
                //    { 520 / 2, 0 + 1 }, /* top center */
                //    { 520 - 1, 0 + 1 }, /* top right */
                //    { 0 + 1, 240 / 2 }, /* left */
                //    { 520 / 2, 240 / 2 }, /* center */
                //    { 520 - 1, 240 / 2 }, /* right */
    };
    std::vector<cv::Scalar> ref_colors = {
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */,
        cv::Scalar(214, 112, 218) /* purple */
        //cv::Scalar(0, 0, 255) /* red */,
        //cv::Scalar(0, 0, 255) /* red */,
        //cv::Scalar(0, 0, 255) /* red */,
        //cv::Scalar(255, 0, 0) /* blue */,
        //cv::Scalar(255, 0, 0) /* blue */,
        //cv::Scalar(255, 0, 0) /* blue */
    };

    // change later part of this if you want to change these bools for non test points.
    bool undistort = !use_test_points && true;
    bool correct_pitch = !use_test_points && true;
    bool correct_longitude = !use_test_points && true;
    
    std::vector<cv::Point> u_grid_points = getGridPoints(
        state,
        points,
        original_img.size(),
        true,
        false,
        false);
    std::vector<cv::Point> d_grid_points = getGridPoints(
        state,
        points,
        original_img.size(),
        false,
        false,
        false);

    cv::Mat undistorted = undistortImage(original_img);

    int point_radius = 2;
    for (size_t i = 0; i < ref_points.size(); i++) {
        cv::circle(in_img, { (int)ref_points[i].x, (int)ref_points[i].y }, point_radius, ref_colors[i], -1);
        cv::circle(undistorted, { (int)ref_points[i].x, (int)ref_points[i].y }, point_radius, ref_colors[i], -1);
    }

    std::vector<cv::Point> ref_grid_points = getGridPoints(
        state,
        ref_points,
        original_img.size(),
        true,
        false,
        false);

    std::vector<cv::Point2f> undistorted_points = undistortPoints(points);
    std::vector<cv::Point2f> undistorted_ref_points = undistortPoints(ref_points);
    //printf("undistorted_points size: %i\n", undistorted_points.size());

    cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);
    grid.setTo(cv::Scalar(255, 255, 255));

    drawCarpet(grid);
    drawObstacles(grid, obstacles);
    
    drawDrone(grid, state);

    for (size_t i = 0; i < ref_grid_points.size(); i++) {
        cv::circle(grid, { (int)ref_grid_points[i].x, (int)ref_grid_points[i].y }, point_radius, ref_colors[i], -1);
    }

    for (size_t i = 0; i < u_grid_points.size(); i++) {
        cv::circle(grid, { (int)u_grid_points[i].x, (int)u_grid_points[i].y }, point_radius, cv::Scalar(255, 128, 128), -1);
    }

    //for (size_t i = 0; i < d_grid_points.size(); i++) {
        //cv::circle(grid, { (int)d_grid_points[i].x, (int)d_grid_points[i].y }, point_radius, cv::Scalar(128, 128, 128), -1);
    //}

    // Add identified points to grid and set their timers.
    for (size_t i = 0; i < u_grid_points.size(); i++)
    {
	    int index = u_grid_points[i].x + GRID_SIZE.x * u_grid_points[i].y;
        addGridElement(index);
    }

    Vector2i drone_grid_pos = getObjectGridPosition(state.optitrack_pos.x, state.optitrack_pos.y);
    Vector2i closest_cell = updateGrid(drone_grid_pos, true);
    Vector2i best_endpoint = { 0, 0 };

    float best_heading = getBestHeading(grid, drone_grid_pos, state.optitrack_angle.z, &best_endpoint);

    //drawHeading(grid, drone_grid_pos, state.optitrack_angle.z + degToRad(45), 1, cv::Scalar(128, 128, 128), 1);

	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
        for (int i = 0; i < GRID_SIZE.x; i++)
        {
			int index = i + offset;
			int timer = getTimer(index);
            if (timer > 0) {
                cv::circle(grid, cv::Point(i, j), 1, cv::Scalar(0, 0, 255), -1);
            }
        }
	}

    cv::imshow("Grid", grid);

    for (size_t i = 0; i < undistorted_ref_points.size(); i++) {
        cv::circle(undistorted, { (int)undistorted_ref_points[i].x, (int)undistorted_ref_points[i].y }, point_radius, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < undistorted_points.size(); i++) {
        cv::circle(undistorted, {(int)undistorted_points[i].x, (int)undistorted_points[i].y}, point_radius, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < ref_points.size(); i++) {
        cv::circle(original_img, { (int)ref_points[i].x, (int)ref_points[i].y }, point_radius, cv::Scalar(255, 0, 0), -1);
    }

    for (size_t i = 0; i < points.size(); i++) {
        cv::circle(original_img, {(int)points[i].x, (int)points[i].y}, point_radius, cv::Scalar(255, 0, 0), -1);
    }

    // Display original and final images.
    cv::imshow("Image", original_img);
    cv::imshow("Final", in_img);

    cv::imshow("Undistorted", undistorted);

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
        // 20240326-081515 // 1.25 zoom
        // 20240324-020231 // 1.0 zoom
        // 20240327-230719 // 1.0 zoom
        const char* drone_images_directory = "../images/20240327-230719/";
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