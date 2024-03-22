#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "constants.h"
#include "image_processing.h"

void processImage(const DroneData& drone_data) {
    // Draw drone position on grid.
    DroneState drone_state = drone_data.state;

    Vector3f drone_pos_norm = { normalizeValue(drone_state.optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                normalizeValue(drone_state.optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2),
                                normalizeValue(drone_state.optitrack_pos.z, -ARENA_SIZE.z / 2, ARENA_SIZE.z / 2) };

    Vector2i drone_pos_grid = { (int)(drone_pos_norm.x * GRID_SIZE.x), (int)(drone_pos_norm.y * GRID_SIZE.y) };

    // drone_pos_grid_clamped
    Vector2i drone_pos = { (int)clamp(drone_pos_grid.x, 0, GRID_SIZE.x - 1), (int)clamp(drone_pos_grid.y, 0, GRID_SIZE.y - 1) };

    int dir_magnitude = 30;

    int x_dir = dir_magnitude * cos(drone_state.optitrack_angle.z);
    int y_dir = dir_magnitude * sin(drone_state.optitrack_angle.z);

    Vector2i start_pos = Vector2i{ (int)clamp(drone_pos.x, 0, GRID_SIZE.x), (int)clamp(drone_pos.y, 0, GRID_SIZE.y) };
    Vector2i end_pos = Vector2i{ (int)clamp(drone_pos.x + x_dir, 0, GRID_SIZE.x), (int)clamp(drone_pos.y + y_dir, 0, GRID_SIZE.y) };
        
    Image img;
    drone_data.image.copyTo(img);

    std::vector<cv::Point> objectDistances2 = processImageForObjects(img);
    
    std::vector<cv::Point> objectDistances;
    cv::Point p1 = { 156, 240 };
    cv::Point p2 = { 364, 240 };
    cv::Point p3 = { 0, 240 };
    cv::Point p4 = { 520, 240 };
    cv::Point p5 = { 156, 100 };
    cv::Point p6 = { 364, 100 };
    cv::Point p7 = { 0, 100 };
    cv::Point p8 = { 520, 100 };
    objectDistances.push_back(p1);
    objectDistances.push_back(p2);
    objectDistances.push_back(p3);
    objectDistances.push_back(p4);
    objectDistances.push_back(p5);
    objectDistances.push_back(p6);
    objectDistances.push_back(p7);
    objectDistances.push_back(p8);
    
        
        
    cv::MatSize size = img.size;
    // OpenCV image size gives [h, w]
    cv::Point center = { size[1] / 2, size[0] / 2 };

    cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);

    int top_left_x = ((ARENA_SIZE.x - CARPET_SIZE.x) / 2) / ARENA_SIZE.x * GRID_SIZE.x;
    int top_left_y = ((ARENA_SIZE.y - CARPET_SIZE.y) / 2) / ARENA_SIZE.y * GRID_SIZE.y;
    int bottom_right_x = top_left_x + CARPET_SIZE.x / ARENA_SIZE.x * GRID_SIZE.x;
    int bottom_right_y = top_left_y + CARPET_SIZE.y / ARENA_SIZE.y * GRID_SIZE.y;

    // Draw green carpet
    cv::rectangle(grid, cv::Point(top_left_x, top_left_y), cv::Point(bottom_right_x, bottom_right_y), cv::Scalar(0, 250, 0), -1);

    // Draw lines from center of screen to found obstacles.
    for (size_t i = 0; i < objectDistances.size(); i++)
    {
        Vector2i point_cam_pos = { objectDistances[i].x, objectDistances[i].y };
        

        DroneState modified_drone_state = drone_state;
        
        float CAMERA_TILT = degToRad(20.0);
        modified_drone_state.optitrack_angle.y -= CAMERA_TILT;

        /*
        modified_drone_state.optitrack_pos.x = 0;
        modified_drone_state.optitrack_pos.y = 0;
        modified_drone_state.optitrack_pos.z = -1;
        modified_drone_state.optitrack_angle.y = 0;
        modified_drone_state.optitrack_angle.x = -0.785398163;
        modified_drone_state.optitrack_angle.z = 0;
        point_cam_pos = { 520 / 2, 240 / 2 };
        */

        Vector2f point_optitrack_pos = getObstacleGridPosition(grid, { 520, 240 }, point_cam_pos, degToRad(DRONE_FOV_ANGLE), modified_drone_state);
        
        //std::cout << "yaw: " << radToDeg(modified_drone_state.optitrack_angle.x) << ", pitch: " << radToDeg(modified_drone_state.optitrack_angle.y) << ", roll: " << radToDeg(modified_drone_state.optitrack_angle.z) << std::endl;

        //std::cout << "x: " << point_optitrack_pos.x << ", y: " << point_optitrack_pos.y << std::endl;

        Vector2f point_norm_pos = { normalizeValue(point_optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                    normalizeValue(point_optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2) };

        Vector2i point_grid_pos = { (int)(point_norm_pos.x * GRID_SIZE.x), (int)(point_norm_pos.y * GRID_SIZE.y) };

        Vector2i point_grid_pos_clamped = { (int)clamp(point_grid_pos.x, 0, GRID_SIZE.x - 1), (int)clamp(point_grid_pos.y, 0, GRID_SIZE.y - 1) };

        cv::circle(grid, cv::Point(point_grid_pos_clamped.x, point_grid_pos_clamped.y), 2, cv::Scalar(255, 0, 0), -1);
        cv::line(img, cv::Point(drone_pos.x, drone_pos.y), cv::Point(point_grid_pos_clamped.x, point_grid_pos_clamped.y), cv::Scalar(0));

        cv::line(img, { center.x, center.y }, objectDistances[i], cv::Scalar(0));
    }

    cv::circle(grid, cv::Point(drone_pos.x, drone_pos.y), DRONE_RADIUS, cv::Scalar(0), -1);
    cv::line(grid, cv::Point(start_pos.x, start_pos.y), cv::Point(end_pos.x, end_pos.y), cv::Scalar(128), 2);

    // Display images.
    cv::imshow("Image", drone_data.image);
    cv::imshow("Final", img);
    cv::imshow("Grid", grid);

    // Pause before going to next frame.
    cv::waitKey(0);

    if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
    {
        return;
    }
}

int main() {

    initDrawingWindows();

    // Directory path to drone images relative to src directory
    const char* drone_images_directory = "../images/run2/";
    const char* drone_data_directory = "../data/";
    const char* cache_data_directory = "../cache/";
    const char* drone_data_file = "run2.csv";

    std::vector<DroneData> drone_data = getDroneData(drone_images_directory, cache_data_directory, drone_data_directory, drone_data_file);
    for (auto& data : drone_data) {
        processImage(data);
    }
    
    cv::waitKey(0);

    destroyDrawingWindows();

    return 0;
}
