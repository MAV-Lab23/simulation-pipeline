#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "image_processing.h"

void processImage(const DroneData& drone_data) {
    uchar* grid_pointer = NULL;

    cv::Mat grid = createGrid(grid_pointer, GRID_SIZE.x, GRID_SIZE.y);

    // Draw drone position on grid.
    const DroneState& drone_state = drone_data.state;

    Vector3f drone_pos_norm = { normalizeValue(drone_state.optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                normalizeValue(drone_state.optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2),
                                normalizeValue(drone_state.optitrack_pos.z, -ARENA_SIZE.z / 2, ARENA_SIZE.z / 2) };

    Vector2i drone_pos_grid = { (int)(drone_pos_norm.x * GRID_SIZE.x), (int)(drone_pos_norm.y * GRID_SIZE.y) };

    // drone_pos_grid_clamped
    Vector2i drone_pos = { (int)clamp(drone_pos_grid.x, 0, GRID_SIZE.x - 1), (int)clamp(drone_pos_grid.y, 0, GRID_SIZE.y - 1) };


    int dir_mag = 30;

    int x_dir = dir_mag * cos(drone_state.optitrack_angle.z);
    int y_dir = dir_mag * sin(drone_state.optitrack_angle.z);

    Vector2i start_pos = Vector2i{ (int)clamp(drone_pos.x, 0, GRID_SIZE.x), (int)clamp(drone_pos.y, 0, GRID_SIZE.y) };
    Vector2i end_pos = Vector2i{ (int)clamp(drone_pos.x + x_dir, 0, GRID_SIZE.x), (int)clamp(drone_pos.y + y_dir, 0, GRID_SIZE.y) };
        
    Image img;
    drone_data.image.copyTo(img);

    std::vector<cv::Point> objectDistances = processImageForObjects(img);
        
    cv::MatSize size = img.size;
    // OpenCV image size gives [h, w]
    cv::Point center = { size[1] / 2, size[0] / 2 };

    // Draw lines from center of screen to found obstacles.
    for (size_t i = 0; i < objectDistances.size(); i++)
    {
        Vector2i point = { objectDistances[i].x, objectDistances[i].y };

        Vector2f grid_offset_pos = getObstacleGridPosition({ 520, 240 }, point, degToRad(DRONE_FOV_ANGLE), drone_state);

        Vector2f point_opti_pos = { normalizeValue(drone_state.optitrack_pos.x + grid_offset_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                    normalizeValue(drone_state.optitrack_pos.y + grid_offset_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2) };

        Vector2i point_pos_grid = { (int)(point_opti_pos.x * GRID_SIZE.x), (int)(point_opti_pos.y * GRID_SIZE.y) };

        Vector2i point_pos = { (int)clamp(point_pos_grid.x, 0, GRID_SIZE.x - 1), (int)clamp(point_pos_grid.y, 0, GRID_SIZE.y - 1) };

        cv::circle(grid, cv::Point(point_pos.x, point_pos.y), 2, cv::Scalar(0), -1);
        cv::line(img, { center.x, size[1] }, objectDistances[i], cv::Scalar(0));
    }

    cv::circle(grid, cv::Point(drone_pos.x, drone_pos.y), DRONE_RADIUS, cv::Scalar(0), -1);
    cv::line(grid, cv::Point(start_pos.x, start_pos.y), cv::Point(end_pos.x, end_pos.y), cv::Scalar(0), 2);

    // Display images.
    cv::imshow("Image", drone_data.image);
    cv::imshow("Final", img);
    cv::imshow("Grid", grid);

    // Pause before going to next frame.
    //cv::waitKey(0);

    if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
    {
        return;
    }

    destroyGrid(grid_pointer);
}

int main() {

    initDrawingWindows();

    // Directory path to drone images relative to src directory
    const char* drone_images_directory = "../images/run1/";
    const char* drone_data_directory = "../data/";
    const char* cache_data_directory = "../cache/";
    const char* drone_data_file = "run1.csv";

    std::vector<DroneData> drone_data = getDroneData(drone_images_directory, cache_data_directory, drone_data_directory, drone_data_file);
    for (auto& data : drone_data) {
        processImage(data);
    }
    
    cv::waitKey(0);

    destroyDrawingWindows();

    return 0;
}
