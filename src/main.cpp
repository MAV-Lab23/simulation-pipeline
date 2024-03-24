#include "draw.h"
#include "drone.h"
#include "utility.h"
#include "constants.h"
#include "image_processing.h"
#include "path_planning.h"

void processImage(const DroneData& drone_data, std::vector<Obstacle> obstacles, bool old_data_files) {//, cv::Mat& prev_grid) {
    // Draw drone position on grid.
    DroneState drone_state = drone_data.state;

    Vector2f drone_pos_norm = { normalizeValue(drone_state.optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                normalizeValue(drone_state.optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2) };

    //printf("%.1f, %.1f, %.1f | %.1f, %.1f, %.1f \n", drone_pos_norm.x, drone_pos_norm.y, drone_state.optitrack_pos.z, radToDeg(drone_state.optitrack_angle.x), radToDeg(drone_state.optitrack_angle.y), radToDeg(drone_state.optitrack_angle.z));

    Vector2i drone_pos_grid = { (int)(drone_pos_norm.x * GRID_SIZE.x), (int)(drone_pos_norm.y * GRID_SIZE.y) };

    // drone_pos_grid_clamped
    // TODO: Remove this clamping as it is technically not correct.
    Vector2i drone_pos = { (int)clamp(drone_pos_grid.x, 0, GRID_SIZE.x - 1), (int)clamp(drone_pos_grid.y, 0, GRID_SIZE.y - 1) };

    int dir_magnitude = 30;

    int x_dir = dir_magnitude * cos(drone_state.optitrack_angle.z);
    int y_dir = dir_magnitude * sin(drone_state.optitrack_angle.z);

    Vector2i start_pos = Vector2i{ (int)clamp(drone_pos.x, 0, GRID_SIZE.x), (int)clamp(drone_pos.y, 0, GRID_SIZE.y) };
    Vector2i end_pos = Vector2i{ (int)clamp(drone_pos.x + x_dir, 0, GRID_SIZE.x), (int)clamp(drone_pos.y + y_dir, 0, GRID_SIZE.y) };
        
    Image img;
    //// Define camera matrix and distortion coefficients
    //cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 254.80142347, 0, 272.9569783,
    //    0, 246.12646616, 173.41638933,
    //    0, 0, 1);
    //cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << 0.03678781, -0.05588154, 0.00051716, -0.01505311);

    //cv::undistort(drone_data.image, img, cameraMatrix, distCoeffs);
    drone_data.image.copyTo(img);

    /*cv::imshow("DistortedImage", drone_data.image);
    cv::imshow("UndistortedImage", img);
    cv::waitKey(0);*/

    std::vector<cv::Point> objectImagePointsInt = processImageForObjects(img);
   
    // Define camera matrix and distortion coefficients
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 187.81447443, 0, 261.61616548,
        0, 186.06739848, 128.43303997,
        0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << -0.0169843, 0.0194144, -0.01150561, 0.00194345);

    // Convert cv::Point to cv::Point2f
    std::vector<cv::Point2f> objectImagePoints;
    for (size_t i = 0; i < objectImagePointsInt.size(); ++i) {
        objectImagePoints.push_back(cv::Point2f(static_cast<float>(objectImagePointsInt[i].x),
            static_cast<float>(objectImagePointsInt[i].y)));
    }

    // Vector for storing undistorted points
    std::vector<cv::Point2f> undistortedObjectPoints;
    std::vector<cv::Point> undistortedObjectPointsInt;
    if (!objectImagePoints.empty()) {
        // Undistort points
        cv::undistortPoints(objectImagePoints, undistortedObjectPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

        // Optionally, convert back to cv::Point if you need integer coordinates after undistortion
        for (size_t i = 0; i < undistortedObjectPoints.size(); ++i) {
            undistortedObjectPointsInt.push_back(cv::Point(static_cast<int>(undistortedObjectPoints[i].x),
                static_cast<int>(undistortedObjectPoints[i].y)));
        }

        //// Print out original and undistorted points
        //for (size_t i = 0; i < objectImagePointsInt.size(); ++i) {
        //    std::cout << "Original: " << objectImagePointsInt[i] << " Undistorted: " << undistortedObjectPointsInt[i] << std::endl;
        //}
    }
    else {
        //std::cout << "No points to undistort!" << std::endl;
    }
        
    cv::MatSize size = img.size;
    // OpenCV image size gives [h, w]
    cv::Point center = { size[1] / 2, size[0] / 2 };

    cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);
    grid.setTo(cv::Scalar(255,255,255));

    int top_left_x = ((ARENA_SIZE.x - CARPET_SIZE.x) / 2) / ARENA_SIZE.x * GRID_SIZE.x;
    int top_left_y = ((ARENA_SIZE.y - CARPET_SIZE.y) / 2) / ARENA_SIZE.y * GRID_SIZE.y;
    int bottom_right_x = top_left_x + CARPET_SIZE.x / ARENA_SIZE.x * GRID_SIZE.x;
    int bottom_right_y = top_left_y + CARPET_SIZE.y / ARENA_SIZE.y * GRID_SIZE.y;

    // Draw green carpet.
    cv::rectangle(grid, cv::Point(top_left_x, top_left_y), cv::Point(bottom_right_x, bottom_right_y), cv::Scalar(0, 250, 0), -1);

    // Draw obstacles.
    if (!old_data_files) {

        int obstacle_radius = 5; // pixels

        for (size_t i = 0; i < obstacles.size(); i++)
        {
            const Obstacle& o = obstacles[i];

            Vector2f o_norm_pos = { normalizeValue(o.optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                    normalizeValue(o.optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2) };

            Vector2i o_grid_pos = { (int)(o_norm_pos.x * GRID_SIZE.x), (int)(o_norm_pos.y * GRID_SIZE.y) };

            Vector2i o_grid_pos_clamped = { (int)clamp(o_grid_pos.x, 0, GRID_SIZE.x - 1), (int)clamp(o_grid_pos.y, 0, GRID_SIZE.y - 1) };

            cv::circle(grid, { o_grid_pos_clamped.x, o_grid_pos_clamped.y }, 5, cv::Scalar(0, 165, 255), -1);
        }
    }

    //cv::Mat grid;
    //prev_grid.copyTo(grid);

    Vector2i* obstacle_list = NULL;
    int length = 0;
    //Vector2f obstacle_list =  
    // Draw lines from center of screen to found obstacles.
    for (size_t i = 0; i < objectGridPoints.size(); i++)
    {
        Vector2i point_cam_pos = { undistortedObjectPointsInt[i].x, undistortedObjectPointsInt[i].y };

        DroneState modified_drone_state = drone_state;
        
        float CAMERA_TILT = degToRad(18.9);
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

        obstacle_list = append_Vector_Element(obstacle_list, &length, point_grid_pos_clamped);
        //printf("%f", obstacle_list[0]);

        cv::Point obstacle_point = cv::Point(point_grid_pos_clamped.x, point_grid_pos_clamped.y);

        cv::circle(grid, cv::Point(point_grid_pos_clamped.x, point_grid_pos_clamped.y), 2, cv::Scalar(255, 0, 0), -1);
        //cv::line(img, cv::Point(drone_pos.x, drone_pos.y), cv::Point(point_grid_pos_clamped.x, point_grid_pos_clamped.y), cv::Scalar(0));

        //cv::line(img, { center.x, center.y }, undistortedObjectPointsInt[i], cv::Scalar(0));

    }

    cv::circle(grid, cv::Point(drone_pos.x, drone_pos.y), DRONE_RADIUS, cv::Scalar(0), -1);
    cv::line(grid, cv::Point(start_pos.x, start_pos.y), cv::Point(end_pos.x, end_pos.y), cv::Scalar(128), 2);

    //prev_grid = prev_grid + grid * 0.4;

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

    bool old_data_files = true;

    //cv::Mat grid = cv::Mat(GRID_SIZE.x, GRID_SIZE.y, CV_8UC3);
    //grid.setTo(cv::Scalar(255,255,255));

    if (old_data_files) {

        // Directory path to drone images relative to src directory
        const char* drone_images_directory = "../images/run1/";
        const char* drone_data_directory = "../data/";
        const char* cache_data_directory = "../cache/";
        const char* drone_data_file = "run1.csv";

        std::vector<DroneData> drone_data = getDroneData(drone_images_directory, cache_data_directory, drone_data_directory, drone_data_file);
    
        for (auto& data : drone_data) {
            processImage(data, {}, old_data_files);//, grid);
        }
    } else {

        const char* drone_images_directory = "../images/20240324-020231/";

        auto [drone_data, obstacles] = getDroneDataNew(drone_images_directory);
    
        for (auto& data : drone_data) {
            processImage(data, obstacles, old_data_files);//, grid);
        }
    }

    
    cv::waitKey(0);

    destroyDrawingWindows();

    std::cin.get();

    return 0;
}
