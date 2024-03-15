#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_processing.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#define PATH_MAX 4096

#include <stdio.h>
#include <string.h>

#ifdef _WIN32  // Define for Windows systems

#include <windows.h>

// Function to get all files in a directory (Windows)
int ForEachFile(const char* directory_path, bool (*callback)(const char* filename, const char* directory)) {
    WIN32_FIND_DATA find_data;
    HANDLE handle;

    // Combine path with wildcard
    char path[MAX_PATH];
    snprintf(path, sizeof(path), "%s\\*", directory_path);

    // Find the first file
    handle = FindFirstFile(path, &find_data);
    if (handle == INVALID_HANDLE_VALUE) {
        printf("Error opening directory: %s\n", directory_path);
        return 1;
    }

    // Loop through files
    do {
        // Skip "." and ".." directories
        if (strcmp(find_data.cFileName, ".") == 0 || strcmp(find_data.cFileName, "..") == 0) {
            continue;
        }

        // Call the callback function with filename
        if (callback(find_data.cFileName, directory_path)) break;
    } while (FindNextFile(handle, &find_data));

    // Close handle
    FindClose(handle);

    return 0;
}

#else  // Define for POSIX-compliant systems (Linux, macOS, etc.)

#include <dirent.h>
#include <sys/types.h>

// Function to get all files in a directory (POSIX)
int ForEachFile(const char* directory_path, bool (*callback)(const char* filename, const char* directory)) {
    DIR* dir;
    struct dirent* entry;

    // Open directory
    dir = opendir(directory_path);
    if (dir == NULL) {
        printf("Error opening directory: %s\n", directory_path);
        return 1;
    }

    // Loop through directory entries
    while ((entry = readdir(dir)) != NULL) {
        // Skip "." and ".." directories
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Call the callback function with filename
        if (callback(entry->d_name, directory_path)) break;
    }

    // Close directory
    closedir(dir);

    return 0;
}

#endif

// Function to check if coordinates are valid within the grid boundaries
bool isValidCoordinate(int width, int height, int x, int y) {
    return (x >= 0 && x < width) && (y >= 0 && y < height);
}

// Function to modify a cell at a given x,y coordinate
void setCell(uchar* grid, int width, int height, int x, int y, unsigned char value) {
    if (isValidCoordinate(width, height, x, y)) {
        grid[y * width + x] = value;
    }
    else {
        fprintf(stderr, "Error: Invalid coordinates (%d, %d) for grid of size (%d, %d)\n", x, y, width, height);
    }
}

// Function to access the value of a cell at a given x,y coordinate
unsigned char getCell(const uchar* grid, int width, int height, int x, int y) {
    if (isValidCoordinate(width, height, x, y)) {
        return grid[y * width + x];
    }
    else {
        fprintf(stderr, "Error: Invalid coordinates (%d, %d) for grid of size (%d, %d)\n", x, y, width, height);
        return 0; // Or return a different default value if preferred
    }
}

cv::Mat createGrid(uchar* out_grid, int width, int height) {
    // Allocate memory for the grid data (1 channel, unsigned char)
    out_grid = (uchar*)malloc(width * height * sizeof(uchar));

    cv::Mat cv_grid(height, width, CV_8UC1, out_grid);
    return cv_grid;
}

void destroyGrid(uchar* in_grid) {
    // Free the allocated memory
    free(in_grid);
}

Vector2i GetObstacleGridPosition(const Vector2i& drone_cam_size,
    const Vector2i& point,
    float drone_fov_width, /* degrees */
    float drone_pitch, /* degrees */
    float drone_heading, /* degrees */
    const Vector2f& drone_pos,
    float drone_height /* meters */) {

    float aspect_ratio = drone_cam_size.y / drone_cam_size.x;
    int center_x = drone_cam_size.x / 2;
    int center_y = drone_cam_size.y / 2;

    float fov_h = drone_fov_width * aspect_ratio;

    int dist_x = point.x - center_x;
    int dist_y = point.y - center_y;

    float frac_screen_y = dist_y / drone_cam_size.y;
    float frac_screen_x = dist_x / drone_cam_size.x;

    float angle_y = frac_screen_y * fov_h;
    float angle_x = frac_screen_x * drone_fov_width;

    // anti clockwise is positive

    float line_angle = drone_pitch - angle_y;

    // TODO: Change this to better reflect height of camera.
    float height_camera = drone_height;

    float x_pos_from_drone = height_camera / tan(line_angle);
    float y_pos_from_drone = x_pos_from_drone * tan(angle_x);

    float R_x = cos(drone_heading) - sin(drone_heading);
    float R_y = sin(drone_heading) + cos(drone_heading);

    // TODO: Check that this operation is in correct order.
    float x_grid = R_x * x_pos_from_drone + drone_pos.x;
    float y_grid = R_y * y_pos_from_drone + drone_pos.y;

    return { (int)x_grid, (int)y_grid };
}

bool ProcessImage(const char* filename, const char* directory_path) {
    char filepath[PATH_MAX];
    int filename_length = strlen(filename);

    uchar* grid_pointer = NULL;

    int grid_width = 300;
    int grid_height = 300;

    cv::Mat grid = createGrid(grid_pointer, grid_width, grid_height);

    // Check for image extensions (modify as needed)
    if (filename_length >= 4 &&
        (strcmp(filename + filename_length - 4, ".jpg") == 0 ||
            strcmp(filename + filename_length - 4, ".png") == 0)) {
        // Construct full filepath
        snprintf(filepath, sizeof(filepath), "%s/%s", directory_path, filename);

        // Read image
        cv::Mat image = cv::imread(filepath);
        if (!image.data) {
            printf("Error reading image: %s\n", filepath);
            return false;
        }

        std::vector<cv::Point> objectDistances = processImageForObjects(image);
        
        cv::MatSize size = image.size;
        // OpenCV image size gives [h, w]
        cv::Point center = { size[1] / 2, size[0] / 2 };

        cv::Mat processed_image;
        image.copyTo(processed_image);

        // Draw lines from center of screen to found obstacles.
        for (size_t i = 0; i < objectDistances.size(); i++)
        {
            int thickness = 2;
            int lineType = cv::LINE_8;

            Vector2i pos = { objectDistances[i].x, objectDistances[i].y };

            //Vector2i pos = GetObstacleGridPosition({ 520, 240 }, { objectDistances[i].x, objectDistances[i].y }, 60, 0, 0, { 0, 0 }, 3);

            pos = { (int)Clamp(pos.x, 0, grid_width - 1), (int)Clamp(pos.y, 0, grid_height - 1) };

            if (grid_pointer != NULL) {
                //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
                setCell(grid_pointer, grid_width, grid_height, pos.x, pos.y, 0);
            }
            else {
                //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
                //grid.at<uchar>(pos.y, pos.x) = 0;
                int circle_radius = 5;
                cv::circle(grid, cv::Point(pos.x, pos.y), circle_radius, 0, -1);
            }

            cv::line(processed_image,
                center,
                objectDistances[i],
                cv::Scalar(0, 0, 0),
                thickness,
                lineType);
        }


        // Display image.
        cv::imshow("Image", image);
        cv::imshow("Processed", processed_image);
        cv::imshow("Grid", grid);

        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            return true;
        }
    }

    destroyGrid(grid_pointer);

    return false;
}

void InitOpenCVWindows() {
    cv::namedWindow("Image");
    cv::namedWindow("Processed");
    cv::namedWindow("Grid");

    cv::moveWindow("Image", 30, 30);
    cv::moveWindow("Processed", 30 + 720, 30);
    cv::moveWindow("Grid", 30, 30 + 300);
}

int main() {

    InitOpenCVWindows();

    // Directory path to drone images relative to src directory
    char* drone_images_directory_path = "../images/run1";

    ForEachFile(drone_images_directory_path, ProcessImage);

    cv::destroyAllWindows();

    return 0;
}