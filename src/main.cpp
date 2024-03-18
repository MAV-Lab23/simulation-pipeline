#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_processing.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>

// SIM
#include <string>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

#define PATH_MAX 4096

#include <stdio.h>
#include <string.h>

#define ROWS_IN_CSV 10

typedef struct {
    char* data[ROWS_IN_CSV];  // Adjust this based on the number of columns in your CSV
    int num_columns;
} CSVRow;

#define MAX_LINE_LENGTH 1024

CSVRow* read_csv_row(FILE* fp) {
    char line[MAX_LINE_LENGTH];
    CSVRow* row = (CSVRow*)malloc(sizeof(CSVRow));
    row->num_columns = 0;

    if (fgets(line, MAX_LINE_LENGTH, fp) == NULL) {
        return NULL;
    }

    char* token = strtok(line, ",");
    while (token != NULL && row->num_columns < ROWS_IN_CSV) {
        row->data[row->num_columns++] = strdup(token);
        token = strtok(NULL, ",");

    }

    return row;
}

void free_csv_row(CSVRow* row) {
    if (row != NULL) {
        for (int i = 0; i < row->num_columns; ++i) {
            free(row->data[i]);
        }
        free(row);
    }
}

int skip_rows(FILE* fp, int num_rows) {
    char buffer[MAX_LINE_LENGTH];
    for (int i = 0; i < num_rows; ++i) {
        if (fgets(buffer, MAX_LINE_LENGTH, fp) == NULL) {
            return -1;  // Handle cases where the file has fewer rows than expected
        }
    }
    return 0;
}

CSVRow* get_data_from_closest_time(const char* filename, const char* csv_file, const char* target_column) {
    FILE* csv_fp = fopen(csv_file, "r");
    if (csv_fp == NULL) {
        return NULL;
    }

    // Extract timestamp from filename (assuming microsecond format)
    long double timestamp = atol(filename) / 1000000.0;

    CSVRow* closest_row = NULL;
    long double closest_diff = INFINITY;
    bool last_row_was_closest = true;

    CSVRow* row;
    int target_col_index = 0;
    static int closest_row_index = 0;
    int row_index = closest_row_index;

    if (skip_rows(csv_fp, closest_row_index) != 0) {
        printf("Error while skipping rows.\n");
        fclose(csv_fp);
        return closest_row;
    }

    int row_checks_before_found = 1;

    while ((row = read_csv_row(csv_fp)) != NULL) {
        //printf("Checking row: %i\n", row_index);
        long double time_value;
        char* endptr;
        time_value = strtold(row->data[target_col_index], &endptr);//, 10);
        if (*endptr != '\0') {
            // Non-numeric value in target column, skip this row
            free_csv_row(row);
            continue;
        }

        long double diff = abs(time_value - timestamp);
        if (diff < closest_diff) {
            last_row_was_closest = true;
            closest_diff = diff;
            free_csv_row(closest_row);  // Free previous closest row (if any)
            closest_row_index = row_index;
            closest_row = row;
        }
        else {
            free_csv_row(row);  // Free non-matching rows
            // If the next row is further away, the shortest distance has been passed.
            if (last_row_was_closest) {
                fclose(csv_fp);
                //printf("Row checks before found row: %i\n", row_checks_before_found);
                return closest_row;
            }
            last_row_was_closest = false;
        }
        row_index += 1;
        row_checks_before_found += 1;
    }

    fclose(csv_fp);
    //printf("Row checks before found row: %i\n", row_checks_before_found);
    return closest_row;
}

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

Vector2f GetObstacleGridPosition(const Vector2i& drone_cam_size,
    const Vector2i& point,
    float drone_fov_width, /* radians */
    float drone_pitch, /* radians */
    float drone_heading, /* radians */
    float drone_height /* meters */) {

    float aspect_ratio = drone_cam_size.y / drone_cam_size.x;
    int center_x = drone_cam_size.x / 2;
    int center_y = drone_cam_size.y / 2;

    float fov_h = drone_fov_width * aspect_ratio;

    int dist_x = point.x - center_x;
    int dist_y = point.y - center_y;

    float frac_screen_x = (float)dist_x / (float)drone_cam_size.x;
    float frac_screen_y = (float)dist_y / (float)drone_cam_size.y;

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
    float x_grid = R_x * x_pos_from_drone;
    float y_grid = R_y * y_pos_from_drone;

    return { x_grid, y_grid };
}

// Normalize the value to the 0-1 range
float normalize_value(float value, float min_value, float max_value) {
    if (min_value == max_value) {
        return 0.5;
    }

    return (value - min_value) / (max_value - min_value);
}

// Normalize the value to the 0-1 range
double normalize_value(double value, double min_value, double max_value) {
    if (min_value == max_value) {
        return 0.5;
    }

    return (value - min_value) / (max_value - min_value);
}

typedef struct DroneState {
    Vector2f opti_pos;
    Vector2i pos;
    float height;
    float heading;
    float pitch;
} DroneState;

DroneState GetDroneState(const char* filename, const char* drone_data_file, int grid_width, int grid_height, const Vector2f& arena_size, float arena_height) {

    const char* target_column = "time";

    CSVRow* data = get_data_from_closest_time(filename, drone_data_file, target_column);

    if (data != NULL) {
        //printf("Data from closest time row:\n");
        //for (int i = 0; i < data->num_columns; ++i) {
        //    printf("%s ", data->data[i]);
        //}
        //printf("\n");
        //free_csv_row(data);
    }
    else {
        //printf("No close match found in CSV data.\n");
    }

    Vector2f drone_pos_opti = { 0, 0 };
    Vector2i drone_pos_grid_clamped = { 0, 0 };
    float drone_heading = 0.0f;
    float drone_pitch = 0.0f;
    float drone_height = 0.0f;

    if (data != NULL) {
        drone_pos_opti = { strtof(data->data[1], NULL), strtof(data->data[2], NULL) }; // meters

        drone_heading = strtof(data->data[9], NULL);
        drone_pitch = strtof(data->data[8], NULL);

        // Convert drone height to positive only values (meters). 
        drone_height = arena_height * normalize_value(strtof(data->data[3], NULL), -arena_height / 2, arena_height / 2);

        Vector2f drone_pos_opti_norm = { normalize_value(drone_pos_opti.x, -arena_size.x / 2, arena_size.x / 2),
                                         normalize_value(drone_pos_opti.y, -arena_size.y / 2, arena_size.y / 2) };

        Vector2i drone_pos_grid = { (int)(drone_pos_opti_norm.x * grid_width), (int)(drone_pos_opti_norm.y * grid_height) };

        drone_pos_grid_clamped = { (int)Clamp(drone_pos_grid.x, 0, grid_width - 1), (int)Clamp(drone_pos_grid.y, 0, grid_height - 1) };
        
        free_csv_row(data);
    }

    return { drone_pos_opti, drone_pos_grid_clamped, drone_height, drone_heading, drone_pitch };
}


bool ProcessImage(const char* filepath, const char* filename, const char* drone_data_file, const char* directory_path) {
    uchar* grid_pointer = NULL;

    int grid_width = 300;
    int grid_height = 300;

    Vector2f arena_size = { 10, 10 }; // meters
    float arena_height = 7.0f; // meters

    cv::Mat grid = createGrid(grid_pointer, grid_width, grid_height);

    // Check for image extensions (modify as needed)
    cv::Mat image = cv::imread(filepath);
    if (!image.data) {
        printf("Error reading image: %s\n", filepath);
        return false;
    }

    //printf("Reading image: %s\n", filename);


    // Draw drone position on grid.
    DroneState drone_state = GetDroneState(filename, drone_data_file, grid_width, grid_height, arena_size, arena_height);
    cv::circle(grid, cv::Point(drone_state.pos.x, drone_state.pos.y), 7, 0, -1);

    //printf("Drone heading: %f\n", drone_state.heading);

    int thickness = 2;
    int lineType = cv::LINE_8;

    int dir_mag = 30;

    int x_dir = dir_mag * cos(drone_state.heading);
    int y_dir = dir_mag * sin(drone_state.heading);

    Vector2i start_pos = Vector2i{ (int)Clamp(drone_state.pos.x, 0, grid_width), (int)Clamp(drone_state.pos.y, 0, grid_height) };
    Vector2i end_pos = Vector2i{ (int)Clamp(drone_state.pos.x + x_dir, 0, grid_width), (int)Clamp(drone_state.pos.y + y_dir, 0, grid_height) };
    cv::line(grid,
        cv::Point(start_pos.x, start_pos.y),
        cv::Point(end_pos.x, end_pos.y),
        cv::Scalar(0, 0, 0),
        thickness,
        lineType);
        

    std::vector<cv::Point> objectDistances = processImageForObjects(image);
        
    cv::MatSize size = image.size;
    // OpenCV image size gives [h, w]
    cv::Point center = { size[1] / 2, size[0] / 2 };

    cv::Mat processed_image;
    image.copyTo(processed_image);

    // Draw lines from center of screen to found obstacles.
    for (size_t i = 0; i < objectDistances.size(); i++)
    {

        Vector2i pos = { objectDistances[i].x, objectDistances[i].y };

        Vector2f grid_offset_pos = GetObstacleGridPosition({ 520, 240 }, { objectDistances[i].x, objectDistances[i].y }, DegToRad(72.0f), drone_state.pitch, drone_state.heading, drone_state.height);

        Vector2f point_opti_pos = { normalize_value(drone_state.opti_pos.x + grid_offset_pos.x, -arena_size.x / 2, arena_size.x / 2),
                                    normalize_value(drone_state.opti_pos.y + grid_offset_pos.y, -arena_size.y / 2, arena_size.y / 2) };

        Vector2i point_pos_grid = { (int)(point_opti_pos.x * grid_width), (int)(point_opti_pos.y * grid_height) };

        Vector2i point_pos_grid_clamped = { (int)Clamp(point_pos_grid.x, 0, grid_width - 1), (int)Clamp(point_pos_grid.y, 0, grid_height - 1) };


        //printf("grid_offset_pos: %f", grid_offset_pos.x);
        //printf(",%f\n", grid_offset_pos.y);

        pos = { (int)Clamp(pos.x, 0, grid_width - 1), (int)Clamp(pos.y, 0, grid_height - 1) };
        //grid_offset_pos = { (int)Clamp(grid_offset_pos.x, 0, grid_width - 1), (int)Clamp(grid_offset_pos.y, 0, grid_height - 1) };

        if (grid_pointer != NULL) {
            //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
            setCell(grid_pointer, grid_width, grid_height, pos.x, pos.y, 0);
        }
        else {
            //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
            //grid.at<uchar>(pos.y, pos.x) = 0;
            int circle_radius = 2;
            //cv::circle(grid, cv::Point(pos.x, pos.y), circle_radius, 0, -1);
            cv::circle(grid, cv::Point(point_pos_grid_clamped.x, point_pos_grid_clamped.y), circle_radius, 0, -1);
        }

        cv::line(processed_image,
            { center.x, size[1] },
            objectDistances[i],
            cv::Scalar(0, 0, 0),
            thickness,
            lineType);
    }


    // Display image.
    cv::imshow("Image", image);
    cv::imshow("Final", processed_image);
    cv::imshow("Grid", grid);

    // Pause before going to next frame.
    cv::waitKey(0);

    if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
    {
        return true;
    }

    destroyGrid(grid_pointer);

    return false;
}

void InitOpenCVWindows() {
    cv::namedWindow("Image");
    cv::namedWindow("Final");
    cv::namedWindow("Floor");
    cv::namedWindow("Filtered");
    cv::namedWindow("Grid");
    cv::namedWindow("Intermediate1");
    cv::namedWindow("Intermediate2");
    cv::namedWindow("Intermediate3");

    cv::moveWindow("Image",           0,           0);
    cv::moveWindow("Final",           0,         275);
    cv::moveWindow("Floor",         530,           0);
    cv::moveWindow("Filtered",      530,         275);
    cv::moveWindow("Grid",          530 * 2,       0);
    cv::moveWindow("Intermediate1", 0,       275 * 2 + 9);
    cv::moveWindow("Intermediate2", 530,     275 * 2 + 9);
    cv::moveWindow("Intermediate3", 530 * 2, 275 * 2 + 9);
}

int main() {

    InitOpenCVWindows();

    // Directory path to drone images relative to src directory
    std::string drone_images_directory_path = "../images/run1";
    std::string drone_data_path = "../data/run1/data.csv";

    std::vector<fs::path> images;

    for (const auto& entry : fs::directory_iterator(drone_images_directory_path))
        images.push_back(entry.path());

    std::sort(images.begin(), images.end(), [](const auto& path1, const auto& path2) {
        return std::stoi(path1.filename().string()) < std::stoi(path2.filename().string());
    });

    for (const auto& path : images)
        ProcessImage(path.string().c_str(), path.filename().string().c_str(), drone_data_path.c_str(), drone_images_directory_path.c_str());

    cv::destroyAllWindows();

    return 0;
}

//int main() {

    //InitOpenCVWindows();

    // Directory path to drone images relative to src directory
    //char* drone_images_directory_path = "../images/run1";

    //ForEachFile(drone_images_directory_path, ProcessImage);

    //cv::destroyAllWindows();

    //return 0;
//}