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
int ForEachFile(const char* directory_path, const char* drone_data_file, bool (*callback)(const char* filename, const char* drone_data_file, const char* directory)) {
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
        if (callback(find_data.cFileName, drone_data_file, directory_path)) break;
    } while (FindNextFile(handle, &find_data));

    // Close handle
    FindClose(handle);

    return 0;
}

#else  // Define for POSIX-compliant systems (Linux, macOS, etc.)

#include <dirent.h>
#include <sys/types.h>

// Function to get all files in a directory (POSIX)
int ForEachFile(const char* directory_path, const char* drone_data_file, bool (*callback)(const char* filename, const char* drone_data_file, const char* directory)) {
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
        if (callback(entry->d_name, drone_data_file, directory_path)) break;
    }

    // Close directory
    closedir(dir);

    return 0;
}

#endif



#define ROWS_IN_CSV 3

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

Vector2i GetDronePosition(const char* filename, const char* drone_data_file, int grid_width, int grid_height) {

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

    Vector2i drone_pos_grid_clamped = { 0, 0 };

    if (data != NULL) {
        Vector2f drone_pos_opti = { strtof(data->data[1], NULL), strtof(data->data[2], NULL) }; // meters

        Vector2f arena_size = { 7, 7 }; // meters

        Vector2f drone_pos_opti_norm = { normalize_value(drone_pos_opti.x, -arena_size.x / 2, arena_size.x / 2),
                                         normalize_value(drone_pos_opti.y, -arena_size.y / 2, arena_size.y / 2) };

        Vector2i drone_pos_grid = { (int)(drone_pos_opti_norm.x * grid_width), (int)(drone_pos_opti_norm.y * grid_height) };

        drone_pos_grid_clamped = { (int)Clamp(drone_pos_grid.x, 0, grid_width - 1), (int)Clamp(drone_pos_grid.y, 0, grid_height - 1) };
        
        free_csv_row(data);
    }

    return drone_pos_grid_clamped;
}


bool ProcessImage(const char* filename, const char* drone_data_file, const char* directory_path) {
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

        //printf("Reading image: %s\n", filename);


        // Draw drone position on grid.
        Vector2i drone_pos_grid_clamped = GetDronePosition(filename, drone_data_file, grid_width, grid_height);
        cv::circle(grid, cv::Point(drone_pos_grid_clamped.x, drone_pos_grid_clamped.y), 7, 0, -1);



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

            // Vector2i image_pos = { objectDistances[i].x, objectDistances[i].y };

            Vector2i grid_pos = GetObstacleGridPosition({ 520, 240 }, { objectDistances[i].x, objectDistances[i].y }, 60, 0, 0, { 0, 0 }, 1.6);

            grid_pos = { (int)Clamp(grid_pos.x, 0, grid_width - 1), (int)Clamp(grid_pos.y, 0, grid_height - 1) };

            if (grid_pointer != NULL) {
                //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
                setCell(grid_pointer, grid_width, grid_height, grid_pos.x, grid_pos.y, 0);
            }
            else {
                //std::cout << "Adding " << pos.x << "," << pos.y << " to grid" << std::endl;
                //grid.at<uchar>(pos.y, pos.x) = 0;
                int circle_radius = 2;
                cv::circle(grid, cv::Point(grid_pos.x, grid_pos.y), circle_radius, 0, -1);
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
    const char* drone_images_directory_path = "../images/run1";
    const char* drone_data_path = "../data/run1/data.csv";

    ForEachFile(drone_images_directory_path, drone_data_path, ProcessImage);

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