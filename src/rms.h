#ifndef RMS_H
#define RMS_H
#include <stdio.h>



struct Coordinate {
    double coordinate_x;
    double coordinate_y;
};

double euclidean_distance(double start_x, double start_y, double destination_x, double destination_y) {
    double delta_x = pow((start_x - destination_x), 2);
    double delta_y = pow((start_y - destination_y), 2);
    return sqrt(delta_x + delta_y);
}

double CalculateRMS(double start_x, double start_y, double angle, int N, Vector2i *obstacles, double safe_distance) {
    double sum_squared_deviation = 0;
    double heading_rad = angle * (M_PI/180);
    // Calculate the transformed drone position
    double x_start_transformed = start_x * cos(heading_rad) - start_y * sin(heading_rad);
    double y_start_transformed = start_x * sin(heading_rad) + start_y * cos(heading_rad);
    for (int i = 0; i < N; i += 2) {
        double x_obstacle_transformed = obstacles[i].x * cos(heading_rad) - obstacles[i].y * sin(heading_rad);
        double y_obstacle_transformed = obstacles[i].x * sin(heading_rad) + obstacles[i].y * cos(heading_rad);
        if (y_obstacle_transformed >= y_start_transformed) {
            // Calculate the deviation in x-direction
            double deviation = fabs(x_start_transformed - x_obstacle_transformed);
            sum_squared_deviation += deviation * deviation;
            if (fabs(x_obstacle_transformed - x_start_transformed) < safe_distance) {
                sum_squared_deviation = 0;
                break;
            }
            else {continue;
            }
        }
        else {continue;
        }
    }
    
    // Calculate RMS deviation
    double rms = sqrt(sum_squared_deviation / N);
    //printf("RMS Deviation: %f\n", rms);
    return rms;
}

struct Coordinate calculateWaypoint(double start_x, double start_y, double angle_degrees, double xmax, double ymax) {
    double angle_rad = angle_degrees * M_PI / 180;
    double slope, y_intercept, endpoint_x, endpoint_y;

    if (angle_degrees >= 0 && angle_degrees < 90) { // First quadrant
        slope = tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = ymax < slope * xmax + y_intercept ? ymax : slope * xmax + y_intercept;
    } else if (angle_degrees >= 90 && angle_degrees < 180) { // Second quadrant
        angle_rad = (90 - angle_degrees) * M_PI / 180;
        slope = 1 / tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = 0 > slope * 0 + y_intercept ? 0 : slope * 0 + y_intercept;
    } else if (angle_degrees >= 180 && angle_degrees < 270) { // Third quadrant
        angle_rad = (angle_degrees - 180) * M_PI / 180;
        slope = tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = 0 > slope * 0 + y_intercept ? 0 : slope * 0 + y_intercept;
    } else if (angle_degrees >= 270 && angle_degrees < 360) { // Fourth quadrant
        angle_rad = (360 - angle_degrees) * M_PI / 180;
        slope = 1 / tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = ymax < slope * xmax + y_intercept ? ymax : slope * xmax + y_intercept;
    }

    // Calculate x-coordinate of the endpoint using the line equation
    endpoint_x = (endpoint_y - y_intercept) / slope;
    
    // Ensure endpoint_x stays within [0, xmax]
    endpoint_x = fmax(fmin(endpoint_x, xmax), 0);

    // Create and return the endpoint coordinate
    struct Coordinate endpoint = {endpoint_x, endpoint_y};
    return endpoint;
}


/*
struct Coordinate calculateWaypoint(double start_x, double start_y, double angle_degrees, double xmax, double ymax) {
    double angle_rad = angle_degrees * M_PI / 180;
    double slope, y_intercept, endpoint_x, endpoint_y;

    if (angle_degrees >= 0 && angle_degrees < 90) { // First quadrant
        slope = 1 / tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = ymax < slope * xmax + y_intercept ? ymax : slope * xmax + y_intercept;
    } else if (angle_degrees >= 90 && angle_degrees < 180) { // Second quadrant
        angle_rad = (90 - angle_degrees) * M_PI / 180;
        slope = tan(angle_rad);
        ymax = 0;
        y_intercept = start_y - slope * start_x;
        endpoint_y = ymax > slope * xmax + y_intercept ? ymax : slope * xmax + y_intercept;
    } else if (angle_degrees >= 180 && angle_degrees < 270) { // Third quadrant
        angle_rad = (angle_degrees - 180) * M_PI / 180;
        slope = 1 / tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = 0 > slope * 0 + y_intercept ? 0 : slope * 0 + y_intercept;
    } else if (angle_degrees >= 270 && angle_degrees < 360) { // Fourth quadrant
        angle_rad = (angle_degrees - 360) * M_PI / 180;
        slope = 1/tan(angle_rad);
        y_intercept = start_y - slope * start_x;
        endpoint_y = 0 > slope * 0 + y_intercept ? 0 : slope * 0 + y_intercept;
    }

    // Calculate x-coordinate of the endpoint using the line equation
    endpoint_x = (endpoint_y - y_intercept) / slope;
    
    // Ensure endpoint_x stays within [0, xmax]
    endpoint_x = fmax(fmin(endpoint_x, xmax), 0);

    // Create and return the endpoint coordinate
    struct Coordinate endpoint = {endpoint_x, endpoint_y};
    return endpoint;
}
*/
#include <cstdlib> // for realloc
//#include "vector2i.h" // assuming you have a header file for Vector2i

Vector2i* appendElement(Vector2i *arr, int *size, Vector2i element) {
    // Increment the size of the array
    (*size)++;
    
    // Reallocate memory for the increased size
    arr = (Vector2i*)realloc(arr, (*size) * sizeof(Vector2i));
    
    // Check if memory allocation was successful
    if (arr == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1); // Exit the program if memory allocation fails
    }

    // Add the new element to the end of the array
    arr[*size - 1] = element;

    return arr;
}

/*
double* appendElement(Vector2i *arr, int *size, Vector2i element) {
    // Increase the size of the array
    (*size)++;
    // Reallocate memory for the new size
    arr = realloc(arr, (*size) * sizeof(double));
    if (arr == NULL) {
        // Handle memory allocation failure
        printf("Memory allocation failed");
        exit(1);
    }
    // Append the new element at the end
    arr[(*size) - 1] = element;
    return arr;
}
*/




#endif