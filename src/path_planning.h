#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "constants.h"

#define ROW 200 //== GRID_SIZE.y
#define COL 200 //== GRID_SIZE.x

/*
ROW/COL = 15 for grid spacing 0.5 meters
ROW/COL = 71 for grid spacing 0.1 meters
*/


typedef struct {
    int parent_i, parent_j;
    double f, g, h;
} cell;

// Utility Functions
bool isValid(int row, int col);
bool isUnBlocked(int grid[][COL], int row, int col);
bool isDestination(int row, int col, int dest_row, int dest_col);
double calculateHValue(int row, int col, int dest_row, int dest_col);
void tracePath(cell cellDetails[][COL], int dest_row, int dest_col);
void aStarSearch(int grid[][COL], int start_row, int start_col, int dest_row, int dest_col);



struct Node {
    int node_y;         //row
    int node_x;         //column
};

struct Coordinate {
    double coordinate_y; //from left down moving up
    double coordinate_x; // from left down moving right
};

double node_to_y_coord(int node_row, double y_spacing) {
    return (ROW-1 -node_row) * y_spacing;
}

double node_to_x_coord(int node_col,double x_spacing) {
    return (node_col) * x_spacing;
    
}

double coordinate_to_row(double coord_y, double y_spacing) {
    return (int) (coord_y / y_spacing);
}

double coordinate_to_col(double coord_x, double x_spacing) {
    return (int) (coord_x / x_spacing);
}


bool isValid(int row, int col) {
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

bool isUnBlocked(int grid[][COL], int row, int col) {
    return grid[row][col] == 1;
}

bool isDestination(int row, int col, int dest_row, int dest_col) {
    return (row == dest_row) && (col == dest_col);
}

double calculateHValue(int row, int col, int dest_row, int dest_col) {
    return sqrt((row - dest_row) * (row - dest_row) + (col - dest_col) * (col - dest_col));
}


double* appendElement(double *arr, int *size, int element) {
    // Increase the size of the array
    (*size)++;
    // Reallocate memory for the new size
    double* new_arr = (double*)realloc(arr, (*size) * sizeof(double));
    if (new_arr == NULL) {
        // Handle memory allocation failure
        printf("Memory allocation failed");
        exit(1);
    }
    // Append the new element at the end
    arr[(*size) - 1] = element;
    return arr;
}


void tracePath(cell cellDetails[][COL], int dest_row, int dest_col) {
    printf("\nThe Path is ");
    int row = dest_row;
    int col = dest_col;

    double *arr = NULL;
    int length = 0;

    ///////
    int size = 7;
    double grid_spacing = size/(ROW-1.0);
    double row_coord = node_to_y_coord(row, grid_spacing);
    double col_coord = node_to_x_coord(col, grid_spacing);
    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
        double row_coord = node_to_y_coord(row, grid_spacing);
        double col_coord = node_to_x_coord(col, grid_spacing);
            // Append elements to the array
        arr = appendElement(arr, &length, col_coord);
            // Append elements to the array
        arr = appendElement(arr, &length, row_coord);
        printf("%f", arr[0]);
        printf("-> (%.2f,%.2f) ", col_coord, row_coord);
    }
    

}



void aStarSearch(int grid[][COL], int start_row, int start_col, int dest_row, int dest_col) {
    if (!isValid(start_row, start_col)) {
        printf("Source is invalid\n");
        return;
    }

    if (!isValid(dest_row, dest_col)) {
        printf("Destination is invalid\n");
        return;
    }

    if (!isUnBlocked(grid, start_row, start_col) || !isUnBlocked(grid, dest_row, dest_col)) {
        printf("Source or the destination is blocked\n");
        return;
    }

    if (isDestination(start_row, start_col, dest_row, dest_col)) {
        printf("We are already at the destination\n");
        return;
    }

    bool closedList[ROW][COL] = { false };
    cell cellDetails[ROW][COL];

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            cellDetails[i][j].f = INFINITY;
            cellDetails[i][j].g = INFINITY;
            cellDetails[i][j].h = INFINITY;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    int i = start_row, j = start_col;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    int openListLength = 1;

    // Direction vectors for the eight possible moves (including diagonals)
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    while (openListLength > 0) {
        double min_f = INFINITY;
        int cellWithMinF_i = -1, cellWithMinF_j = -1;

        for (int x = 0; x < ROW; x++) {
            for (int y = 0; y < COL; y++) {
                if (!closedList[x][y] && cellDetails[x][y].f < min_f) {
                    min_f = cellDetails[x][y].f;
                    cellWithMinF_i = x;
                    cellWithMinF_j = y;
                }
            }
        }

        i = cellWithMinF_i;
        j = cellWithMinF_j;
        closedList[i][j] = true;
        openListLength--;

        for (int k = 0; k < 8; k++) {
            int new_i = i + dx[k];
            int new_j = j + dy[k];

            if (isValid(new_i, new_j)) {
                if (isDestination(new_i, new_j, dest_row, dest_col)) {
                    cellDetails[new_i][new_j].parent_i = i;
                    cellDetails[new_i][new_j].parent_j = j;
                    printf("The destination cell is found\n");
                    tracePath(cellDetails, dest_row, dest_col);
                    return;
                } else if (!closedList[new_i][new_j] && isUnBlocked(grid, new_i, new_j)) {
                    double gNew = cellDetails[i][j].g + sqrt((new_i - i) * (new_i - i) + (new_j - j) * (new_j - j));
                    double hNew = calculateHValue(new_i, new_j, dest_row, dest_col);
                    double fNew = gNew + hNew;

                    if (cellDetails[new_i][new_j].f == INFINITY || cellDetails[new_i][new_j].f > fNew) {
                        openListLength++;
                        cellDetails[new_i][new_j].f = fNew;
                        cellDetails[new_i][new_j].g = gNew;
                        cellDetails[new_i][new_j].h = hNew;
                        cellDetails[new_i][new_j].parent_i = i;
                        cellDetails[new_i][new_j].parent_j = j;
                    }
                }
            }
        }
    }

    printf("Failed to find the destination cell\n");
}


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



#include <cstdlib> // for realloc
//#include "vector2i.h" // assuming you have a header file for Vector2i

Vector2i* append_Vector_Element(Vector2i *arr, int *size, Vector2i element) {
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

#endif





