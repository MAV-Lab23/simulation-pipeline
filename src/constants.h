#ifndef GROUP_10_CONSTANTS_H
#define GROUP_10_CONSTANTS_H

#include <limits.h>
#include <stdio.h>
#include <float.h>

// IMAGE PROCESSING

// Floor isolation
// Define range of green color in HSV
// Threshold the HSV image to get only green color.
#define REAL_WORLD_GREEN_H_MIN 20
#define REAL_WORLD_GREEN_S_MIN 0
#define REAL_WORLD_GREEN_V_MIN 0
#define REAL_WORLD_GREEN_H_MAX 80
#define REAL_WORLD_GREEN_S_MAX 255
#define REAL_WORLD_GREEN_V_MAX 220

// Paparazzi camera differs from real world camera.
#define PAPARAZZI_GREEN_H_MIN 20
#define PAPARAZZI_GREEN_S_MIN 100
#define PAPARAZZI_GREEN_V_MIN 0
#define PAPARAZZI_GREEN_H_MAX 80
#define PAPARAZZI_GREEN_S_MAX 255
#define PAPARAZZI_GREEN_V_MAX 140

// Contour selection

#define CONTOUR_SELECTION_COUNT 10
#define CONTOUR_AREA_THRESHOLD 150
#define HORIZON_Y_DIST_THRESHOLD 10
#define LOOPS_BEFORE_HORIZON_RESET 5
#define CONTOUR_ABOVE_HORIZON_AREA_THRESHOLD 500

// Contour parsing

// Rejects points further than this from any hull line.
#define TOO_CLOSE_TO_ANY_HULL_LINE 8

// Hull points within these ranges of edge of screen are considered
// part of the edges of the screen (if both points are within, it is a "UNACCEPTABLE" hull line). 
#define HULL_WIDTH_THRESHOLD 5
#define HULL_HEIGHT_THRESHOLD 5

// Reject points further than this distance from the closest "ACCEPTABLE" hull line.
#define TOO_CLOSE_TO_CLOSEST_HULL_LINE 20.0f

// Rejects point further than this distance from the bottom of the screen.
// If the floor contour randomly jumps up, this will prevent that from being considered.
#define TOO_CLOSE_TO_FLOOR_LINE 5.0f

// Any points within this many pixels of another point are considered as part of one obstacle.
// This essentially depends on what the longest fully straight contour is.
// If a contour is fully straight, it will only place points at the top and bottom,
// which will make those points far apart and hence considered different obstacles bases.
#define OBSTACLE_CONTOUR_GROUPING_DISTANCE 30.0f
#define LOWEST_POINT_DISTANCE_THRESHOLD 35.0f

// NAVIGATION

#define ACCEPTABLE_HEADING_POINT_TO_OBSTACLE_DISTANCE 0.4f // meters

#define MOVE_DISTANCE 0.8f // max waypoint displacement before re-evaluating heading, meters

// DO NOT CHANGE THIS VALUE, IT WILL MESS UP HEADING SEARCH
#define DEGREES_TOTAL 360

// This you can change to alter how much the drone turns per new heading.
#define HEADING_INCREMENT 10.0f
#define HEADING_COUNT (DEGREES_TOTAL / HEADING_INCREMENT)

// Headings which are this many grid cells from carpet edge are also skipped.
#define HEADING_WIDTH_PADDING 17.0f
#define HEADING_HEIGHT_PADDING 17.0f

// Timers

#ifdef IN_PAPARAZZI
#define OBSTACLE_POINT_MAX_LIFETIME 40 // frames of avoider periodic function (4 hz)
#else
#define OBSTACLE_POINT_MAX_LIFETIME 100 // image frames (10 hz)
#endif

// MISC

#define M_PI 3.14159265358979323846

#define DEG_TO_RAD(x) ((x) * (M_PI / 180.))
#define RAD_TO_DEG(x) ((x) * (180. / M_PI))

#define INVALID_POINT INT_MAX
#define INVALID_POINT_FLT FLT_MAX

// Grid and arena / carpet parameters

#define GRID_WIDTH 300
#define GRID_HEIGHT 300
#define GRID_LENGTH (GRID_WIDTH * GRID_HEIGHT)

#define ARENA_WIDTH 7.0f // meters
#define ARENA_HEIGHT 7.0f // meters
#define CARPET_WIDTH 6.0f // meters
#define CARPET_HEIGHT 6.0f // meters

#define GROUND_HEIGHT 0.0f

#define METERS_PER_GRID_CELL_X (ARENA_WIDTH / (float)GRID_WIDTH)
#define METERS_PER_GRID_CELL_Y (ARENA_HEIGHT / (float)GRID_HEIGHT)

#define TRUE_NORTH_TO_CARPET_ANGLE 0.454001 // radians

#endif