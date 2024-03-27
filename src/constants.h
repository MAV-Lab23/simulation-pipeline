#ifndef GROUP_10_CONSTANTS_H
#define GROUP_10_CONSTANTS_H

#include <limits.h>
#include <stdio.h>
#include <float.h>

#include "types.h"

// POSITIVE ROTATION IS CLOCKWISE
// NEGATIVE ROTATION IS COUNTER - CLOCKWISE

// Grid resolution / size

static const Vector2i GRID_DIMENSIONS = { 1000, 1000 };
static const int TILE_SIZE = 5;
static const int HALF_TILE_SIZE = TILE_SIZE / 2;
static const Vector2i GRID_SIZE = { GRID_DIMENSIONS.x / TILE_SIZE, GRID_DIMENSIONS.y / TILE_SIZE };
static const int GRID_LENGTH = GRID_SIZE.x * GRID_SIZE.y;

static const Vector3f ARENA_SIZE = { 10, 10, 2 }; // meters
static const Vector2f CARPET_SIZE = { 7, 7 }; // meters

static const int INVALID_POINT = INT_MAX;
static const float INVALID_POINT_FLT = FLT_MAX;

static const float TRUE_NORTH_TO_CARPET_ANGLE = 0.454001; // radians

// Image convolution

static const float CURRENT_IMAGE_WEIGHT = 0.4f; // 0 to 1

// Camera parameters

static const Vector2f FOCAL_LENGTH = { 328.88704246, 328.23230449 };
static const Vector2f DISTORTION_CENTER = { 261.99543697, 218.7788551 };
static const float DISTORTION_COEFFS[5] = { -0.34929476, 0.16049764, -0.00051511, 0.00109877, -0.0425097 };

// MAKE SURE TO UPDATE THIS AS IT IS CURRENTLY NOT TAKEN FROM THE IMAGE!
static const Vector2f IMAGE_SIZE = { 520, 240 };

static const Vector2f CAMERA_FOV = {
    M_PI / 180.0 * 125.0,
    M_PI / 180.0 * 85.0
    //2 * atan2(IMAGE_SIZE.x, 2 * FOCAL_LENGTH.x) + M_PI / 180.0 * 28.0,
    //2 * atan2(IMAGE_SIZE.y, 2 * FOCAL_LENGTH.y) + M_PI / 180.0 * 14.0
}; // radians

static const Vector2f HALF_CAMERA_FOV = {
    CAMERA_FOV.x / 2.0f, CAMERA_FOV.y / 2.0f
}; // radians

static const float CAMERA_TILT = 18.9; // degrees

#endif