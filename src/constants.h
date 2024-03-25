#ifndef GROUP_10_CONSTANTS_H
#define GROUP_10_CONSTANTS_H

#include <limits.h>
#include <stdio.h>

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

static const float TRUE_NORTH_TO_CARPET_ANGLE = 0.454001; // radians

// Image convolution

static const float CURRENT_IMAGE_WEIGHT = 0.4f; // 0 to 1

// Drone parameters

static const float DRONE_FOV_ANGLE = 140.0f; // degrees (full fov)
static const float CAMERA_TILT = 18.9; // degrees

#endif