#pragma once

#include "types.h"

// POSITIVE ROTATION IS CLOCKWISE
// NEGATIVE ROTATION IS COUNTER - CLOCKWISE

// Grid resolution / size

static const Vector2i GRID_DIMENSIONS = { 1000, 1000 };
static const int TILE_SIZE = 5;
static const int HALF_TILE_SIZE = TILE_SIZE / 2;
static const Vector2i GRID_SIZE = { GRID_DIMENSIONS.x / TILE_SIZE, GRID_DIMENSIONS.y / TILE_SIZE };
static const int GRID_LENGTH = GRID_SIZE.x * GRID_SIZE.y;

static const Vector3f ARENA_SIZE = { 10, 10, 7 }; // meters

// Image convolution

static const float CURRENT_IMAGE_WEIGHT = 0.4; // 0 to 1

// Drone parameters

static const float DRONE_FOV_ANGLE = 72.0f; // degrees (full fov)
static const int DRONE_RADIUS = 7; // pixels / frame
