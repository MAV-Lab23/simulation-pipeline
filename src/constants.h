#pragma once

#include <types.h>

// POSITIVE ROTATION IS CLOCKWISE
// NEGATIVE ROTATION IS COUNTER - CLOCKWISE

// Grid resolution / size

constexpr const Vector2i GRID_DIMENSIONS = { 1000, 1000 };
constexpr const int TILE_SIZE{ 5 };
constexpr const int HALF_TILE_SIZE{ TILE_SIZE / 2 };
constexpr const Vector2i GRID_SIZE = { GRID_DIMENSIONS.x / TILE_SIZE, GRID_DIMENSIONS.y / TILE_SIZE };
constexpr const int GRID_LENGTH{ GRID_SIZE.x * GRID_SIZE.y };

constexpr const Vector3f ARENA_SIZE = { 10, 10, 7 }; // meters

// Image convolution

constexpr const float CURRENT_IMAGE_WEIGHT{ 0.4 }; // 0 to 1

// Drone parameters

constexpr const float DRONE_FOV_ANGLE{ 72.0f }; // degrees (full fov)
constexpr const int DRONE_RADIUS{ 7 }; // pixels / frame