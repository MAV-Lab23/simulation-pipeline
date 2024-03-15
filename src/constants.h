#pragma once

#include <types.h>

// POSITIVE ROTATION IS CLOCKWISE
// NEGATIVE ROTATION IS COUNTER - CLOCKWISE

// Grid resolution / size

constexpr const Vector2i GRID_DIMENSIONS = { 700, 700 };
constexpr const int TILE_SIZE{ 10 };
constexpr const int HALF_TILE_SIZE{ TILE_SIZE / 2 };
constexpr const Vector2i GRID_SIZE = { GRID_DIMENSIONS.x / TILE_SIZE, GRID_DIMENSIONS.y / TILE_SIZE };
constexpr const int GRID_LENGTH{ GRID_SIZE.x * GRID_SIZE.y };

constexpr const int DIVIDER_WIDTH{ 10 }; // pixels
constexpr const Vector2i WINDOW_SIZE = { GRID_SIZE.x * 2 + DIVIDER_WIDTH, GRID_SIZE.y };

// Adding obstacles to screen

// Percent by which obstacle value is added to grid in one frame(from 0 to 255).
constexpr const int VALUE_CHANGE{ int(0.5 * 255) };

// Image convolution

constexpr const float CURRENT_IMAGE_WEIGHT{ 0.4 }; // 0 to 1

// Drone parameters

constexpr const float DRONE_FOV_ANGLE{ 90.0f }; // degrees (full fov)
constexpr const float DRONE_ROTATION_SPEED{ 5.0f }; // degrees / frame
constexpr const int DRONE_FOV_DEPTH{ 400 }; // pixels
constexpr const int DRONE_SPEED{ 15 }; // pixels / frame
constexpr const int DRONE_RADIUS{ 3 }; // pixels / frame