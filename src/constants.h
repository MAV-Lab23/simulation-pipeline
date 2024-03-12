#pragma once

// POSITIVE ROTATION IS CLOCKWISE
// NEGATIVE ROTATION IS COUNTER - CLOCKWISE

// General math

#define M_PI 3.14159265358979323846

// Grid resolution / size

constexpr const int GRID_DIMENSIONS[2] = { 700, 700 };
constexpr const int TILE_SIZE{ 10 };
constexpr const int HALF_TILE_SIZE{ TILE_SIZE / 2 };
constexpr const int GRID_SIZE[2] = { GRID_DIMENSIONS[0] / TILE_SIZE, GRID_DIMENSIONS[1] / TILE_SIZE };
constexpr const int GRID_LENGTH{ GRID_SIZE[0] * GRID_SIZE[1] };

constexpr const int DIVIDER_WIDTH{ 10 }; // pixels
constexpr const int WINDOW_SIZE[2] = { GRID_SIZE[0] * 2 + DIVIDER_WIDTH, GRID_SIZE[1] };

// Adding obstacles to screen

// Percent by which obstacle value is added to grid in one frame(from 0 to 255).
constexpr const int VALUE_CHANGE{ 0.5 * 255 };

// Image convolution

constexpr const float CURRENT_IMAGE_WEIGHT{ 0.4 }; // 0 to 1

// Drone parameters

constexpr const float DRONE_FOV_ANGLE{ 90.0f }; // degrees (full fov)
constexpr const float DRONE_ROTATION_SPEED{ 5.0f }; // degrees / frame
constexpr const int DRONE_FOV_DEPTH{ 400 }; // pixels
constexpr const int DRONE_SPEED{ 15 }; // pixels / frame
constexpr const int DRONE_RADIUS{ 3 }; // pixels / frame