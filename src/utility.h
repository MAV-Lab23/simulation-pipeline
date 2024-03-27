#ifndef GROUP_10_UTILITY_H
#define GROUP_10_UTILITY_H

#include <stdio.h>

#include "constants.h"
#include "types.h"

#ifndef MAX

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#endif

#ifndef MIN

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#endif

// Normalize the value to the 0-1 range
static float normalizeValue(const float value, const float min_value, const float max_value) {
	if (min_value == max_value) {
		return 0.5;
	}

	float val = (value - min_value) / (max_value - min_value);

    //assert(val >= 0.0 && val >= 0.0);
    //assert(val <= 1.0 && val <= 1.0);

	return val;
}

void getCarpetCorners(Vector2i* out_top_left, Vector2i* out_bottom_right) {
    // Calculate carpet start and end points.
    out_top_left->x = (ARENA_SIZE.x - CARPET_SIZE.x) / (2 * ARENA_SIZE.x) * GRID_SIZE.x;
    out_top_left->y = (ARENA_SIZE.y - CARPET_SIZE.y) / (2 * ARENA_SIZE.y) * GRID_SIZE.y;
    out_bottom_right->x = out_top_left->x + CARPET_SIZE.x / ARENA_SIZE.x * GRID_SIZE.x;
    out_bottom_right->y = out_top_left->y + CARPET_SIZE.y / ARENA_SIZE.y * GRID_SIZE.y;
}

static Vector2i getObstacleGridPos(const Vector3f rotated_obstacle_pos) {
	float adjust = 2.0f;
#ifdef IN_PAPARAZZI
	adjust = 0.0f;
#endif

	Vector2f pos;
	pos.x = normalizeValue(rotated_obstacle_pos.x, -(ARENA_SIZE.x + adjust) / 2.0f, (ARENA_SIZE.x + adjust) / 2.0f);
	pos.y = normalizeValue(rotated_obstacle_pos.y, -(ARENA_SIZE.y + adjust) / 2.0f, (ARENA_SIZE.y + adjust) / 2.0f);

	Vector2i obstacle_grid_pos = { (int)(pos.x * GRID_SIZE.x), (int)(pos.y * GRID_SIZE.y) };

	return obstacle_grid_pos;
}

// Transform obstacle points to grid coordinate frame.
static Vector3f rotateObstaclePos(const Vector3f optitrack_pos) {
	float angle = M_PI / 2 - TRUE_NORTH_TO_CARPET_ANGLE;

	Vector3f pos;

	pos.x = -(optitrack_pos.x * cos(angle) - optitrack_pos.y * sin(angle));
	pos.y =   optitrack_pos.x * sin(angle) + optitrack_pos.y * cos(angle);
	// TODO: Check if this assumption is correct or if z-axis convention is changed.
	pos.z = optitrack_pos.z;
	
	return pos;
}

static bool validVectorInt(const Vector2i vector) {
	return vector.x != INVALID_POINT && vector.y != INVALID_POINT; 
}

static bool validVectorFloat(const Vector2f vector) {
	return vector.x != INVALID_POINT_FLT && vector.y != INVALID_POINT_FLT; 
}

// Returns { INVALID_POINT, INVALID_POINT } if coordinate is outside of grid.
static Vector2i optitrackCoordinateToGrid(const Vector2f opti_pos) {

	Vector2f pos;

	pos.x = normalizeValue(opti_pos.x, -ARENA_SIZE.x / 2.0f, ARENA_SIZE.x / 2.0f);
	pos.y = normalizeValue(opti_pos.y, -ARENA_SIZE.y / 2.0f, ARENA_SIZE.y / 2.0f);

	Vector2i grid_pos = { (int)(pos.x * GRID_SIZE.x), (int)(pos.y * GRID_SIZE.y) };

	if (grid_pos.x < 0 ||
		grid_pos.x > GRID_SIZE.x ||
		grid_pos.y < 0 ||
		grid_pos.y > GRID_SIZE.y) {
			
		grid_pos.x = INVALID_POINT;
		grid_pos.y = INVALID_POINT;
	}

	return grid_pos;
}

static float degToRad(float degrees) {
	return M_PI / 180.0f * degrees;
}

static float radToDeg(float radians) {
	return 180.0f / M_PI * radians;
}

static float clamp(float d, float min, float max) {
	const float t = d < min ? min : d;
	return t > max ? max : t;
}

#ifndef IN_PAPARAZZI

#include <string>
#include <fstream>
#include <filesystem>
#include <vector>
#include <utility> // std::pair

// From: https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
void writeCSV(const std::filesystem::path& filename, const std::vector<std::pair<std::string, std::vector<long double>>> dataset) {
	// Make a CSV file with one or more columns of integer values
	// Each column of data is represented by the pair <column name, column data>
	//   as std::pair<std::string, std::vector<int>>
	// The dataset is represented as a vector of these columns
	// Note that all columns should be the same size

	// Create an output filestream object
	std::ofstream myFile(filename);

	// Send column names to the stream
	for (int j = 0; j < dataset.size(); ++j)
	{
		myFile << dataset.at(j).first;
		if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
	}
	myFile << "\n";

	// Send data to the stream
	for (int i = 0; i < dataset.at(0).second.size(); ++i)
	{
		for (int j = 0; j < dataset.size(); ++j)
		{
			myFile << dataset.at(j).second.at(i);
			if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
		}
		myFile << "\n";
	}

	// Close the file
	myFile.close();
}

#endif

#endif
