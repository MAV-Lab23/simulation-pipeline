#ifndef GROUP_10_UTILITY_H
#define GROUP_10_UTILITY_H

#include <stdio.h>

#include "constants.h"
#include "types.h"

#define M_PI 3.14159265358979323846

#ifndef MAX

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#endif

#ifndef MIN

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#endif

static bool validVector(const Vector2i vector) {
	return vector.x != INVALID_POINT && vector.y != INVALID_POINT; 
}

static bool validVector(const Vector2f vector) {
	return vector.x != INVALID_POINT_FLT && vector.y != INVALID_POINT_FLT; 
}

// Normalize the value to the 0-1 range
static float normalizeValue(float value, float min_value, float max_value) {
	if (min_value == max_value) {
		return 0.5;
	}

	float val = (value - min_value) / (max_value - min_value);

    //assert(val >= 0.0 && val >= 0.0);
    //assert(val <= 1.0 && val <= 1.0);

	return val;
}

// Returns { INVALID_POINT, INVALID_POINT } if coordinate is outside of grid.
static Vector2i optitrackCoordinateToGrid(Vector2f pos) {

	pos.x = normalizeValue(pos.x, -ARENA_SIZE.x / 2.0, ARENA_SIZE.x / 2.0);
	pos.y = normalizeValue(pos.y, -ARENA_SIZE.y / 2.0, ARENA_SIZE.y / 2.0);

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

/*
float floor(float value) {
	float tmp = (float)(int)value;
	return (tmp != value) ? (tmp - 1.0f) : tmp;
}
*/

// I know I could use a template here but I don't wanna add every possible condition for T.

static float clamp(float d, float min, float max) {
	const float t = d < min ? min : d;
	return t > max ? max : t;
}

static int flatten(const Vector2i coordinate) {
	return GRID_SIZE.x * coordinate.y + coordinate.x;
}

static Vector2i worldToGrid(const Vector2i pos) {
	Vector2i transformed_pos = {
		(int)floor(clamp(pos.x, 0, GRID_DIMENSIONS.x) / TILE_SIZE),
		(int)floor(clamp(pos.y, 0, GRID_DIMENSIONS.y) / TILE_SIZE)
	};
	return transformed_pos;
}

// TODO: Clean these up using casting

// This allocates memory. Clean up after yourself!
static float* createEmptyFloatGrid() {
	float* out_grid = (float*)malloc(GRID_LENGTH * sizeof(float));
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			out_grid[i + offset] = 0.0f;
		}
	}
	return out_grid;
}

static int* createEmptyIntGrid() {
	int* out_grid = (int*)malloc(GRID_LENGTH * sizeof(int));
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			out_grid[i + offset] = 0;
		}
	}
	return out_grid;
}

static void clearFloatGrid(float* out_grid) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			out_grid[i + offset] = 0.0f;
		}
	}
}

static void clearIntGrid(int* out_grid) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			out_grid[i + offset] = 0;
		}
	}
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
