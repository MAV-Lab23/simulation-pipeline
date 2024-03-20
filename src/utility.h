#pragma once

#include <stdio.h>

#include <constants.h>
#include <types.h>

#define M_PI 3.14159265358979323846

#ifndef MAX

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

#endif

#ifndef MIN

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#endif

// Normalize the value to the 0-1 range
float normalizeValue(float value, float min_value, float max_value) {
	if (min_value == max_value) {
		return 0.5;
	}

	return (value - min_value) / (max_value - min_value);
}

cv::Mat createGrid(uchar* out_grid, int width, int height) {
	// Allocate memory for the grid data (1 channel, unsigned char)
	out_grid = (uchar*)malloc(width * height * sizeof(uchar));

	cv::Mat cv_grid(height, width, CV_8UC1, out_grid);
	return cv_grid;
}

void destroyGrid(uchar* in_grid) {
	// Free the allocated memory
	free(in_grid);
}

float degToRad(float degrees) {
	return 2.0f * M_PI / 180.0f * degrees;
}

float radToDeg(float radians) {
	return 180.0f / (2.0f * M_PI) * radians;
}

/*
float floor(float value) {
	float tmp = (float)(int)value;
	return (tmp != value) ? (tmp - 1.0f) : tmp;
}
*/

// I know I could use a template here but I don't wanna add every possible condition for T.

float clamp(float d, float min, float max) {
	const float t = d < min ? min : d;
	return t > max ? max : t;
}

int flatten(const Vector2i& coordinate) {
	return GRID_SIZE.x * coordinate.y + coordinate.x;
}

Vector2i worldToGrid(const Vector2i& pos) {
	return Vector2i{
		(int)floor(clamp(pos.x, 0, GRID_DIMENSIONS.x) / TILE_SIZE),
		(int)floor(clamp(pos.y, 0, GRID_DIMENSIONS.y) / TILE_SIZE)
	};
}

/*
// This allocates memory. Clean up after yourself!
uchar* CreateEmptyGrid() {
	uchar* out_grid = (uchar*)malloc(GRID_LENGTH * sizeof(uchar));
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

void ClearGrid(uchar* out_grid) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			out_grid[i + offset] = 0;
		}
	}
}
*/

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