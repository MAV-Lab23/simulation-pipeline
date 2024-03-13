#pragma once

#include <stdio.h>
#include <cassert>
#include <constants.h>
#include <types.h>

#define M_PI 3.14159265358979323846

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

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

float DegToRad(float degrees) {
	return 2.0f * M_PI / 180.0f * degrees;
}

float RadToDeg(float radians) {
	return 180.0f / (2.0f * M_PI) * radians;
}

float Floor(float value) {
	float tmp = (float)(int)value;
	return (tmp != value) ? (tmp - 1.0f) : tmp;
}

// I know I could use a template here but I don't wanna add every possible condition for T.

int Clamp(int d, int min, int max) {
	const int t = d < min ? min : d;
	return t > max ? max : t;
}

float Clamp(float d, float min, float max) {
	const float t = d < min ? min : d;
	return t > max ? max : t;
}

int Flatten(const Vec2i& coordinate) {
	return GRID_SIZE.x * coordinate.y + coordinate.x;
}

Vec2i WorldToGrid(const Vec2i& pos) {
	return Vec2i{
		(int)Floor(Clamp(pos.x, 0, GRID_DIMENSIONS.x) / TILE_SIZE),
		(int)Floor(Clamp(pos.y, 0, GRID_DIMENSIONS.y) / TILE_SIZE)
	};
}

BoundingRect GetBoundingRect(uchar* cells) {
	assert(cells != nullptr);

	int min_x, max_x = 0;
	int min_y, max_y = 0;

	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			int index = i + offset;
			uchar value = cells[i];
			if (value) {
				min_x = MIN(min_x, i);
				max_x = MAX(max_x, i);
				min_y = MIN(min_y, j);
				max_y = MAX(max_y, j);
			}
		}
	}

	return { { min_x, min_y }, { max_x, max_y } };
}