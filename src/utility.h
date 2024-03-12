#pragma once

#include <math.h>
#include <stdio.h>

#include <types.h>
#include <constants.h>

float DegToRad(float degrees) {
	return 2.0f * M_PI / 180.0f * degrees;
}

float RadToDeg(float radians) {
	return 180.0f / (2.0f * M_PI) * radians;
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
	return GRID_SIZE[0] * coordinate[1] + coordinate[0];
}

Vec2i WorldToGrid(const Vec2i& pos) {
	return Vec2i{
		(int)floor(Clamp(pos[0], 0, GRID_DIMENSIONS[0]) / TILE_SIZE),
		(int)floor(Clamp(pos[1], 0, GRID_DIMENSIONS[1]) / TILE_SIZE)
	};
}

struct BoundingRect {
	Vec2i min{};
	Vec2i max{};
};

BoundingRect GetBoundingRect(uchar* cells) {
	assert(cells != nullptr);

	int min_x, max_x = 0;
	int min_y, max_y = 0;

	for (int j = 0; j < GRID_SIZE[1]; j++)
	{
		int offset = j * GRID_SIZE[0];
		for (int i = 0; i < GRID_SIZE[0]; i++)
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