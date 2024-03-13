#pragma once

using uchar = unsigned char;

typedef struct Vec2i {
	int x;
	int y;
} Vec2i;

typedef struct Color {
	uchar r;
	uchar g;
	uchar b;
} Color;

typedef struct BoundingRect {
	Vec2i min;
	Vec2i max;
} BoundingRect;

typedef struct FOV {
	Vec2i pos;
	Vec2i right_pos;
	Vec2i left_pos;
} FOV;

typedef struct Drone {
	Vec2i pos;
	float angle;
	float r_angle;
	float fov_half_angle;

	uchar* fov_cells;
	uchar* drone_cells;
} Drone;

typedef struct Simulation {
	uchar* values;
	uchar* stored_values;
	Drone drone;
} Simulation;