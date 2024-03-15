#pragma once

typedef unsigned char uchar;

typedef struct Vector2f {
	float x;
	float y;
} Vector2f;

typedef struct Vector2i {
	int x;
	int y;
} Vector2i;

typedef struct Color {
	uchar r;
	uchar g;
	uchar b;
} Color;

typedef struct BoundingRect {
	Vector2i min;
	Vector2i max;
} BoundingRect;

typedef struct FOV {
	Vector2i pos;
	Vector2i right_pos;
	Vector2i left_pos;
} FOV;

typedef struct Drone {
	Vector2i pos;
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