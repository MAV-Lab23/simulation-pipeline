#pragma once

typedef unsigned char uchar;

#ifdef IN_PAPARAZZI

// TODO: Implement cv::Mat in paparazzi compatible code if needed.
//#include <opencv2/core/mat.hpp>
typedef uchar* Image;

#else

#include <opencv2/core/mat.hpp>
typedef cv::Mat Image;

#endif

typedef struct Vector2f {
	float x;
	float y;
} Vector2f;

typedef struct Vector3f {
	float x;
	float y;
	float z;
} Vector3f;

typedef struct Vector2i {
	int x;
	int y;
} Vector2i;

typedef struct Color {
	uchar r;
	uchar g;
	uchar b;
} Color;

typedef struct DroneState {
	Vector3f optitrack_pos; /* meters */
	// x = yaw, y = pitch, z = roll
	Vector3f optitrack_angle; /* radians */
} DroneState;

typedef struct DroneData {
	Image image;
	DroneState state;
} DroneData;
