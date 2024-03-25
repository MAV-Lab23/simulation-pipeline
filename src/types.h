#ifndef GROUP_10_TYPES_H
#define GROUP_10_TYPES_H

typedef unsigned char uchar;

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

enum CoordinateSystem{ NED, ENU };

typedef struct Obstacle {
	Vector3f optitrack_pos;
	// x = roll, y = pitch, z = yaw
	Vector3f optitrack_angle; /* radians */
} Obstacle;

typedef struct DroneState {
	Vector3f optitrack_pos; /* meters */
	// x = roll, y = pitch, z = yaw
	Vector3f optitrack_angle; /* radians */
	// x = roll rate, y = pitch rate, z = yaw rate
	Vector3f optitrack_ang_rates; /* radians / sec */
} DroneState;

#endif