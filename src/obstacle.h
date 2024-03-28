#ifndef GROUP_10_OBSTACLE_H
#define GROUP_10_OBSTACLE_H

#include <stdio.h>
#include <math.h>

#include "utility.h"
#include "constants.h"

static void transformObstacleToGrid(float x, float y, int* out_x, int* out_y) {
	float angle = M_PI / 2 - TRUE_NORTH_TO_CARPET_ANGLE;

    // 2D rotation.
	x = -(x * cos(angle) - y * sin(angle));
	y =   x * sin(angle) + y * cos(angle);

	x = normalize(x, -(ARENA_WIDTH) / 2.0f, (ARENA_WIDTH) / 2.0f);
	y = normalize(y, -(ARENA_HEIGHT) / 2.0f, (ARENA_HEIGHT) / 2.0f);

	*out_x = (int)(x * GRID_WIDTH);
    *out_y = (int)(y * GRID_HEIGHT);
}

#endif