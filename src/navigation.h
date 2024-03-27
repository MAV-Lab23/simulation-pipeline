#ifndef GROUP_10_NAVIGATION_H
#define GROUP_10_NAVIGATION_H

#include <assert.h>

#include "drone.h"
#include "constants.h"
#include "utility.h"

#ifndef IN_PAPARAZZI
#include "draw.h"
#endif

#ifndef PRINT
#define PRINT(string,...) printf("[navigation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

#define HEADING_INCREMENT 10
#define DEGREES_TOTAL 360
#define HEADING_COUNT (DEGREES_TOTAL / HEADING_INCREMENT)

const int OBSTACLE_POINT_MAX_LIFETIME = 200; // frames of periodic function

static float probabilities[GRID_LENGTH] = { 0 };
static int timers[GRID_LENGTH] = { 0 };

static float move_distance = 1.0f; // max waypoint displacement before re-evaluating [m]

static float closest_obstacle_distance = FLT_MAX; // distance to nearest obstacle [m]
static float closest_obstacle_distance_threshold = 1; // max distance to nearest obstacle before finding new heading [m]

static float heading_diff = HEADING_INCREMENT;

static void printGrid(void) {
  for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			int index = i + offset;
      PRINT("(%i, %i): probability: %.1f, timer: %i \n", i, j, probabilities[index], timers[index]);
    }
  }
}

static void printGridElement(int i, int j) {
  int index = i + j * GRID_SIZE.x;
  PRINT("(%i, %i): probability: %.1f, timer: %i \n", i, j, probabilities[index], timers[index]);
}


static void setGridProbability(int index, float probability) {
	assert(index >= 0 && index < GRID_LENGTH);
	probabilities[index] = clamp(probability, 0.0f, 1.0f);
}

static float getGridProbability(int index) {
	assert(index >= 0 && index < GRID_LENGTH);
	return probabilities[index];
}

static void setTimer(int index, int value) {
	assert(index >= 0 && index < GRID_LENGTH);
	timers[index] = clamp(value, 0, OBSTACLE_POINT_MAX_LIFETIME);
}

static int getTimer(int index) {
	assert(index >= 0 && index < GRID_LENGTH);
	return timers[index];
}

static void addGridElement(int index) {
	setGridProbability(index, 1.0);
	setTimer(index, OBSTACLE_POINT_MAX_LIFETIME);
}

static float distanceSquared(const Vector2f p1, const Vector2f p2) {
  return powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2);
}

static float customModf(float x, float y) {
  return x - floorf(x / y) * y;
}

static float normalizeHeading(float heading) {
  return customModf(heading, 360.0f);
}

static void findBestHeading(
#ifndef IN_PAPARAZZI
    cv::Mat& grid,
#endif
  const Vector2i drone_position,
  const float drone_heading /* radians */,
  const float max_distance,
  const float min_point_distance,
  Vector2i* best_endpoint,
  float* best_heading /* radians */) {

    float distance_threshold2 = min_point_distance * min_point_distance / (METERS_PER_GRID_CELL.x * METERS_PER_GRID_CELL.x);

    float smallest_heading_difference = DEGREES_TOTAL;

    // Headings which are this many grid cells from carpet edge are also skipped.
    float heading_width_padding = 10;
    float heading_height_padding = 10;
    
    Vector2i top_left;
    Vector2i bottom_right;
    getCarpetCorners(&top_left, &bottom_right);

  for (float heading = 0.0f; heading < DEGREES_TOTAL; heading += HEADING_INCREMENT) {
    float heading_rad = heading * M_PI / 180.0;

    #ifndef IN_PAPARAZZI
    //drawHeading(grid, drone_position, heading, max_distance, cv::Scalar(128, 128, 128), 1);
    #endif

    Vector2f endpoint = {
      drone_position.x + max_distance / METERS_PER_GRID_CELL.x * -cos(heading_rad),
      drone_position.y + max_distance / METERS_PER_GRID_CELL.y * -sin(heading_rad)
    };

    // Skip headings which would take the drone out of bounds (and some headings near those).
    if (endpoint.x > bottom_right.x - heading_width_padding ||
        endpoint.x < top_left.x + heading_width_padding ||
        endpoint.y > bottom_right.y - heading_height_padding ||
        endpoint.y < top_left.y + heading_height_padding) continue;

    float shortest_distance2 = FLT_MAX;

    for (int j = 0; j < GRID_SIZE.y; j++) {
        int offset = j * GRID_SIZE.x;
        for (int i = 0; i < GRID_SIZE.x; i++) {
            int index = i + GRID_SIZE.x * j;
            Vector2f point = { (float)i, (float)j };
            float probability = getGridProbability(index);
            if (probability > 0) {
                float point_distance2 = distanceSquared(endpoint, point);
                if (point_distance2 <= shortest_distance2) {
                    shortest_distance2 = point_distance2;
                }
            }
        }
    }


    if (shortest_distance2 != FLT_MAX && shortest_distance2 > distance_threshold2) {
        float norm_drone_heading = (int)radToDeg(drone_heading) % 360;
        //PRINT("drone: %.2f\n", norm_drone_heading);
        float diff = (float)((int)fabsf(norm_drone_heading - heading) % 360);
        float diff_other_way = fabsf(diff - 360);
        diff = MIN(diff, diff_other_way);
        // Draw all potentially acceptable heading endpoints.
    #ifndef IN_PAPARAZZI
        cv::circle(grid, { (int)endpoint.x, (int)endpoint.y }, 1, cv::Scalar(255, 0, 0), -1);
    #endif
        if (diff <= smallest_heading_difference) {
            smallest_heading_difference = diff;
            best_endpoint->x = (int)endpoint.x;
            best_endpoint->y = (int)endpoint.y;
            *best_heading = heading_rad;
        }
        // shortest_distance2 < best_heading_distance2
        // best_heading_distance2 = shortest_distance2;
        //*best_heading = heading;
        //float dist = sqrtf(shortest_distance2 / (GRID_SIZE.x * GRID_SIZE.x + GRID_SIZE.y * GRID_SIZE.y));
    }
  }
    //PRINT("----\n");
    if (smallest_heading_difference == DEGREES_TOTAL) {
        *best_heading = INVALID_POINT_FLT;
        best_endpoint->x = INVALID_POINT;
        best_endpoint->y = INVALID_POINT;
        // Found no headings which take drone to a point where waypoint
        // endpoint is distance_threshold away from a potential obstacle.
        // Change drone heading until one is found, if not go forward.
    } else {
        // Draw best heading endpoint.
    #ifndef IN_PAPARAZZI
        cv::circle(grid, { best_endpoint->x, best_endpoint->y }, 3, cv::Scalar(0, 0, 255), -1);
    #endif
    }
}

// Gets the best drone heading in radians.
static float getBestHeading(
#ifndef IN_PAPARAZZI
    cv::Mat& grid,
#endif
    const Vector2i drone_grid_pos, float drone_heading, Vector2i* best_endpoint) {
    float best_heading = drone_heading; // radians
    float min_point_distance = closest_obstacle_distance_threshold;

    findBestHeading(
#ifndef IN_PAPARAZZI
        grid, 
#endif
        drone_grid_pos, drone_heading, move_distance, min_point_distance, best_endpoint, &best_heading);

    //PRINT("Best endpoint found: (%i, %i), Increasing heading by: %.3f \n", best_endpoint->x, best_endpoint->y, best_heading);
    
    return best_heading;
}

static Vector2i getObjectGridPosition(float optitrack_x, float optitrack_y) {
  Vector2f opti_pos;
  opti_pos.x = optitrack_x;
  opti_pos.y = optitrack_y;

  Vector2i drone_grid_pos = optitrackCoordinateToGrid(opti_pos);
    
	// Drone outside of probabilities
	if (!validVectorInt(drone_grid_pos)) {
      Vector2f norm_pos;

      norm_pos.x = normalizeValue(optitrack_x, -ARENA_SIZE.x / 2.0f, ARENA_SIZE.x / 2.0f);
      norm_pos.y = normalizeValue(optitrack_y, -ARENA_SIZE.y / 2.0f, ARENA_SIZE.y / 2.0f);

      // Get a clamped pos as an estimate of drone position.
      drone_grid_pos.x = clamp((int)(norm_pos.x * GRID_SIZE.x), 0, GRID_SIZE.x);
      drone_grid_pos.y = clamp((int)(norm_pos.y * GRID_SIZE.y), 0, GRID_SIZE.y);
	}
  return drone_grid_pos;
}

// Update timers and grids that become empty
Vector2i updateGrid(const Vector2i drone_grid_pos, bool subtract_timers) {
	Vector2i closest_cell = { INVALID_POINT, INVALID_POINT };
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			int index = i + offset;
			int timer = getTimer(index);
      if (subtract_timers) {
        timer -= 1;
      }
			// TODO: Consider scaling probability from 1 to 0 based on (timer / OBSTACLE_POINT_MAX_LIFETIME).
			if (timer <= 0) {
				setGridProbability(index, 0);
			}
			// Timer is clamped to minimum 0 inside setTimer
			setTimer(index, timer);
      if (timer > 0) {
				float probability = getGridProbability(index);
				if (probability > 0) {
          // TODO: Figure out if this is necessary or useful.
					// float x_dist = (i - drone_grid_pos.x) * METERS_PER_GRID_CELL.x;
					// float y_dist = (j - drone_grid_pos.y) * METERS_PER_GRID_CELL.y;
					// float dist = sqrtf(x_dist * x_dist + y_dist * y_dist);
					// if (dist <= closest_obstacle_distance) {
					// 	closest_obstacle_distance = dist;
					// 	closest_cell.x = i;
					// 	closest_cell.x = j;
					// }
				}
			}
		}
	}
    return closest_cell;
}

#endif