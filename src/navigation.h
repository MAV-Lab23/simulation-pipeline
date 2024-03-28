#ifndef GROUP_10_NAVIGATION_H
#define GROUP_10_NAVIGATION_H

#include <assert.h>
#include <math.h>

#include "drone.h"
#include "constants.h"
#include "utility.h"

#ifndef IN_PAPARAZZI
#include "draw.h"
#endif

#ifndef PRINT
#define PRINT(string,...) printf("[navigation->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

extern float probabilities[GRID_LENGTH];
extern int timers[GRID_LENGTH];

static void printGrid(void) {
  for (int j = 0; j < GRID_HEIGHT; j++)
	{
		int offset = j * GRID_WIDTH;
		for (int i = 0; i < GRID_WIDTH; i++)
		{
			int index = i + offset;
      PRINT("(%i, %i): probability: %.1f, timer: %i \n", i, j, probabilities[index], timers[index]);
    }
  }
}

static void printGridElement(int i, int j) {
  int index = i + j * GRID_WIDTH;
  PRINT("(%i, %i): probability: %.1f, timer: %i \n", i, j, probabilities[index], timers[index]);
}


static void setGridProbability(int index, float probability) {
	if (index >= 0 && index < GRID_LENGTH) {
	  probabilities[index] = clamp(probability, 0.0f, 1.0f);
  }
}

static float getGridProbability(int index) {
	if (index >= 0 && index < GRID_LENGTH) {
	  return probabilities[index];
  }
  return 0.0f;
}

static void setTimer(int index, int value) {
	if (index >= 0 && index < GRID_LENGTH) {
	  timers[index] = clamp(value, 0, OBSTACLE_POINT_MAX_LIFETIME);
  }
}

static int getTimer(int index) {
	if (index >= 0 && index < GRID_LENGTH) {
	  return timers[index];
  }
  return 0;
}

static void addGridElement(int index) {
	setGridProbability(index, 1.0);
	setTimer(index, OBSTACLE_POINT_MAX_LIFETIME);
}

static float distanceSquared(const cv::Point p1, const cv::Point p2) {
  return powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2);
}

static float customModf(float x, float y) {
  return x - floorf(x / y) * y;
}

static float normalizeHeading(float heading) {
  return customModf(heading, DEGREES_TOTAL);
}

static void findBestHeading(
    cv::Mat& grid,
  const cv::Point drone_position,
  const float drone_heading /* radians */,
  const float max_distance,
  const float min_point_distance,
  cv::Point* best_endpoint,
  float* best_heading, /* radians */
  bool draw) {

    float distance_threshold2 = min_point_distance * min_point_distance / (METERS_PER_GRID_CELL_X * METERS_PER_GRID_CELL_X);

    float smallest_heading_difference = DEGREES_TOTAL;
    
    cv::Point top_left;
    cv::Point bottom_right;
    getCarpetCornerGridPoints(&top_left, &bottom_right);

  for (float heading = 0.0f; heading < DEGREES_TOTAL; heading += HEADING_INCREMENT) {
    float heading_rad = DEG_TO_RAD(heading);

    if (draw) {
      //drawHeading(grid, drone_position, heading, max_distance, cv::Scalar(128, 128, 128), 1);
    }

    cv::Point2f endpoint = {
      drone_position.x + max_distance / METERS_PER_GRID_CELL_X * -cos(heading_rad),
      drone_position.y + max_distance / METERS_PER_GRID_CELL_Y * -sin(heading_rad)
    };

    static_assert(HEADING_WIDTH_PADDING < GRID_WIDTH);
    static_assert(HEADING_HEIGHT_PADDING < GRID_HEIGHT);

    // Skip headings which would take the drone out of bounds (and some headings near those).
    if (endpoint.x > bottom_right.x - HEADING_WIDTH_PADDING  ||
        endpoint.x < top_left.x     + HEADING_WIDTH_PADDING  ||
        endpoint.y > bottom_right.y - HEADING_HEIGHT_PADDING ||
        endpoint.y < top_left.y     + HEADING_HEIGHT_PADDING) continue;

    float shortest_distance2 = FLT_MAX;

    for (int j = 0; j < GRID_HEIGHT; j++) {
        int offset = j * GRID_WIDTH;
        for (int i = 0; i < GRID_WIDTH; i++) {
            int index = i + offset;
            cv::Point2f point = { (float)i, (float)j };
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
        float norm_drone_heading = (int)RAD_TO_DEG(drone_heading) % DEGREES_TOTAL;
        float diff = (float)((int)fabsf(norm_drone_heading - heading) % DEGREES_TOTAL);
        // Rotate in the other direction to see if any points there are closer
        float diff_other_direction = fabsf(diff - DEGREES_TOTAL);
        diff = fminf(diff, diff_other_direction);
        // Draw all potentially acceptable heading endpoints.
        if (draw) {
          cv::circle(grid, { (int)endpoint.x, (int)endpoint.y }, 1, cv::Scalar(255, 0, 0), -1);
        }
        if (diff <= smallest_heading_difference) {
            smallest_heading_difference = diff;
            best_endpoint->x = (int)endpoint.x;
            best_endpoint->y = (int)endpoint.y;
            *best_heading = heading_rad;
        }
    }
  }
    if (smallest_heading_difference == DEGREES_TOTAL) {
        *best_heading = INVALID_POINT_FLT;
        best_endpoint->x = INVALID_POINT;
        best_endpoint->y = INVALID_POINT;
        // Found no headings which take drone to a point where waypoint
        // endpoint is distance_threshold away from a potential obstacle.
        // Change drone heading until one is found, if not go forward.
    } else {
        // Draw best heading endpoint.
        if (draw) {
          cv::circle(grid, { best_endpoint->x, best_endpoint->y }, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
}

// Gets the best drone heading in radians.
static float getBestHeading(cv::Mat& grid, const cv::Point drone_grid_pos, float drone_heading, cv::Point* best_endpoint, bool draw) {
    float best_heading = drone_heading; // radians
    float min_point_distance = ACCEPTABLE_HEADING_POINT_TO_OBSTACLE_DISTANCE;

    findBestHeading(grid, drone_grid_pos, drone_heading, MOVE_DISTANCE, min_point_distance, best_endpoint, &best_heading, draw);
    return best_heading;
}

// Update timers and grids that become empty
cv::Point updateGrid(const cv::Point& drone_grid_pos, bool subtract_timers) {
	cv::Point closest_cell = { INVALID_POINT, INVALID_POINT };
	for (int j = 0; j < GRID_HEIGHT; j++)
	{
		int offset = j * GRID_WIDTH;
		for (int i = 0; i < GRID_WIDTH; i++)
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
					// float x_dist = (i - drone_grid_pos.x) * METERS_PER_GRID_CELL_X;
					// float y_dist = (j - drone_grid_pos.y) * METERS_PER_GRID_CELL_Y;
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

float updateNavigation(const DroneState& state, cv::Mat& grid, bool draw, bool subtract_timers) {
  cv::Point drone_grid_pos = optitrack3DToGrid(state.pos, true);
  cv::Point closest_cell = updateGrid(drone_grid_pos, subtract_timers);
  cv::Point best_endpoint = { 0, 0 };
  float best_heading = getBestHeading(grid, drone_grid_pos, state.heading.z, &best_endpoint, draw);
  return gridToOptitrackHeading(best_heading);
}

#endif