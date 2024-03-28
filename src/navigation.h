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

static float distanceSquared(const cv::Point p1, const cv::Point p2) {
  return powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2);
}

static float customModf(float x, float y) {
  return x - floorf(x / y) * y;
}

static float normalizeHeading(float heading) {
  return customModf(heading, DEGREES_TOTAL);
}

const float HEADING_INCREMENT_RAD = HEADING_INCREMENT * M_PI / 180.0;

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

  // Headings which are this many grid cells from carpet edge are also skipped.
  float heading_width_padding = 10;
  float heading_height_padding = 10;
  
  cv::Point top_left;
  cv::Point bottom_right;
  getCarpetCornerGridPoints(&top_left, &bottom_right);

  //find longest distance of consecutive valid angles
  float consecutive_current_start = 0;
  uint8_t consecutive_current_count = 0;
  uint8_t consecutive_max_count = 0;
  float consecutive_max_start = 0;
  float consecutive_max_end = 0;

  //consecutive angles at the start and end, to handl wrap around from 0 to 360 degrees
  uint8_t consecutive_start_count = 0;
  float consecutive_start_end = 0;

  //will be set to false as soon as there is the first heading that doesnt work, used for the angle wraparound
  bool consecutive_from_start = true;

  for (float heading = 0.0f; heading < DEGREES_TOTAL; heading += HEADING_INCREMENT) {    
    float heading_rad = heading * M_PI / 180.0;

    #ifndef IN_PAPARAZZI
    //drawHeading(grid, drone_position, heading, max_distance, cv::Scalar(128, 128, 128), 1);
    #endif

    cv::Point2f endpoint = {
      drone_position.x + max_distance / METERS_PER_GRID_CELL_X * -cos(heading_rad),
      drone_position.y + max_distance / METERS_PER_GRID_CELL_Y * -sin(heading_rad)
    };

    // Skip headings which would take the drone out of bounds (and some headings near those).
    if (endpoint.x > bottom_right.x - HEADING_WIDTH_PADDING ||
        endpoint.x < top_left.x + HEADING_WIDTH_PADDING ||
        endpoint.y > bottom_right.y - HEADING_WIDTH_PADDING ||
        endpoint.y < top_left.y + HEADING_WIDTH_PADDING){
      //invalid heading
      if(consecutive_current_count > consecutive_max_count){
        consecutive_max_count = consecutive_current_count;
        consecutive_max_start = consecutive_current_start;
        consecutive_max_end = heading_rad - HEADING_INCREMENT_RAD;
      }

      //is consecutive sequence from the start?
      if(consecutive_from_start){
        consecutive_start_count = consecutive_current_count;
        consecutive_start_end = heading_rad - HEADING_INCREMENT_RAD;
        consecutive_from_start = false;
      }

      consecutive_current_count = 0;
      continue;
    }

    float shortest_distance2 = FLT_MAX;

    for (int j = 0; j < GRID_HEIGHT; j++) {
      int offset = j * GRID_WIDTH;
      for (int i = 0; i < GRID_WIDTH; i++) {
          int index = i + GRID_WIDTH * j;
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

    //

    if (shortest_distance2 != FLT_MAX && shortest_distance2 > distance_threshold2) {
      float norm_drone_heading = (int)RAD_TO_DEG(drone_heading) % 360;
      // Draw all potentially acceptable heading endpoints.
      if (draw) {
          cv::circle(grid, { (int)endpoint.x, (int)endpoint.y }, 1, cv::Scalar(255, 0, 0), -1);
        }


      if(consecutive_current_count == 0){
        consecutive_current_start = heading_rad;
      }

      consecutive_current_count += 1;
    }
    else{
      //invalid heading
      if(consecutive_current_count > consecutive_max_count){
        consecutive_max_count = consecutive_current_count;
        consecutive_max_start = consecutive_current_start;
        consecutive_max_end = heading_rad - HEADING_INCREMENT_RAD;
      }

      //is consecutive sequence from the start?
      if(consecutive_from_start){
        consecutive_start_count = consecutive_current_count;
        consecutive_start_end = heading_rad - HEADING_INCREMENT_RAD;
        consecutive_from_start = false;
      }

      consecutive_current_count = 0;
    }
  }

  //to handle cases where all headings are fine
  if(consecutive_from_start){
    consecutive_max_count = consecutive_current_count;
    *best_heading = drone_heading;
      std::cout << "1" << std::endl;
  }else 
  {
    uint8_t consecutive_wraparound_count = (consecutive_current_count + consecutive_start_count);
    if(consecutive_wraparound_count > consecutive_max_count){ //start end wrap around is longest consecutive sequence
      *best_heading = (consecutive_current_start + (2 * M_PI - consecutive_current_start + consecutive_start_end)/2);
      if(*best_heading > (2 * M_PI)){
        *best_heading -= (2 * M_PI);
      }
      std::cout << "2" << std::endl;
    }else{
      *best_heading = (consecutive_max_end + consecutive_max_start)/2;
      std::cout << "3" << std::endl;
    }
  }

  best_endpoint->x = drone_position.x + max_distance / METERS_PER_GRID_CELL_X * -cos(*best_heading);
  best_endpoint->y = drone_position.y + max_distance / METERS_PER_GRID_CELL_Y * -sin(*best_heading);

  std::cout << *best_heading << " best heading" << std::endl;

  // Draw best heading endpoint.
  if (draw) {
    cv::circle(grid, { best_endpoint->x, best_endpoint->y }, 3, cv::Scalar(0, 0, 255), -1);
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