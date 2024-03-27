#include "group_10_avoider.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#include "navigation.h"
#include "drone.h"
#include "constants.h"
#include "utility.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

float obstacle_x = -FLT_MAX;
float obstacle_y = -FLT_MAX;

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t set_nav_heading(float new_heading);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;

int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID ABI_BROADCAST
#endif

static abi_event obstacle_detection_ev;

static void obstacle_detection_cb(uint8_t __attribute__((unused)) sender_id, float x, float y)
{
	obstacle_x = x;
	obstacle_y = y;
	// Convert 2D to 1D coordinate.
	int index = obstacle_x + GRID_SIZE.x * obstacle_y;

	// Add obstacle point to probabilities.
  addGridElement(index);
  printGridElement(obstacle_x, obstacle_y);
}

void group_10_avoider_init(void) {
	// Initialise random values
	srand(time(NULL));
  chooseRandomIncrementAvoidance();
	AbiBindMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, &obstacle_detection_ev, obstacle_detection_cb);
}

void group_10_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  DroneState state = getDroneState();

  Vector2i drone_grid_pos = getDroneGridPosition(state);
  float drone_heading = state.optitrack_angle.z;
  float best_heading = drone_heading;
  Vector2i best_endpoint = drone_grid_pos;

  Vector2i closest_cell = updateGrid(drone_grid_pos);

	if (validVectorInt(closest_cell)) {
		PRINT("Closest probabilities cell: (%i, %i)\n", closest_cell.x, closest_cell.y);
	} else {
		PRINT("No close probabilities cell found\n");
	}

	PRINT("Closest obstacle distance (cells): %.3f\n", closest_obstacle_distance);

  // update our safe confidence using color threshold
  if (closest_obstacle_distance > closest_obstacle_distance_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  int turns_before_moving = HEADING_COUNT;
  static int turns = 0;

  switch (navigation_state){
    case SAFE:
      turns = 0;
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, move_distance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, move_distance);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      best_heading = getBestHeading(drone_grid_pos, drone_heading, &best_endpoint);

      if (best_heading != INVALID_POINT_FLT) {
        increase_nav_heading(best_heading);

        moveWaypointForward(WP_TRAJECTORY, move_distance);

        // make sure we have a couple of good readings before declaring the way safe
        if (obstacle_free_confidence >= 2){
          navigation_state = SAFE;
        }
      } else {
        // If no best heading founds, drone is in crowded area so turn it and try again.
        increase_nav_heading(heading_diff);
        turns++;
      }
      // If drone has turns a full 360 degrees and still has not found a safe heading, move it forward slightly and start over.
      if (turns >= turns_before_moving) {
        turns = 0;
        moveWaypointForward(WP_TRAJECTORY, 0.2 * move_distance);
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_diff);

      moveWaypointForward(WP_TRAJECTORY, move_distance);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_diff);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

uint8_t set_nav_heading(float new_heading)
{
  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  PRINT("Setting nav heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_diff = HEADING_INCREMENT;
    PRINT("Set avoidance increment to: %f\n", heading_diff);
  } else {
    heading_diff = -HEADING_INCREMENT;
    PRINT("Set avoidance increment to: %f\n", heading_diff);
  }
  return false;
}