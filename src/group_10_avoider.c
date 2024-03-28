#include "group_10_avoider.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#include "opencv_wrapper.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#ifndef PRINT
#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif

float heading_diff = HEADING_INCREMENT;

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increaseNavHeading(float incrementDegrees);
static uint8_t setNavHeading(float new_heading);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  MOVING,
  IDLE
};

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID ABI_BROADCAST
#endif

static abi_event obstacle_detection_ev;

static void obstacle_detection_cb(uint8_t __attribute__((unused)) sender_id, float x, float y)
{
	// Add obstacle point to probability grid.
  addNavigationObstacle((int)x, (int)y);
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

  int turns_before_moving = HEADING_COUNT;
  static int turns = 0;

  float dist_x;
  float dist_y;
  float dist;

  static int start_pos_x = 0;
  static int start_pos_y = 0;
  int current_pos_x;
  int current_pos_y;
  float diff;
  float best_heading = getNavigationHeading();

  switch (navigation_state) {
    case IDLE:
        break;
    case MOVING:
        current_pos_x = stateGetPositionEnu_i()->x;
        current_pos_y = stateGetPositionEnu_i()->y;
        dist_x = start_pos_x - current_pos_x;
        dist_y = start_pos_y - current_pos_y;
        dist = sqrtf(dist_x * dist_y + dist_y * dist_y);
        diff = MOVE_DISTANCE - dist;
        if (diff > 0) {
          if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
              navigation_state = OUT_OF_BOUNDS;
            } else {
              moveWaypointForward(WP_TRAJECTORY, diff);
              moveWaypointForward(WP_GOAL, diff);
            }
        } else {
            navigation_state = SEARCH_FOR_SAFE_HEADING;
        }
      break;
    case SAFE:
      turns = 0;
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, MOVE_DISTANCE);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        moveWaypointForward(WP_GOAL, MOVE_DISTANCE);
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
      if (best_heading != INVALID_POINT_FLT) {
        setNavHeading(best_heading);

        start_pos_x = stateGetPositionEnu_i()->x;
        start_pos_y = stateGetPositionEnu_i()->y;

        moveWaypointForward(WP_TRAJECTORY, MOVE_DISTANCE);
        moveWaypointForward(WP_GOAL, MOVE_DISTANCE);

        navigation_state = MOVING;
      } else {
        // If no best heading founds, drone is in crowded area so turn it and try again.
        increaseNavHeading(heading_diff);
        turns++;
      }
      // If drone has turns a full 360 degrees and still has not found a safe heading, move it forward slightly and start over.
      if (turns >= turns_before_moving) {
        turns = 0;
        float retry_move_distance = 0.1; // fraction of MOVE_DISTANCE.
        moveWaypointForward(WP_TRAJECTORY, retry_move_distance * MOVE_DISTANCE);
        moveWaypointForward(WP_GOAL, retry_move_distance * MOVE_DISTANCE);
      }
      break;
    case OUT_OF_BOUNDS:
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))){
        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      } else {
        if (best_heading != INVALID_POINT_FLT) {
          setNavHeading(best_heading);
          moveWaypointForward(WP_TRAJECTORY, MOVE_DISTANCE);
        } else {
          increaseNavHeading(heading_diff);
          moveWaypointForward(WP_TRAJECTORY, 1.0f);
        }
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
uint8_t increaseNavHeading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  //PRINT("Increased heading to %f\n", DegOfRad(new_heading));
  return false;
}

uint8_t setNavHeading(float new_heading)
{
  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  //PRINT("Setting nav heading to %f\n", DegOfRad(new_heading));
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
  //PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //              stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
  //              POS_FLOAT_OF_BFP(new_coor->y));
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
    //PRINT("Set avoidance increment to: %f\n", heading_diff);
  } else {
    heading_diff = -HEADING_INCREMENT;
    //PRINT("Set avoidance increment to: %f\n", heading_diff);
  }
  return false;
}