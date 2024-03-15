#include "group_10_obstacle_detector.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_1_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state_1 = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count_1 = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence_1 = 0;   // a measure of how certain we are that the way ahead if safe.

const int16_t max_trajectory_confidence_1 = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef GROUP_10_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define GROUP_10_VISUAL_DETECTION_ID to the orange filter
#error Please define GROUP_10_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count_1 = quality;
}

#ifndef GROUP_10_FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define GROUP_10_FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define GROUP_10_FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void group_10_obstacle_detector_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(GROUP_10_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(GROUP_10_FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void group_10_obstacle_detector_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state_1 = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence_1 = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_1_threshold = oag_color_count_1_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("color_count_1: %d  threshold: %d state: %d \n", color_count_1, color_count_1_threshold, navigation_state_1);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // update our safe confidence using color threshold
  if(color_count_1 < color_count_1_threshold){
    obstacle_free_confidence_1++;
  } else {
    obstacle_free_confidence_1 -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence_1
  Bound(obstacle_free_confidence_1, 0, max_trajectory_confidence_1);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence_1);

  switch (navigation_state_1){
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state_1 = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence_1 == 0){
        navigation_state_1 = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state_1 = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence_1 >= 2){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state_1 = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state_1 = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence_1 = 0;

        // ensure direction is safe before continuing
        navigation_state_1 = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}
