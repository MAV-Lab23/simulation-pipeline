#ifndef GROUP_10_OBSTACLE_DETECTOR_H
#define GROUP_10_OBSTACLE_DETECTOR_H

extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_floor_count_frac;  // floor detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]

extern void group_10_obstacle_detector_init(void);
extern void group_10_obstacle_detector_periodic(void);

#endif

