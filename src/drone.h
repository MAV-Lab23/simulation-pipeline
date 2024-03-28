#ifndef GROUP_10_DRONE_H
#define GROUP_10_DRONE_H

#include <stdlib.h>
#include <math.h>
#include <vector>

#include "utility.h"
#include "constants.h"
#include "transform.h"

#include <opencv2/opencv.hpp>

// Optitrack state of the drone.
typedef struct DroneState {
	cv::Point3f pos; /* meters */
	// x = roll, y = pitch, z = yaw
	cv::Point3f heading; /* radians */
	// x = roll rate, y = pitch rate, z = yaw rate
	cv::Point3f optitrack_ang_rates; /* radians / sec */
} DroneState;

static DroneState optitrackToScreenState(DroneState state, CoordinateSystem coordinate_system) {
    // Translate yaw from relative to true north to relative to negative x axis.
    // In screen space: left = 0, up = 90, right = 180, down = 360.
    state.heading.z = optitrackToGridHeading(state.heading.z);
    //state.heading.y += 0.0f;//DEG_TO_RAD(4.0f);//DEG_TO_RAD(CAMERA_TILT);

    // TODO: Consider offseting position due to camera not being on the center of the drone.
    //float CAM_OFFSET = 0.05; // meters forward of the drone center.
    //state.pos.x += CAM_OFFSET * -cos(state.heading.z);
    //state.pos.y += CAM_OFFSET * -sin(state.heading.z);
    //state.pos.z += ???;

    state.pos = optitrackToGridRotation(state.pos, coordinate_system);
    return state;
}

#ifdef IN_PAPARAZZI

#include "state.h"

// Returns drone state relative to OpenCV coordinate system: (0, 0) = top left; (>0, >0) = bottom right.
static DroneState getDroneState() {

    cv::Point3f fixed_pos_v = { stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z };
    cv::Point3f optitrack_angle_v = { stateGetNedToBodyEulers_f()->phi, stateGetNedToBodyEulers_f()->theta, stateGetNedToBodyEulers_f()->psi };
    cv::Point3f optitrack_ang_rates_v = { stateGetBodyRates_f()->p, stateGetBodyRates_f()->q, stateGetBodyRates_f()->r };
    
    DroneState state;

    state.pos = fixed_pos_v;
    state.heading = optitrack_angle_v;
    state.optitrack_ang_rates = optitrack_ang_rates_v;

    state = optitrackToScreenState(state, NED);

    return state;
}

#endif

#endif