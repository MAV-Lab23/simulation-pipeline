#ifndef GROUP_10_DEBUG_H
#define GROUP_10_DEBUG_H

#include "drone.h"

static void fixDroneState(DroneState& out_state) {
    // Fix drone state to center at given inclination for testing.
    out_state.pos.x = 0;
    out_state.pos.y = 0;
    out_state.pos.z = 1;
    out_state.heading.x = 0;
    out_state.heading.y = -DEG_TO_RAD(45);
    //in_state.heading.z = 0;
}

#endif