#include "group_10_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include "drone.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define GRID_ROWS 200
#define GRID_COLS 200
#define GRID_SIZE (GRID_ROWS * GRID_COLS)
#define DEG_TO_RAD (M_PI / 180.0)
#define SCAN_DISTANCE 30 // 1.5 meters in grid cells

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i* new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i* new_coor);
static uint8_t setNavHeading(float heading);
static float findNewHeading();

void initializeGrid(float* grid);
float pathObstacleSum(float* grid, int x0, int y0, int x1, int y1);
void findBestPath(float* grid, int startX, int startY, float maxDistance, int* bestX, int* bestY);
float checkObstaclesAhead(float* grid, int currentX, int currentY, float currentHeading);

enum navigation_state_t {
    SAFE,
    OBSTACLE_FOUND,
    SEARCH_FOR_SAFE_HEADING,
    OUT_OF_BOUNDS
};

enum navigation_state_t nav_state = SEARCH_FOR_SAFE_HEADING;

float maxDistance = 2.25; // max waypoint displacement [m]
float new_heading = 0.f;

float obstacle_x = -10000;
float obstacle_y = -10000;

int32_t obstacle_free_confidence = 0;

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID ABI_BROADCAST
#endif

static abi_event obstacle_detection_ev;

static void obstacle_detection_cb(uint8_t __attribute__((unused)) sender_id, float x, float y)
{
    obstacle_x = x;
    obstacle_y = y;

    // TODO: Add obstacle to grid.
}

void group_10_avoider_init(void)
{
    new_heading = findNewHeading();
    AbiBindMsgGROUP_10_OBSTACLE_DETECTION(GROUP_10_OBSTACLE_DETECTION_ID, &obstacle_detection_ev, obstacle_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void group_10_avoider_periodic(void)
{
    // only evaluate our state machine if we are flying
    if (!autopilot_in_flight()) {
        return;
    }

    //PRINT("Retrieved position from ABI: (x: %.3f, y: %.3f) \n", obstacle_x, obstacle_y);

    float moveDistance = maxDistance;

    DroneState drone_state = getDroneState();

    Vector2f point_norm_pos = { normalizeValue(point_optitrack_pos.x, -ARENA_SIZE.x / 2, ARENA_SIZE.x / 2),
                                      normalizeValue(point_optitrack_pos.y, -ARENA_SIZE.y / 2, ARENA_SIZE.y / 2) };

    Vector2i point_grid_pos = { (int)(point_norm_pos.x * GRID_COLS.x), (int)(point_norm_pos.y * GRID_ROWS.y) };

    Vector2i point_grid_pos_clamped = { (int)clamp(point_grid_pos.x, 0, GRID_COLS.x - 1), (int)clamp(point_grid_pos.y, 0, GRID_ROWS.y - 1) };

    float currentX = point_grid_pos_clamped.x;
    float currentY = point_grid_pos_clamped.y;

    // TODO: Check grid here to see if an obstacle is ahead / found.
    float grid[GRID_SIZE];
    initializeGrid(grid); // Add some obstacles

    // Variables to hold the results
    int bestX, bestY;

    // TODO: Alter obstacle_free_confidence based on grid output.

    float obstacle_confidence = checkObstaclesAhead(grid, currentX, currentY, drone_state.optitrack_angle.z);

    switch (nav_state) {
    case SAFE:
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
            nav_state = OUT_OF_BOUNDS;
        }
        else if (obstacle_confidence > 0.5) {
            nav_state = OBSTACLE_FOUND;
        }
        else {
            moveWaypointForward(WP_GOAL, moveDistance);
        }

        break;
    case OBSTACLE_FOUND:
        // stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);

        // TODO: Search new direction
        new_heading = findNewHeading();

        nav_state = SEARCH_FOR_SAFE_HEADING;

        break;
    case SEARCH_FOR_SAFE_HEADING:
        setNavHeading(new_heading);

        // make sure we have a couple of good readings before declaring the way safe
        if (obstacle_free_confidence >= 2) {
            nav_state = SAFE;
        }
        break;
    case OUT_OF_BOUNDS:
        setNavHeading(new_heading);
        moveWaypointForward(WP_TRAJECTORY, 1.5f);

        if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
            // add offset to head back into arena
            setNavHeading(new_heading);

            // reset safe counter
            obstacle_free_confidence = 0;

            // ensure direction is safe before continuing
            nav_state = SEARCH_FOR_SAFE_HEADING;
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
uint8_t setNavHeading(float heading)
{
    float new_heading = heading;
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
uint8_t calculateForwards(struct EnuCoor_i* new_coor, float distanceMeters)
{
    float heading = stateGetNedToBodyEulers_f()->psi;

    // Now determine where to place the waypoint you want to go to
    new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
    new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));

    //PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));

    return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i* new_coor)
{
    //PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));

    waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);

    return false;
}

// This function finds the new heading based on the best path determined.
float findNewHeading(float* grid, int currentX, int currentY, float maxDistance) {
    int bestX, bestY;
    findBestPath(grid, currentX, currentY, maxDistance, &bestX, &bestY);

    // Compute the angle (in radians) between the current position and the best position
    float deltaX = bestX - currentX;
    float deltaY = bestY - currentY;
    float heading = atan2(deltaY, deltaX); // atan2 gives the angle in radians between the x-axis and the line connecting the two points

    return heading;
}

// Initialize the grid with sample data
void initializeGrid(float* grid) {
    // Fill with probabilities initially (0.0 for free space)
    for (int i = 0; i < GRID_SIZE; i++) {
        grid[i] = 0.0; // Assume free space
    }
    // Example: Set some cells as having a probability of being an obstacle
    grid[50 + (50 * GRID_ROWS)] = 0.5; // 50% chance
    grid[100 + (100 * GRID_ROWS)] = 0.8; // 80% chance
    grid[140 + (100 * GRID_ROWS)] = 0.7; // 70% chance
    // Add more obstacles with their probabilities as needed
}

// Sum the probabilities along the path from (x0, y0) to (x1, y1)
float pathObstacleSum(float* grid, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    float sum = 0.0; // Sum of probabilities

    while (1) {
        // Add the probability of the current cell to the sum
        sum += grid[x0 + (y0 * GRID_ROWS)];
        if (x0 == x1 && y0 == y1) break; // End of line
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return sum; // Return the total probability sum
}

// Function to find the best direction from the current position
void findBestPath(float* grid, int startX, int startY, float maxDistance, int* bestX, int* bestY) {
    float bestProb = INFINITY;
    *bestX = startX; // Initialize with start position in case no better path is found
    *bestY = startY;

    for (int angle = 0; angle < 360; angle += 360 / 270) {
        float rad = angle * M_PI / 180.0;
        int endX = startX + (int)(cos(rad) * maxDistance);
        int endY = startY + (int)(sin(rad) * maxDistance);

        // Ensure endX and endY are within grid boundaries
        endX = (endX < 0) ? 0 : (endX >= GRID_COLS) ? GRID_COLS - 1 : endX;
        endY = (endY < 0) ? 0 : (endY >= GRID_ROWS) ? GRID_ROWS - 1 : endY;

        float probSum = pathObstacleSum(grid, startX, startY, endX, endY);
        if (probSum < bestProb) {
            bestProb = probSum;
            *bestX = endX;
            *bestY = endY;
        }
    }
}

// Additional function to check for obstacles within a certain angle and distance
float checkObstaclesAhead(float* grid, int currentX, int currentY, float currentHeading) {
    float totalProbability = 0.0;
    int numCellsChecked = 0;

    // Convert angles from degrees to radians
    float startAngle = currentHeading - 10 * DEG_TO_RAD;
    float endAngle = currentHeading + 10 * DEG_TO_RAD;

    // Iterate over the specified angle range
    for (float angle = startAngle; angle <= endAngle; angle += DEG_TO_RAD) {
        for (int dist = 0; dist < SCAN_DISTANCE; dist++) {
            int checkX = currentX + (int)(cos(angle) * dist);
            int checkY = currentY + (int)(sin(angle) * dist);

            // Ensure the coordinates are within grid boundaries
            if (checkX >= 0 && checkX < GRID_COLS && checkY >= 0 && checkY < GRID_ROWS) {
                totalProbability += grid[checkX + (checkY * GRID_COLS)];
                numCellsChecked++;
            }
        }
    }

    // Return the average probability per cell checked
    return (numCellsChecked > 0) ? totalProbability / numCellsChecked : 0.0;
}
