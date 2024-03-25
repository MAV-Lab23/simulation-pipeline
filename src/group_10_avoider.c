#include "group_10_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include "drone.h"
#include "constants.h"
#include <time.h>
#include <stdio.h>
//#include <assert.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define DEG_TO_RAD (M_PI / 180.0)
#define SCAN_DISTANCE 30 // 1.5 meters in grid cells
#define MAX_SEARCH_RADIUS (GRID_SIZE.x * 2.25 / ARENA_SIZE.x)
#define PROB_THRESHOLD 0.3

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i* new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i* new_coor);
static uint8_t setNavHeading(float heading);

float findNewHeading(float maxDistance);
void initializeGrid();
float pathObstacleCheck(int x0, int y0, int x1, int y1);
float findNearestNonZero(int startX, int startY);
void findBestPath(int startX, int startY, float maxDistance, int* bestX, int* bestY);
float checkObstaclesAhead(float currentHeading);
void printGrid(float currentHeading, float newHeading);

enum navigation_state_t {
	SAFE,
	OBSTACLE_FOUND,
	SEARCH_FOR_SAFE_HEADING,
	OUT_OF_BOUNDS
};

enum navigation_state_t nav_state = SEARCH_FOR_SAFE_HEADING;

float maxDistance = 2.25; // max waypoint displacement [m]
float maxSearchDistance = 1.2; // max search distance when checking ahead for obstacles [m]
float new_heading = 0.f;
float* grid;
int* timers;
Vector2i current = { GRID_SIZE.x / 2, GRID_SIZE.y / 2 };
// TEMPORARY: FOR TESTING: HAS NOTHING TO DO WITH ACTUAL OBSTACLE COUNTS.
#define OBSTACLE_COUNT 5
#define OBJECTS_LENGTH 5

int obj_centers[OBJECTS_LENGTH][2];

float obstacle_x = -10000;
float obstacle_y = -10000;

int OBSTACLE_POINT_MAX_LIFETIME = 10000; // frames of periodic function

int32_t obstacle_free_confidence = 0;

#ifndef GROUP_10_OBSTACLE_DETECTION_ID
#define GROUP_10_OBSTACLE_DETECTION_ID ABI_BROADCAST
#endif

static void setGridProbability(int index, float probability) {
	//assert(index >= 0 && index < GRID_LENGTH);
	grid[index] = clamp(probability, 0.0f, 1.0f);
}

static float getGridProbability(int index) {
	//assert(index >= 0 && index < GRID_LENGTH);
	return grid[index];
}

static void setTimer(int index, int value) {
	//assert(index >= 0 && index < GRID_LENGTH);
	timers[index] = clamp(value, 0, OBSTACLE_POINT_MAX_LIFETIME);
}

static int getTimer(int index) {
	//assert(index >= 0 && index < GRID_LENGTH);
	return timers[index];
}

static abi_event obstacle_detection_ev;

static void obstacle_detection_cb(uint8_t __attribute__((unused)) sender_id, float x, float y)
{
	obstacle_x = x;
	obstacle_y = y;

	// Convert 2D to 1D coordinate.
	int index = obstacle_x + GRID_SIZE.x * obstacle_y;

	// Add obstacle point to grid.
	setGridProbability(index, 1.0);
	setTimer(index, OBSTACLE_POINT_MAX_LIFETIME);

	/*
	// TODO: Consider something incremental like this instead:
	int timer = getTimer(index);
	if (timer >= 0) {
		float PROBABILITY_INCREMENT = 0.1;
		float probability = getGridProbability(index);
		setGridProbability(index, probability + PROBABILITY_INCREMENT);
	}
	
	*/

	//PRINT("Retrieved position from ABI: (x: %.3f, y: %.3f) \n", obstacle_x, obstacle_y);
}

void group_10_avoider_init(void)
{
	//grid = createEmptyFloatGrid();
	//timers = createEmptyIntGrid();
	//initializeGrid(); // Add some obstacles
	new_heading = 0; //findNewHeading(maxDistance);
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

	/*
	// Update timers and grids that become empty
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			int index = i + offset;
			int timer = getTimer(index) - 1;
			// TODO: Consider scaling probability from 1 to 0 based on (timer / OBSTACLE_POINT_MAX_LIFETIME).
			if (timer <= 0) {
				setGridProbability(index, 0);
			}
			// Timer is clamped to minimum 0 inside setTimer
			setTimer(index, timer);
		}
	}
	*/

	float moveDistance = maxDistance;

	/*

	DroneState drone_state = getDroneState();

	Vector2f drone_opti_pos = { drone_state.optitrack_pos.x, drone_state.optitrack_pos.y };
	
	Vector2i grid_pos = optitrackCoordinateToGrid(drone_opti_pos);

	current.x = grid_pos.x;
	current.y = grid_pos.y;

	// TODO: Check grid here to see if an obstacle is ahead / found.

	// Variables to hold the results
	int bestX, bestY;
	*/

	// TODO: Alter obstacle_free_confidence based on grid output.

	//float obstacle_confidence = checkObstaclesAhead(drone_state.optitrack_angle.z);

	new_heading += 0.03;

	setNavHeading(new_heading);


	/*

	float obstacle_confidence = 0;

	switch (nav_state) {
	case SAFE:
		// Move waypoint forward
		moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

		if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
			nav_state = OUT_OF_BOUNDS;
		}
		else if (obstacle_confidence > PROB_THRESHOLD) {
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
		new_heading = findNewHeading(maxDistance);

		nav_state = SEARCH_FOR_SAFE_HEADING;

		break;
	case SEARCH_FOR_SAFE_HEADING:
		setNavHeading(new_heading);

		// make sure we have a couple of good readings before declaring the way safe
		if (obstacle_confidence <= 0.3) {
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
			obstacle_confidence = 0;

			// ensure direction is safe before continuing
			nav_state = SEARCH_FOR_SAFE_HEADING;
		}
		break;
	default:
		break;
	}
	return;
	*/
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
float findNewHeading(float maxDistance) {
	int bestX, bestY;
	findBestPath(current.x, current.y, maxDistance, &bestX, &bestY);

	// Compute the angle (in radians) between the current position and the best position
	int deltaX = bestX - current.x;
	int deltaY = bestY - current.y;
	float heading = atan2(deltaY, deltaX); // atan2 gives the angle in radians between the x-axis and the line connecting the two points

	return heading;
}

// Initialize the grid with sample data
void initializeGrid() {
	float obstacles[OBSTACLE_COUNT][2] = { {1.5, -2.5}, {-1.8, -3.4}, {0.6, 0.7}, {-1.8, 0.5}, {2.8, 2.5} };
	for (size_t i = 0; i < OBSTACLE_COUNT; i++)
    {
		float angle = M_PI / 2 - TRUE_NORTH_TO_CARPET_ANGLE;

		Vector2f optitrack_pos = {
			-(obstacles[i][0] * cos(angle) - obstacles[i][1] * sin(angle)),
			  obstacles[i][0] * sin(angle) + obstacles[i][1] * cos(angle),
		};

        Vector2i obstacle_grid_pos = optitrackCoordinateToGrid(optitrack_pos);

        if (!validVector(obstacle_grid_pos)) continue;

		obj_centers[i][0] = obstacle_grid_pos.x;
		obj_centers[i][1] = obstacle_grid_pos.y;
    }

	// Define radius and max probability for the obstacles
	int radius = 5;
	float radius2 = radius * radius;
	float maxProbability = 1.0f;

	// Create circular obstacles with decreasing probabilities
	for (int k = 0; k < OBJECTS_LENGTH; k++) {
		for (int y = obj_centers[k][1] - radius; y <= obj_centers[k][1] + radius; y++) {
			for (int x = obj_centers[k][0] - radius; x <= obj_centers[k][0] + radius; x++) {
				// Check if inside the grid
				if (x >= 0 && x < GRID_SIZE.x && y >= 0 && y < GRID_SIZE.y) {
					// Calculate distance from the center of the obstacle
					float distance2 = (x - obj_centers[k][0]) * (x - obj_centers[k][0]) + (y - obj_centers[k][1]) * (y - obj_centers[k][1]);
					if (distance2 <= radius2) {
						// Set probability based on distance from center (linearly decreasing)
						grid[x + y * GRID_SIZE.x] = maxProbability * (1 - sqrt(distance2 / radius2));
					}
				}
			}
		}
	}
}

// Check for obstacles along a path and return distance closest non-zero cell
float pathObstacleCheck(int x0, int y0, int x1, int y1) {
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy, e2;
	float min_dist = INFINITY;

	while (1) {
		if (grid[x0 + (y0 * GRID_SIZE.x)] >= PROB_THRESHOLD) break;
		if (x0 == x1 && y0 == y1) break; // End of line
		float dist_to_nonzero = findNearestNonZero(x0, y0);
		if (dist_to_nonzero < min_dist) { min_dist = dist_to_nonzero; }
		e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
	return min_dist; // Return the distance to closest non-zero object
}

// Function to find the distance to the nearest non-zero value in the grid from a specific point
float findNearestNonZero(int startX, int startY) {
	for (int radius = 1; radius <= MAX_SEARCH_RADIUS; radius++) {
		for (int dy = -radius; dy <= radius; dy++) {
			for (int dx = -radius; (abs(dx) + abs(dy)) <= radius; dx++) {
				int x = startX + dx;
				int y = startY + dy;
				// Check if (x, y) is within grid boundaries
				if (x >= 0 && x < GRID_SIZE.x && y >= 0 && y < GRID_SIZE.y) {
					// Check if the grid cell is non-zero
					if (grid[x + y * GRID_SIZE.x] > 0) {
						// Return the distance to this cell
						return sqrt(pow(x - startX, 2) + pow(y - startY, 2));
					}
				}
			}
		}
	}
	return -1; // Return -1 if no non-zero value is found within the search radius
}

// Function to find the best direction from the current position
void findBestPath(int startX, int startY, float maxDistance, int* bestX, int* bestY) {
	*bestX = startX; // Initialize with start position in case no better path is found
	*bestY = startY;
	float max_min_dist = 0.f;

	for (int angle = 0; angle < 360; angle += 360 / 270) {
		float rad = angle * M_PI / 180.0;
		int endX = startX + (int)(cos(rad) * maxDistance);
		int endY = startY + (int)(sin(rad) * maxDistance);

		// Ensure endX and endY are within grid boundaries
		endX = (endX < 0) ? 0 : (endX >= GRID_SIZE.x) ? GRID_SIZE.x - 1 : endX;
		endY = (endY < 0) ? 0 : (endY >= GRID_SIZE.y) ? GRID_SIZE.y - 1 : endY;

		float min_dist = pathObstacleCheck(startX, startY, endX, endY);

		if (min_dist > max_min_dist) {
			*bestX = endX;
			*bestY = endY;
		}
	}
}

// Additional function to check for obstacles within a certain angle and distance
float checkObstaclesAhead(float currentHeading) {
	float closestDistance = INFINITY;

	// Convert the heading and maxSearchDistance to a line equation of the form ax + by + c = 0
	// Line is baseed on drone's current position and heading
	float dx = cos(currentHeading);
	float dy = sin(currentHeading);

	// Line parameters a, b, c for the line equaation derived from  y = mx + n
	float a = -dy;
	float b  = dx;
	float c = dy * current.x - dx * current.y;

	// Iterate over object centres in grid coordinates and check if they are within maxSearchDistance
	for (int i = 0; i < OBJECTS_LENGTH; i++) {
		int obj0 = obj_centers[i][0];
		int obj1 = obj_centers[i][1];
		float distance2 = pow((a * obj0 + b * obj1 + c), 2) /  (a * a + b * b);
		if (distance2 <= maxSearchDistance * maxDistance) {
			/// Compute the angle from the drone's heading to the object
			float deltaX = obj0 - current.x;
			float deltaY = obj1 - current.y;
			float objectAngle = atan2(deltaY, deltaX) - currentHeading;

			//  Check if the angle is withhhin the specified limit
			if (fabs(objectAngle) <= 10 * DEG_TO_RAD) {
				closestDistance = fmin(closestDistance, distance2);
			}
		}
	}
	return closestDistance;
}


void printGrid(float currentHeading, float newHeading) {
	printf("Grid (with current position and headings):\n");
	for (int y = 0; y < GRID_SIZE.y; y++) {
		for (int x = 0; x < GRID_SIZE.x; x++) {
			// Mark the current position with 'C', the path of current heading with '-', and the path of new heading with '+'
			if (x == current.x && y == current.y) {
				printf("C ");
			}
			else if (grid[x + y * GRID_SIZE.x] > 0) { // Assume any non-zero value indicates an obstacle
				printf("%.1f ", grid[x + y * GRID_SIZE.x]); // Mark obstacles
			}
			else {
				printf(". "); // Mark free space
			}
		}
		printf("\n");
	}
	printf("Current Position: (%d, %d)\n", current.x, current.y);
	printf("Current Heading: %.2f radians\n", currentHeading);
	printf("New Heading: %.2f radians\n", newHeading);
}