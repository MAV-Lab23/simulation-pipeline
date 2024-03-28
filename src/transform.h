#ifndef GROUP_10_TRANSFORM_H
#define GROUP_10_TRANSFORM_H

#include <opencv2/opencv.hpp>

#include "constants.h"
#include "utility.h"
#include "camera.h"

typedef enum {
	NED,
	ENU
} CoordinateSystem;

// rpy = roll pitch yaw
static cv::Point3f rotate(const cv::Point3f& u, const cv::Point3f& rpy) {
    // Conversion of Euler to rotation matrix.
    // a = yaw, b = pitch, y = roll
    float y = rpy.x;
    float b = rpy.y;
    float a = rpy.z;

	float cos_yaw = cos(a);
	float cos_pitch = cos(b);
	float cos_roll = cos(y);

	float sin_yaw = sin(a);
	float sin_pitch = sin(b);
	float sin_roll = sin(y);

    // https://en.wikipedia.org/wiki/Rotation_matrix#General_3D_rotations

	float R1 = cos_yaw * cos_pitch;
	float R2 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
	float R3 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;

	float R5 = sin_yaw * cos_pitch;
	float R6 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
	float a7 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;

	float R9 = -sin_pitch;
	float R10 = cos_pitch * sin_roll;
	float R11 = cos_pitch * cos_roll;

	cv::Point3f result;

	result.x = R1 * u.x + R2  * u.y + R3  * u.z;
	result.y = R5 * u.x + R6  * u.y + a7  * u.z;
	result.z = R9 * u.x + R10 * u.y + R11 * u.z;
	
	return result;
}

static cv::Point2f cameraToOptitrackBirdsEye(
	const cv::Size& img_size,
    const cv::Point3f& pos,
    const cv::Point3f& heading,
    const cv::Point2f cam_point) {

    cv::Point2f point_norm;
	point_norm.x = normalize(cam_point.x, 0.0f, (float)img_size.width);
    point_norm.y = normalize(cam_point.y, 0.0f, (float)img_size.height);

    float longitude = (point_norm.x * 2 - 1) * HALF_CAMERA_FOV.x;
    float latitude =  (point_norm.y * 2 - 1) * HALF_CAMERA_FOV.y;
	
	float cos_lat = cos(latitude);
	float cos_lon = cos(longitude);

	cv::Point3f u;

	u.x = cos_lat * cos_lon;
	u.y = cos_lat * sin(longitude);
	u.z = cos_lon * sin(latitude);

	cv::Point3f opti_dir = rotate(u, heading);

    // Intersection parameter
    float t = -(GROUND_HEIGHT + pos.z) / opti_dir.z;

	cv::Point2f intersection = { INVALID_POINT_FLT, INVALID_POINT_FLT };

	if (t > 0) {
		return intersection;
	}

    intersection.x = pos.x + t * opti_dir.x;
    intersection.y = pos.y + t * opti_dir.y;

    return intersection;
}

static cv::Point3f rotation(cv::Point3f pos, float angle, CoordinateSystem coordinate_system) {
    if (coordinate_system == NED) {
        pos.x = pos.x * cos(angle) - pos.y * sin(angle);
        pos.y = pos.x * sin(angle) + pos.y * cos(angle);
        pos.z = -pos.z;
    } else if (coordinate_system == ENU) {
        pos.x = pos.x * sin(angle) + pos.y * cos(angle);
        pos.y = pos.x * cos(angle) - pos.y * sin(angle);
        pos.z = pos.z;
    }
    return pos;
}

static cv::Point3f gridToOptitrackRotation(cv::Point3f pos, CoordinateSystem coordinate_system) {
    pos = rotation(pos, -TRUE_NORTH_TO_CARPET_ANGLE, coordinate_system);
    return pos;
}

static cv::Point3f optitrackToGridRotation(cv::Point3f pos, CoordinateSystem coordinate_system) {
    pos = rotation(pos, TRUE_NORTH_TO_CARPET_ANGLE, coordinate_system);
    return pos;
}

static float optitrackToGridHeading(float heading /* radians */) {
    heading += TRUE_NORTH_TO_CARPET_ANGLE + M_PI;
    return heading;
}

static float gridToOptitrackHeading(float heading /* radians */) {
    heading -= TRUE_NORTH_TO_CARPET_ANGLE + M_PI;
    return heading;
}

static void getCarpetCornerGridPoints(cv::Point* out_top_left, cv::Point* out_bottom_right) {
    // Calculate carpet start and end points.
    out_top_left->x = (ARENA_WIDTH - CARPET_WIDTH) / (2 * ARENA_WIDTH) * GRID_WIDTH;
    out_top_left->y = (ARENA_HEIGHT - CARPET_HEIGHT) / (2 * ARENA_HEIGHT) * GRID_HEIGHT;
    out_bottom_right->x = out_top_left->x + CARPET_WIDTH / ARENA_WIDTH * GRID_WIDTH;
    out_bottom_right->y = out_top_left->y + CARPET_HEIGHT / ARENA_HEIGHT * GRID_HEIGHT;
}

static bool validGridPoint(const cv::Point& p) {
	return p.x != INVALID_POINT && p.y != INVALID_POINT; 
}

// Returns { INVALID_POINT, INVALID_POINT } if coordinate is outside of grid.
static cv::Point optitrack2DToGrid(cv::Point2f pos, bool clamp_pos = false) {
	pos.x = normalize(pos.x, -ARENA_WIDTH / 2.0f, ARENA_WIDTH / 2.0f);
	pos.y = normalize(pos.y, -ARENA_HEIGHT / 2.0f, ARENA_HEIGHT / 2.0f);

	cv::Point grid_pos = { (int)(pos.x * GRID_WIDTH), (int)(pos.y * GRID_HEIGHT) };

	if (clamp_pos) {
		grid_pos.x = clamp(grid_pos.x, 0, GRID_WIDTH);
		grid_pos.y = clamp(grid_pos.y, 0, GRID_HEIGHT);
	} else {
		if (grid_pos.x < 0 || grid_pos.x > GRID_WIDTH ||
			grid_pos.y < 0 || grid_pos.y > GRID_HEIGHT) {
				
			grid_pos.x = INVALID_POINT;
			grid_pos.y = INVALID_POINT;
		}
	}

	return grid_pos;
}

static cv::Point optitrack3DToGrid(const cv::Point3f& pos, bool clamp_pos = false) {
	return optitrack2DToGrid({ pos.x, pos.y }, clamp_pos);
}

#endif