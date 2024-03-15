#pragma once

#include <stdlib.h>
#include <shape.h>
#include <colors.h>
#include <types.h>
#include <math.h>

void DroneInit(Drone* self, Vector2i starting_pos, float starting_angle /*degrees*/) {
	self->pos = starting_pos;
	self->angle = starting_angle;
	self->r_angle = DegToRad(starting_angle);
	assert(DRONE_FOV_ANGLE <= 90); // "Angles above 90 do not follow the formula" 
	self->fov_half_angle = DegToRad(DRONE_FOV_ANGLE / 2);
	self->fov_cells = CreateEmptyGrid();
	self->drone_cells = CreateEmptyGrid();
}

void DroneRotate(Drone* self, float angle_diff) {
	// Modulo for negative numbers
	self->angle = (((int)(self->angle + angle_diff) % 360) + 360) % 360;
	self->r_angle = DegToRad(self->angle);
}

// dir = 1->forward, dir = -1->backward
void DroneMove(Drone* self, float distance, int dir) {
	float x_movement = dir * distance * cos(self->r_angle);
	float y_movement = dir * distance * sin(self->r_angle);
	self->pos = Vector2i{
		Clamp(self->pos.x + (int)x_movement, 0, GRID_DIMENSIONS.x),
		Clamp(self->pos.y + (int)y_movement, 0, GRID_DIMENSIONS.y)
	};
}

// Returns 3 corners of fov : drone pos, right edge corner, left edge corner
FOV DroneGetFOV(Drone* self) {
	int right_edge_x = DRONE_FOV_DEPTH * cos(self->r_angle + self->fov_half_angle);
	int right_edge_y = DRONE_FOV_DEPTH * sin(self->r_angle + self->fov_half_angle);
	int left_edge_x = DRONE_FOV_DEPTH * cos(self->r_angle - self->fov_half_angle);
	int left_edge_y = DRONE_FOV_DEPTH * sin(self->r_angle - self->fov_half_angle);
	Vector2i right_pos = Vector2i{ Clamp(self->pos.x + right_edge_x, 0, GRID_DIMENSIONS.x), Clamp(self->pos.y + right_edge_y, 0, GRID_DIMENSIONS.y) };
	Vector2i left_pos = Vector2i{ Clamp(self->pos.x + left_edge_x, 0, GRID_DIMENSIONS.x), Clamp(self->pos.y + left_edge_y, 0, GRID_DIMENSIONS.y) };
	Vector2i pos = Vector2i{ Clamp(self->pos.x, 0, GRID_DIMENSIONS.x), Clamp(pos.y, 0, GRID_DIMENSIONS.y) };
	return { pos, right_pos, left_pos };
}

bool DroneObjectInFOV(Drone* self, const Vector2i& point) {
	FOV fov = DroneGetFOV(self);

	Vector2i v1 = Vector2i{ point.x - fov.pos.x, point.y - fov.pos.y };
	Vector2i v2 = Vector2i{ point.x - fov.left_pos.x, point.y - fov.left_pos.y };
	Vector2i v3 = Vector2i{ point.x - fov.right_pos.x, point.y - fov.right_pos.y };

	// Calculate the cross products
	int c1 = v1.x * v2.y - v1.y * v2.x;
	int c2 = v2.x * v3.y - v2.y * v3.x;
	int c3 = v3.x * v1.y - v3.y * v1.x;

	// Check if all cross products have the same sign(inside or outside the triangle)
	return (c1 > 0 && c2 > 0 && c3 > 0) || (c1 < 0 && c2 < 0 && c3 < 0);
}

void DroneResetCellsInFOV(Drone* self) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			int index = i + offset;
			Vector2i coord{ i, i };
			if (DroneObjectInFOV(self, coord)) {
				self->fov_cells[index] = 1;
			} else {
				self->fov_cells[index] = 0;
			}
		}
	}
}

void DroneUpdate(Drone* self) {
	SetCircleCells(self->drone_cells, self->pos, DRONE_RADIUS);
	DroneResetCellsInFOV(self);
}
