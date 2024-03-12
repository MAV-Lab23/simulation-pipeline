#pragma once

#include <types.h>
#include <shape.h>
#include <colors.h>

struct FOV {
	Vec2i pos;
	Vec2i right_pos;
	Vec2i left_pos;
};

class Drone {
public:
	Drone(Vec2i starting_pos, float starting_angle) : pos{ starting_pos }, angle{ starting_angle }, obj{ DRONE_RADIUS } {
		r_angle = DegToRad(angle);
		assert(DRONE_FOV_ANGLE <= 90); // "Angles above 90 do not follow the formula" 
		fov_half_angle = DegToRad(DRONE_FOV_ANGLE / 2);
		fov_cells = (uchar*)malloc(GRID_LENGTH * sizeof(uchar));
		drone_cells = (uchar*)malloc(GRID_LENGTH * sizeof(uchar));
	}

	void Rotate(float angle_diff) {
		// Modulo for negative numbers
		angle = (((int)(angle + angle_diff) % 360) + 360) % 360;
		r_angle = DegToRad(angle);
	}

	// dir = 1->forward, dir = -1->backward
	void Move(float distance, int dir) {
		float x_movement = dir * distance * cos(r_angle);
		float y_movement = dir * distance * sin(r_angle);
		pos = Vec2i{
			Clamp(pos[0] + (int)x_movement, 0, GRID_DIMENSIONS[0]),
			Clamp(pos[1] + (int)y_movement, 0, GRID_DIMENSIONS[1])
		};
	}

	void DrawDirection() {
		int x_dir = obj.radius * cos(r_angle) * TILE_SIZE;
		int y_dir = obj.radius * sin(r_angle) * TILE_SIZE;
		Vec2i pos = Vec2i(Clamp(pos[0], 0, GRID_DIMENSIONS[0]), Clamp(pos[1], 0, GRID_DIMENSIONS[1]));
		Vec2i end_pos = Vec2i(Clamp(pos[0] + x_dir, 0, GRID_DIMENSIONS[0]), Clamp(pos[1] + y_dir, 0, GRID_DIMENSIONS[1]));
		//pygame.draw.line(sim.screen, GREEN, pos, end_pos, width = int(HALF_TILE_SIZE));
	}

	void DrawFOV() {
		FOV fov = GetFOV();
		//pygame.draw.line(sim.screen, BLUE, fov.pos, fov.right_pos, width = int(HALF_TILE_SIZE));
		//pygame.draw.line(sim.screen, BLUE, fov.pos, fov.left_pos, width = int(HALF_TILE_SIZE));
		//pygame.draw.line(sim.screen, BLUE, fov.right_pos, fov.left_pos, width = int(HALF_TILE_SIZE));
	}

	void Draw() {
		DrawCells(drone_cells, BLUE);
		//highlight_fov(sim);
		DrawDirection();
		DrawFOV();
		//drone.draw_fov_bounding_box(self);
	}

	// Returns 3 corners of fov : drone pos, right edge corner, left edge corner
	FOV GetFOV() {
		int right_edge_x = DRONE_FOV_DEPTH * cos(r_angle + fov_half_angle);
		int right_edge_y = DRONE_FOV_DEPTH * sin(r_angle + fov_half_angle);
		int left_edge_x = DRONE_FOV_DEPTH * cos(r_angle - fov_half_angle);
		int left_edge_y = DRONE_FOV_DEPTH * sin(r_angle - fov_half_angle);
		Vec2i right_pos = Vec2i(Clamp(pos[0] + right_edge_x, 0, GRID_DIMENSIONS[0]), Clamp(pos[1] + right_edge_y, 0, GRID_DIMENSIONS[1]));
		Vec2i left_pos = Vec2i(Clamp(pos[0] + left_edge_x, 0, GRID_DIMENSIONS[0]), Clamp(pos[1] + left_edge_y, 0, GRID_DIMENSIONS[1]));
		Vec2i pos = Vec2i(Clamp(pos[0], 0, GRID_DIMENSIONS[0]), Clamp(pos[1], 0, GRID_DIMENSIONS[1]));
		return { pos, right_pos, left_pos };
	}

	bool InFOV(const Vec2i& point) {
		FOV fov = GetFOV();

		Vec2i v1 = Vec2i(point[0] - fov.pos[0], point[1] - fov.pos[1]);
		Vec2i v2 = Vec2i(point[0] - fov.left_pos[0], point[1] - fov.left_pos[1]);
		Vec2i v3 = Vec2i(point[0] - fov.right_pos[0], point[1] - fov.right_pos[1]);

		// Calculate the cross products
		int c1 = v1[0] * v2[1] - v1[1] * v2[0];
		int c2 = v2[0] * v3[1] - v2[1] * v3[0];
		int c3 = v3[0] * v1[1] - v3[1] * v1[0];

		// Check if all cross products have the same sign(inside or outside the triangle)
		return (c1 > 0 && c2 > 0 && c3 > 0) || (c1 < 0 && c2 < 0 && c3 < 0);
	}

	void ResetCellsInFOV() {
		for (int j = 0; j < GRID_SIZE[1]; j++)
		{
			int offset = j * GRID_SIZE[0];
			for (int i = 0; i < GRID_SIZE[0]; i++)
			{
				int index = i + offset;
				if (InFOV(Vec2i{ i, i })) {
					fov_cells[index] = 1;
				} else {
					fov_cells[index] = 0;
				}
			}
		}
	}

	void Update() {
		obj.SetCells(drone_cells, pos);
		ResetCellsInFOV();
	}

	void HighlightFOV() {
		DrawCells(fov_cells, LIGHT_PINK);
	}

	void DrawFOVBoundingBox() {
		BoundingRect bounding = GetBoundingRect(fov_cells);
		if (bounding.min == Vec2i{ 0, 0 } && bounding.max == Vec2i{ 0, 0 })
			return;
		//bounding_box = pygame.Rect(min_x * TILE_SIZE, min_y * TILE_SIZE, (max_x - min_x) * TILE_SIZE, (max_y - min_y) * TILE_SIZE);
		//pygame.draw.rect(sim.screen, YELLOW, bounding_box, width = 1);
	}
private:
	Circle obj;
	Vec2i pos{};
	float angle{ 0.0f };
	float r_angle{ 0.0f };
	uchar* fov_cells{ nullptr };
	uchar* drone_cells{ nullptr };
	float fov_half_angle{ 0.0f };
};
