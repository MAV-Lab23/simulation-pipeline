#pragma once

#include <constants.h>
#include <utility.h>
#include <types.h>

void SetCircleCells(uchar* out_cells, const Vector2i& pos, int rad) {
	const Vector2i grid_pos = WorldToGrid(pos);
	int rad2 = rad * rad;
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			Vector2i dist{ i - grid_pos.x, j - grid_pos.y };
			out_cells[i + offset] = (int)(dist.x * dist.x + dist.y * dist.y <= rad2);
		}
	}
}

bool RectanglePointOverlap(const Vector2i& coord, const Vector2i& pos, const Vector2i& size, float angle /* degrees */) {
	Vector2i t = { pos.x - coord.x, pos.y - coord.y };

	Vector2i half_size = { size.x / 2, size.y / 2 };
	int b_radius_sq = half_size.x * half_size.x + half_size.y * half_size.y;
	float r_angle = DegToRad(angle);

	if (t.x * t.x + t.y * t.y > b_radius_sq)
		return false;

	float costheta = cos(r_angle);
	float sintheta = sin(r_angle);

	int rx = (int)(t.x * costheta - t.y * sintheta);
	int ry = (int)(t.y * costheta + t.x * sintheta);

	if (rx > -half_size.x &&
		rx < half_size.x &&
		ry > -half_size.y &&
		ry < half_size.y)
		return true;

	return false;
}

// pos is relative to world
void SetRectangleCells(uchar* out_cells, const Vector2i& pos, const Vector2i& size, float angle /* degrees */) {
	const Vector2i grid_pos = WorldToGrid(pos);
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			Vector2i coord{ i, j };
			out_cells[i + offset] = RectanglePointOverlap(coord, pos, size, angle);
		}
	}
}
