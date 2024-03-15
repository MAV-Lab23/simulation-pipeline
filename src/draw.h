#pragma once

#include <drone.h>
#include <types.h>

void DrawCells(uchar* cells, const Color& color) {
	// for (x, y) in cells{
		//pygame.draw.rect(screen, color, grid[x, y], 0)
	//}
}

void DrawGrid(uchar* grid, uchar* values, int offset_x) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			//out_grid[i + offset] = 0;
			//rect = copy.copy(grid[x, y])
			//rect.x += offset_x
			//pygame.draw.rect(screen, (values[x, y], values[x, y], values[x, y]), rect, 1)
		}
	}
}

void DrawDivider() {
	//rect = pygame.Rect(GRID_SIZE[0] * TILE_SIZE, 0, DIVIDER_WIDTH, GRID_DIMENSIONS[1])
	//pygame.draw.rect(screen, GREEN, rect, 0)
}

void DrawScreen() {
	//screen.fill((0, 0, 0)) // Fill the screen with black.

	//DrawGrid(grid, values, 0)
	//DrawGrid(grid, stored_values, GRID_DIMENSIONS[0] + DIVIDER_WIDTH)

	//DrawDivider()

	//DroneDraw();

	//pygame.display.flip()
}

void DroneDrawDirection(Drone* self) {
	int x_dir = DRONE_RADIUS * cos(self->r_angle) * TILE_SIZE;
	int y_dir = DRONE_RADIUS * sin(self->r_angle) * TILE_SIZE;
	Vector2i pos = Vector2i{ Clamp(pos.x, 0, GRID_DIMENSIONS.x), Clamp(pos.y, 0, GRID_DIMENSIONS.y) };
	Vector2i end_pos = Vector2i{ Clamp(pos.x + x_dir, 0, GRID_DIMENSIONS.x), Clamp(pos.y + y_dir, 0, GRID_DIMENSIONS.y) };
	//pygame.draw.line(sim.screen, GREEN, pos, end_pos, width = int(HALF_TILE_SIZE));
}

void DroneDrawFOV(Drone* self) {
	FOV fov = DroneGetFOV(self);
	//pygame.draw.line(sim.screen, BLUE, fov.pos, fov.right_pos, width = int(HALF_TILE_SIZE));
	//pygame.draw.line(sim.screen, BLUE, fov.pos, fov.left_pos, width = int(HALF_TILE_SIZE));
	//pygame.draw.line(sim.screen, BLUE, fov.right_pos, fov.left_pos, width = int(HALF_TILE_SIZE));
}

void DrawDrone(Drone* self) {
	DrawCells(self->drone_cells, BLUE);
	//highlight_fov(sim);
	DroneDrawDirection(self);
	DroneDrawFOV(self);
	//drone.draw_fov_bounding_box(self);
}

void DroneHighlightFOV(Drone* self) {
	DrawCells(self->fov_cells, LIGHT_PINK);
}

void DroneDrawFOVBoundingBox(Drone* self) {
	BoundingRect bounding = GetBoundingRect(self->fov_cells);
	if (bounding.min.x == 0 && bounding.min.y == 0 && bounding.max.x == 0 && bounding.max.y == 0)
		return;
	//bounding_box = pygame.Rect(min_x * TILE_SIZE, min_y * TILE_SIZE, (max_x - min_x) * TILE_SIZE, (max_y - min_y) * TILE_SIZE);
	//pygame.draw.rect(sim.screen, YELLOW, bounding_box, width = 1);
}