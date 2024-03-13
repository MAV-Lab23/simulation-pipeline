#pragma once

#include <stdio.h>
#include <types.h>
#include <utility.h>
#include <drone.h>
#include <draw.h>

void InitSimulation(Simulation* self) {
	self->values = CreateEmptyGrid();
	self->stored_values = CreateEmptyGrid();
	//stored_values = np.full((GRID_SIZE[0], GRID_SIZE[1]), 255);
	self->drone = Drone{ GRID_DIMENSIONS.x / 2, GRID_DIMENSIONS.y / 2, 0, 0, DRONE_FOV_ANGLE / 2, NULL, NULL };
}

// direction is - 1 for decrease and 1 for increase
void ChangeValues(Simulation* self, int direction) {
	for (int j = 0; j < GRID_SIZE.y; j++)
	{
		int offset = j * GRID_SIZE.x;
		for (int i = 0; i < GRID_SIZE.x; i++)
		{
			self->values[i + offset] = Clamp(self->values[i + offset] + direction * VALUE_CHANGE, 0, 255);
		}
	}
}

void ClearValues(Simulation* self) {
	 ClearGrid(self->values);
}

void StoreValues(Simulation* self) {
	BoundingRect bounding = GetBoundingRect(self->drone.fov_cells);

	// Ensure valid coordinates within array bounds
	int x1 = MAX(0, bounding.min.x);  // Clamp x1 to avoid negative indexing
	int y1 = MAX(0, bounding.min.y);  // Clamp y1 to avoid negative indexing
	int x2 = MIN(GRID_DIMENSIONS.y, bounding.max.x);  // Clamp x2 to array width
	int y2 = MIN(GRID_DIMENSIONS.x, bounding.max.y);  // Clamp y2 to array height

	//fov_relative_cells = np.empty_like(drone.fov_cells);

	//for i, cell in enumerate(drone.fov_cells) {
		// fov cells indexes are relative to entire stored_values grid, so shift blur_cells relative to subgrid
		//fov_relative_cells[i] = (max(0, cell[0] - x1 - 1), max(0, cell[1] - y1 - 1))
	//}
	// only blur grid in field of view
	//stored_values[x1:x2, y1 : y2] = WeightedCombine(values[x1:x2, y1 : y2], \
		stored_values[x1:x2, y1 : y2], \
		fov_relative_cells,
		//CURRENT_IMAGE_WEIGHT)
	//}
}

void UpdateSimulation(Simulation* self, float dt, int i) {
	// HandleInputEvents();

	DroneUpdate(&self->drone);

	// Update stored values automatically every X cycles
	//if i % 10 == 0{
	StoreValues(self);
}

void RunSimulation() {
	Simulation* self = (Simulation*)malloc(1 * sizeof(Simulation));

	//pygame.init()

	float fps = 60.0;
	// fpsClock = pygame.time.Clock()

	// screen = pygame.display.set_mode((WINDOW_SIZE[0], WINDOW_SIZE[1]))
	// pygame.display.set_caption('cycle shape{ X, resize shape{ arrow keys, rotate OBB{ Z/C, draw{ left mouse, erase{ right mouse, clear{ V, update grid{ space, move & rotate drone{ W/S & Q/E')

	InitSimulation(self);

	int i = 0;
	float dt = 1 / fps; // dt is the time since last frame.
	while (true) { // Loop forever!
		UpdateSimulation(self, dt, i); // You can update / draw here, I've just moved the code for neatness.
		DrawScreen();
		// dt = fpsClock.tick(fps)
		i += 1;
	}
}