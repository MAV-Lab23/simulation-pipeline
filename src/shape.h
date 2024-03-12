#pragma once

#include <types.h>
#include <constants.h>
#include <utility.h>
#include <limits.h>

class Shape {
public:
	virtual void SetCells(uint8_t* out_cells, Vec2i pos) = 0;
	virtual void IncreaseSize(Vec2i amount) = 0;
	virtual void Rotate(float angle_amount) = 0;
	virtual ~Shape() = default;
};

class Circle : public Shape {
public:
	Circle(int radius) : radius{ radius }, rad2{ radius * radius } {}
	// pos is relative to world
	virtual void SetCells(uint8_t* out_cells, Vec2i pos) override final {
		const Vec2i grid_pos = WorldToGrid(pos);
		for (int j = 0; j < GRID_SIZE[1]; j++)
		{
			int offset = j * GRID_SIZE[0];
			for (int i = 0; i < GRID_SIZE[0]; i++)
			{
				int index = i + offset;
				Vec2i dist = Vec2i{ i, j } - grid_pos;
				out_cells[i] = (int)(dist.dot(dist) <= rad2);
			}
		}
	}
	virtual void IncreaseSize(Vec2i amount) override final {
		radius = Clamp(radius + amount[0], 1, MIN(GRID_SIZE[0], GRID_SIZE[1]));
		rad2 = radius * radius;
	}
	virtual void Rotate(float angle_amount) override final {}
public:
	int radius{ 0 };
private:
	int rad2{ 0 };
};

class Rectangle : public Shape {
public:
	Rectangle(Vec2i size) : size{ size }, angle{ angle }, r_angle{ DegToRad(angle) } {
		ResetExtents(size);
	}
	// pos is relative to world
	virtual void SetCells(uint8_t* out_cells, Vec2i pos) override final {
		const Vec2i grid_pos = WorldToGrid(pos);
		for (int j = 0; j < GRID_SIZE[1]; j++)
		{
			int offset = j * GRID_SIZE[0];
			for (int i = 0; i < GRID_SIZE[0]; i++)
			{
				int index = i + offset;
				out_cells[i] = Overlaps(Vec2i{ i, j }, pos);
			}
		}
	}
	virtual void IncreaseSize(Vec2i amount) override final {
		ResetExtents(Vec2i{
			Clamp(size[0] + amount[0], 1, MIN(GRID_SIZE[0], GRID_SIZE[1])),
			Clamp(size[1] + amount[1], 1, MIN(GRID_SIZE[0], GRID_SIZE[1]))
		});
	}
	virtual void Rotate(float angle_amount) override final {
		// Modulo for negative numbers
		angle = (((int)(angle + angle_amount) % 360) + 360) % 360;
		r_angle = DegToRad(angle);
	}

	bool Overlaps(Vec2i coord, Vec2i pos) {
		Vec2i t = pos - coord;

		if (t[0] * t[0] + t[1] * t[1] > b_radius_sq)
			return false;

		float costheta = cos(r_angle);
		float sintheta = sin(r_angle);

		int rx = (int)(t[0] * costheta - t[1] * sintheta);
		int ry = (int)(t[1] * costheta + t[0] * sintheta);

		if (rx > -half_size[0] &&
			rx < half_size[0] &&
			ry > -half_size[1] &&
			ry < half_size[1])
			return true;

		return false;
	}
private:
	void ResetExtents(Vec2i size) {
		this->size = size;
		half_size = this->size / 2;
		b_radius_sq = half_size[0] * half_size[0] + half_size[1] * half_size[1];
	}
public:
	Vec2i size{}; // screen relative
	float angle{ 0 }; // degrees
private:
	Vec2i half_size{};
	float r_angle{ 0 }; // degrees
	int b_radius_sq{ 0 };
};
