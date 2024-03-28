#ifndef GROUP_10_UTILITY_H
#define GROUP_10_UTILITY_H

#include <stdio.h>

// Clamp value between min and max
static float clamp(float d, float min, float max) {
	const float t = d < min ? min : d;
	return t > max ? max : t;
}

// Normalize a value to the 0.0->1.0 range given its current range bounds.
static float normalize(float value, float min, float max) {
	if (min == max) return 0.5;
	return (value - min) / (max - min);
}

#endif
