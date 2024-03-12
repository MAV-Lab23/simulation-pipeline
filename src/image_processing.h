#pragma once

#include <types.h>
#include <constants.h>
#include <utility.h>
#include <opencv2/core/mat.hpp>

cv::Mat WeightedCombine(const cv::Mat& addition, const cv::Mat& storage, float addition_weight) {
	assert(addition_weight >= 0.0f && addition_weight <= 1.0f);
	assert(addition != nullptr && out_storage != nullptr);

	float storage_weight = 1.0f - addition_weight;

	return addition_weight * addition + storage_weight * storage;
}