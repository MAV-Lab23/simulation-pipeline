#ifndef OPENCV_WRAPPER_H
#define OPENCV_WRAPPER_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GROUP_10_OPENCV
int opencv_wrapper(char *img, int width, int height, const DroneState state);
#endif

#ifdef __cplusplus
}
#endif

#endif

