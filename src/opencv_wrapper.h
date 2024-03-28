#ifndef GROUP_10_OPENCV_WRAPPER_H
#define GROUP_10_OPENCV_WRAPPER_H

#include "constants.h"

#ifdef __cplusplus
extern "C" {
#endif

void addNavigationObstacle(int x, int y);
float getNavigationHeading();
void parseImage(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif

