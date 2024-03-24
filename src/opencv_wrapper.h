#ifndef OPENCV_WRAPPER_H
#define OPENCV_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif


struct contour_estimation {
  float contour_d_x;
  float contour_d_y;
  float contour_d_z;
};

struct contour_threshold {
  int lower_y, upper_y, lower_u, upper_u, lower_v, upper_v;
};

extern struct contour_estimation cont_est;
extern struct contour_threshold cont_thres;

int opencv_wrapper(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif
