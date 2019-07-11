#ifndef __BEZIER_H
#define __BEZIER_H

#include <assert.h>

#include "utils.h"
#include "vec3.hpp"

struct D {
    d_t t0, t1;
    d_t width;
    d_t y0, y1;
    d_t width2;
};

class Bezier {
public:
    d_t *dx, *dy;
    d_t max, height, max2, r, divideNum;
    int n;
    D data[20];
    Bezier(d_t *px, d_t *py, int n, int divideNum, d_t r);
    void build(d_t *px, d_t *py, int n, int divideNum, d_t r);
    Vec3 getpos(d_t t) const;
    Vec3 getdir(d_t t) const;
    Vec3 getdir2(d_t t) const;
};

#endif // __BEZIER_H