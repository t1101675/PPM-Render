#include <vector>
#include <iostream>

#include "utils.h"
#include "vec3.hpp"
#include "bezier.h"

Bezier::Bezier(d_t *px, d_t *py, int n, int divideNum, d_t r) {
    if (px && py) {
        build(px, py, n, divideNum, r);
    }
    else {
        this->n = n;
        this->divideNum = divideNum;
        this->r = r;
        this->dx = nullptr;
        this->dy = nullptr;
        this->max = 0;
        this->height = 0;
        this->max2 = 0;
    }
}

void Bezier::build(d_t *px, d_t *py, int n, int divideNum, d_t r) {
    this->n = n;
    this->divideNum = divideNum;
    this->r = r;
    dx = new d_t[n];
    dy = new d_t[n];
    assert(absd(py[0]) <= 1e-6);
    --n;
    // preproces
    for (int i = 0; i <= n; ++i) {
        dx[i] = px[0];
        dy[i] = py[0];
        for (int j = 0; j <= n - i; ++j) {
            px[j] = px[j + 1] - px[j];
            py[j] = py[j + 1] - py[j];
        }
    }
    d_t n_down = 1, fac = 1, nxt = n;
    for (int i = 0; i <= n; ++i, --nxt) {
        fac = fac * (i == 0 ? 1 : i);
        dx[i] = dx[i] * n_down / fac;
        dy[i] = dy[i] * n_down / fac;
        n_down *= nxt;
    }
    max = 0;
    d_t interval = 1. / (divideNum - 1), c = 0;
    for (int cnt = 0; cnt <= divideNum; c += interval, ++cnt) {
        data[cnt].width = 0;
        data[cnt].t0 = maxd(0., c - r);
        data[cnt].t1 = mind(1., c + r);
        data[cnt].y0 = getpos(data[cnt].t0).y;
        data[cnt].y1 = getpos(data[cnt].t1).y;
        for (d_t t = data[cnt].t0; t <= data[cnt].t1; t += 0.00001) {
            Vec3 pos = getpos(t);
            if (data[cnt].width < pos.x)
                data[cnt].width = pos.x;
        }
        if (max < data[cnt].width)
            max = data[cnt].width;
        data[cnt].width += PZ;
        data[cnt].width2 = data[cnt].width * data[cnt].width;
    }
    max += PZ;
    max2 = max * max;
    height = getpos(1).y;
}

Vec3 Bezier::getpos(d_t t) const {
    d_t ans_x = 0, ans_y = 0, t_pow = 1;
    for (int i = 0; i <= n; ++i) {
        ans_x += dx[i] * t_pow;
        ans_y += dy[i] * t_pow;
        t_pow *= t;
    }
    return Vec3(ans_x, ans_y);
}
Vec3 Bezier::getdir(d_t t) const {
    d_t ans_x = 0, ans_y = 0, t_pow = 1;
    for (int i = 1; i <= n; ++i) {
        ans_x += dx[i] * i * t_pow;
        ans_y += dy[i] * i * t_pow;
        t_pow *= t;
    }
    return Vec3(ans_x, ans_y);
}
Vec3 Bezier::getdir2(d_t t) const {
    d_t ans_x = 0, ans_y = 0, t_pow = 1;
    for (int i = 2; i <= n; ++i) {
        ans_x += dx[i] * i * (i - 1) * t_pow;
        ans_y += dy[i] * i * (i - 1) * t_pow;
        t_pow *= t;
    }
    return Vec3(ans_x, ans_y);
}
