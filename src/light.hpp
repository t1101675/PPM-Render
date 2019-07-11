#ifndef LIGHT_H_
#define LIGHT_H_

#include "vec3.hpp"
#include "utils.h"

class Photon {
public:
    Vec3 p; //位置
    Vec3 n; //方向
    Color c; //颜色
    d_t r; //送光半径

    Photon(const Vec3& p = Vec3(), const Vec3& n = Vec3(), const Color& c = Color()) {
        this->p = p;
        this->n = n;
        this->c = c;
    }
};

class ViewPoint {
public:
    Vec3 p;
    Color weight;
    int index;
    ViewPoint(const Vec3& p = Vec3(), const Vec3& dir = Vec3(), const Color& weight = Color(), int index = -1) {
        this->p = p;
        this->weight = weight;
        this->index = index;
    }

    ViewPoint& operator=(const ViewPoint& vp) {
        if (this == &vp) return *this;
        this->p = vp.p;
        this->weight = vp.weight;
        this->index = vp.index;
        return *this;
    }
};

class ViewPointCmp {
public:
    int D;
    ViewPoint* vpB;
    ViewPointCmp(int D, ViewPoint* vpB) { 
        this->D = D; 
        this->vpB = vpB;
    }
    bool operator()(int vp1, int vp2) const {
        if (D == 0)
            return vpB[vp1].p.x < vpB[vp2].p.x;
        else if (D == 1)
            return vpB[vp1].p.y < vpB[vp2].p.y;
        else
            return vpB[vp1].p.z < vpB[vp2].p.z;
    }
};

class Light {
public:
    Vec3 p0;
    Vec3 norm;
    d_t length;
    Light(const Vec3 &p0 = Vec3(), const Vec3 &norm = Vec3(0, 0, 1), d_t length = MAX_DOUBLE) {
        this->p0 = p0;
        this->norm = norm.normal();
        this->length = length;
    }
    Vec3 lightPoint(d_t t) const { return p0 + t * norm; }
};

#endif //LIGHT_H_