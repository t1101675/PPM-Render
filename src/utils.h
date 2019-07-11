#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <json/json.h>
#include <math.h>
#include <omp.h>
#include <time.h>
#include <stdlib.h>

#include "vec3.hpp"

bool qe(d_t A, d_t B, d_t C, d_t& x1, d_t& x2);
inline d_t squareDistance(const Vec3 &p1, const Vec3 &p2) { return p1.squareDistance(p2); }
inline d_t distance(const Vec3 &p1, const Vec3 &p2) { return p1.distance(p2); }
void printVec3(const Vec3 &v);
inline d_t maxd(d_t d1, d_t d2) { return d1 < d2 ? d2 : d1; }
inline d_t mind(d_t d1, d_t d2) { return d1 < d2 ? d1 : d2; }
inline d_t absd(d_t d) { return d > 0 ? d : -d; }
inline Vec3 eachMax(const Vec3 &v1, const Vec3 &v2) { return Vec3(maxd(v1.x, v2.x), maxd(v1.y, v2.y), maxd(v1.z, v2.z)); }
inline Vec3 eachMin(const Vec3 &v1, const Vec3 &v2) { return Vec3(mind(v1.x, v2.x), mind(v1.y, v2.y), mind(v1.z, v2.z)); }
inline d_t det(const Vec3& a, const Vec3& b, const Vec3& c) { return a.x * (b.y * c.z - b.z * c.y) - b.x * (a.y * c.z - a.z * c.y) + c.x * (a.y * b.z - a.z * b.y); }
Vec3 randDir(const Vec3& norm, unsigned short* X);
Vec3 jArr2Vec3(const Json::Value& arr);
Vec3 doGamma(const Color& color, d_t gamma);

const d_t PZ = d_t(1e-6);
const d_t NZ = d_t(-1e-6);
const d_t MAX_DOUBLE = 1e8;
const d_t pix2coor = 200;
const Color BLACK(0, 0, 0);
const Color RED(1.0, 0, 0);
const Color BLUE(0, 0, 1.0);
const Color GREEN(0, 1.0, 0);
const Color WHITE(1.0, 1.0, 1.0);
const int MAX_KDNODE = 60000000;
const int MAX_OBJNODE = 400000;
const d_t PI = 3.14159265;
const d_t INIT_R = 20;
const d_t INF = d_t(1 << 20);



#endif //UTILS_H_