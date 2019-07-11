#ifndef VEC3_H_
#define VEC3_H_

#include <math.h>
#include <iostream>

typedef double d_t;
typedef float f_t;

class Vec3 {
public:
    d_t x, y, z;
    Vec3(d_t xx = 0, d_t yy = 0, d_t zz = 0): x(xx), y(yy), z(zz) {}
    Vec3(const Vec3& p): Vec3(p.x, p.y, p.z) {}

    Vec3 normal() const { d_t len = length(); return len < 1e-8 ? Vec3() : Vec3(x / len, y / len, z / len); }

    friend Vec3 operator+ (const Vec3 &v1, const Vec3 &v2) { return Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z); }
    friend Vec3 operator- (const Vec3 &v1, const Vec3 &v2) { return Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z); }
    friend Vec3 operator* (d_t a, const Vec3 &v) { return Vec3(v.x * a, v.y * a, v.z * a); }
    friend Vec3 operator* (const Vec3 &v, d_t a) { return Vec3(v.x * a, v.y * a, v.z * a); }
    friend Vec3 operator* (const Vec3 &v1, const Vec3 &v2) { return Vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z); } 
    friend Vec3 operator- (const Vec3 &v) { return Vec3(-v.x, -v.y, -v.z); }
    friend Vec3 operator+ (const Vec3 &v, d_t r) { return Vec3(v.x + r, v.y + r, v.z + r); }
    friend Vec3 operator- (const Vec3 &v, d_t r) { return Vec3(v.x - r, v.y - r, v.z - r); }
    friend Vec3 operator/ (const Vec3 &v, d_t a) { return Vec3(v.x / a, v.y / a, v.z / a); }
    void operator+= (const Vec3 &v) { x += v.x; y += v.y; z += v.z; }


    Vec3 cross(const Vec3& v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    d_t dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    d_t length() const { return sqrt(x * x + y * y + z * z); }
    d_t squareLength() const { return x * x + y * y + z * z; }
    d_t distance(const Vec3& p) const { return (Vec3(x - p.x, y - p.y, z - p.z)).length(); }
    d_t squareDistance(const Vec3& p) const { return (Vec3(x - p.x, y - p.y, z - p.z)).squareLength();} 
};

typedef Vec3 Color;

#endif //VEC3_H_