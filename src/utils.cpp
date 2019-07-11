#include "utils.h"

#include <json/json.h>
#include <stdlib.h>

bool qe(d_t A, d_t B, d_t C, d_t &x1, d_t &x2) { return false; }

Vec3 randDir(const Vec3& norm, unsigned short* X) {
    d_t phi = 2 * PI * erand48(X);
    d_t cos_theta2 = erand48(X);
    d_t cos_theta = sqrt(cos_theta2);
    d_t sin_theta = sqrt(1 - cos_theta2);
    Vec3 w = norm; //u, v, w, 坐标系
    Vec3 u = absd(w.x) > 0.1 ? w.cross(Vec3(0, 1, 0)).normal() : w.cross(Vec3(1, 0, 0)).normal();
    Vec3 v = w.cross(u);
    return (u * cos(phi) * cos_theta + v * sin(phi) * cos_theta + w * sin_theta); //球坐标
}

void printVec3(const Vec3 &v) {
    std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")" << " ";
}

Vec3 jArr2Vec3(const Json::Value& arr) {
    return Vec3(arr[0u].asDouble(), arr[1u].asDouble(), arr[2u].asDouble());
}

Vec3 doGamma(const Color& color, d_t gamma) {
    return Color(pow(color.x, gamma), pow(color.y, gamma), pow(color.z, gamma));
}
