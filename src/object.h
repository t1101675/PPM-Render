#ifndef OBJECT_H_
#define OBJECT_H_

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <string>

#include "vec3.hpp"
#include "light.hpp"

struct AABB {
    Vec3 maxBound;
    Vec3 minBound;
    Vec3 center;

    AABB() {
        maxBound = Vec3();
        minBound = Vec3();
        center = Vec3();
    }

    bool inAABB(const Vec3& p) const {
        return !((minBound.x > p.x + PZ) || (p.x > maxBound.x + PZ) ||
               (minBound.y > p.y + PZ) || (p.y > maxBound.y + PZ) ||
               (minBound.z > p.z + PZ) || (p.z > maxBound.z + PZ));
    }

    void set(const Vec3& maxB, const Vec3& minB) {
        maxBound = maxB;
        minBound = minB;
        center = (maxBound + minBound) / 2;
    }

    bool intersect(const Light& l) {
        if (inAABB(l.p0)) { // inside
            return true;
        }
        else {
            d_t t = -INF;
            if ((l.norm.x < NZ) || (l.norm.x > PZ))
                t = maxd(t, mind((minBound.x - l.p0.x) / l.norm.x, (maxBound.x - l.p0.x) / l.norm.x));
            if ((l.norm.y < NZ) || (l.norm.y > PZ))
                t = maxd(t, mind((minBound.y - l.p0.y) / l.norm.y, (maxBound.y - l.p0.y) / l.norm.y));
            if ((l.norm.z < NZ) || (l.norm.z > PZ))
                t = maxd(t, mind((minBound.z - l.p0.z) / l.norm.z, (maxBound.z - l.p0.z) / l.norm.z));
            if (t < NZ)
                return false;
            Vec3 hit = l.lightPoint(t);
            if (!inAABB(hit)) {
                return false;
            }
            return true;
        }
    }
};


class Object {
public:
    d_t wr; //反射率
    d_t wt; //透射率
    d_t eda; //折射率
    d_t Ks; //反射高光
    d_t Kt; //透射高光
    d_t Kds; //漫反射
    d_t Kdt; //漫透射
    d_t Ka; //环境光
    Color c; //物体颜色
    std::string name;
    cv::Mat texture;

    bool hasTexture;
    bool fromJson; //从json创建完后一定要设置这个否则会发生内存泄漏
    
    AABB aabb;
    Object(d_t wr = 0, d_t wt = 0, d_t eda = 1, const Color& c = Color(), d_t Ks = 3, d_t Kt = 0.1, d_t Kds = 3, d_t Kdt = 0.1, d_t Ka = 1) {
        this->wr = wr;
        this->wt = wt;
        this->eda = eda;
        this->c = c;
        this->Ks = Ks;
        this->Kt = Kt;
        this->Kds = Kds;
        this->Kdt = Kdt;
        this->Ka = Ka;
        this->fromJson = false;
        this->hasTexture = false;
    }
    virtual ~Object();
    void loadMaterial(const Json::Value& config);
    virtual void loadTexture(const std::string& path);
    virtual void computeAABB() = 0;
    virtual bool intersect(const Light& l) const = 0;
    virtual bool intersect(const Light& l, Light& rl, Light& tl) const = 0;
    virtual bool intersect(const Light& l, Vec3& p, Vec3& n) const = 0;
    virtual Light RL(const Vec3 &p, const Vec3& norm, const Light& l) const;
    virtual Light TL(const Vec3 &p, const Vec3& norm, const Light& l, bool in, bool& valid, d_t& R) const;
    virtual Color getColor(const Vec3& p) const;
    virtual Color getColor(d_t xx, d_t yy) const;
    virtual Color getColor(int xx, int yy) const;
    virtual Light generateLight(unsigned short* X) const { return Light(); }
    
};

class ObjCmp {
private:
    Object** objs;
    int D;
public:
    ObjCmp(int D, Object** objs) {
        this->objs = objs;
        this->D = D;
    }
    bool operator() (int index1, int index2) {
        if (D == 0)
            return objs[index1]->aabb.center.x < objs[index2]->aabb.center.x;
        else if (D == 1)
            return objs[index1]->aabb.center.y < objs[index2]->aabb.center.y;
        else
            return objs[index1]->aabb.center.z < objs[index2]->aabb.center.z;
    }
};

#endif //OBJECT_H_