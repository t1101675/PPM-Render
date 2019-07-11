#ifndef _BOBJ_H
#define _BOBJ_H

#include <assert.h>

#include "object.h"
#include "bezier.h"
#include "light.hpp"

class BezierObj: public Object {
    //z 轴放置
private:
    Vec3 pos;
    Bezier bezier;

    d_t solve_t(d_t y0) const { 
        d_t t = 0.5;
        for (int i = 10; i--;)
        {
            if (t < 0)
                t = 0;
            else if (t > 1)
                t = 1;
            d_t f = bezier.getpos(t).y - y0, df = bezier.getdir(t).y;
            if (NZ < f && f < PZ)
                return t;
            t -= f / df;
        }
        return -1;
    }
    virtual Vec3 change_for_bezier(Vec3 inter_p) const {
        d_t t = solve_t(inter_p.z - pos.z);
        d_t theta = atan2(inter_p.y - pos.y, inter_p.x - pos.x); // between -pi ~ pi
        if (theta < 0)
            theta += 2 * PI;
        return Vec3(theta, t, 0);
    }
    d_t get_sphere_intersect(Light light, Vec3 p0, d_t r) const {
        Vec3 ro = p0 - light.p0;
        d_t b = light.norm.dot(ro);
        d_t d = b * b - ro.dot(ro) + r * r;
        if (d < 0)
            return -1;
        else
            d = sqrt(d);
        d_t t = b - d > PZ ? b - d : b + d > PZ ? b + d : -1;
        if (t < 0)
            return -1;
        return t;
    }
    bool check(d_t tLow, d_t tUpp, d_t tInit, Light light, d_t a, d_t b, d_t c, d_t &final_dis) const {
        d_t t = newton(tInit, a, b, c, tLow, tUpp);
        if (t <= 0 || t >= 1)
            return false;
        Vec3 bPos = bezier.getpos(t);
        d_t bx = bPos.x, by = bPos.y;
        d_t err = bx - sqrt(a * (by - b) * (by - b) + c);
        if (absd(err) > 100 * PZ)
            return false;
        d_t dis = (pos.z + by - light.p0.z) / light.norm.z;
        if (dis < PZ)
            return false;

        Vec3 inter_p = light.lightPoint(dis);
        if (absd(squareDistance(Vec3(pos.x, pos.y, pos.z + by), inter_p) - bx * bx) > 100 * PZ)
            return false;
        if (dis < final_dis) {
            final_dis = dis;
            return true;
        }
        return false;
    }
    d_t getft(d_t t, d_t a, d_t b, d_t c) const {
        if (t < 0)
            t = PZ;
        if (t > 1)
            t = 1 - PZ;
        Vec3 loc = bezier.getpos(t);
        d_t x = loc.x, y = loc.y;
        return x - sqrt(a * (y - b) * (y - b)+ c);
    }
    d_t newton(d_t t, d_t a, d_t b, d_t c, d_t low = PZ, d_t upp = 1 - PZ) const {
        // solve sqrt(a( y(t) + bPos.y - b) ^ 2 + c) = x(t)
        // f(t) = x(t) - sqrt( a(y(t) + bPos.y - b) ^ 2 + c)
        // f'(t) = x'(t) - a(y(t) + bPos.y - b) * y'(t) / sqrt(...)

        d_t f = 0, df = 0, bx = 0, by = 0, dbx = 0, dby = 0, sq = 0;
        Vec3 bPos, bdir;
        for (int i = 10; i--;)
        {
            if (t < 0)
                t = low;
            if (t > 1)
                t = upp;
            bPos = bezier.getpos(t), bdir = bezier.getdir(t);
            bx = bPos.x, dbx = bdir.x;
            by = bPos.y, dby = bdir.y;
            sq = sqrt(a * (by - b) * (by - b) + c);
            f = bx - sq;
            df = dbx - a * (by - b) * dby / sq;
            if (absd(f) < PZ)
                return t;
            t -= f / df;
        }
        return -1;
    }
    d_t newton2(d_t t, d_t a, d_t b, d_t c) {
        d_t dft, ddft, y, dx, dy, ddx, ddy, sq;
        Vec3 loc, dir, dir2;
        for (int i = 5; i--;)
        {
            if (t < 0)
                t = PZ;
            if (t > 1)
                t = 1 - PZ;
            loc = bezier.getpos(t), dir = bezier.getdir(t), dir2 = bezier.getdir2(t);
            y = loc.y, dx = dir.x, dy = dir.y;
            ddx = dir2.x, ddy = dir2.y;
            sq = sqrt(a * (y - b) * (y - b) + c);
            dft = dx - a * (y - b) * dy / sq;
            d_t temp = a * (y - b) * dy;
            ddft = ddx - a * ((y - b) * ddy + dy * dy) / sq + temp * temp / sq / sq / sq;
            if (std::abs(dft) < PZ)
                return t;
            t -= dft / ddft;
        }
        return -1;
    }

public:
    
    BezierObj(d_t x0 = 0.0, d_t y0 = 0.0, d_t z0 = 0.0, d_t* px = nullptr, d_t* py = nullptr, int n = 0, int num = 0, d_t r = 0.0) : Object(), pos(x0, y0, z0), bezier(px, py, n, num, r) {

    }
    Vec3 norm(const Vec3 p) const {
        Vec3 tmp = change_for_bezier(p); //(theta, t, 0)
        Vec3 dir = bezier.getdir(tmp.y);
        Vec3 d_surface = Vec3(cos(tmp.x), sin(tmp.x), dir.y / dir.x);
        Vec3 d_circ = Vec3(-sin(tmp.x), cos(tmp.x), 0);
        Vec3 n = d_circ.cross(d_surface).normal();
        if (n.dot(Vec3(cos(tmp.x), sin(tmp.x), 0)) < 0) n = -n; //outside
        return n;
    }
    
    void load(const Json::Value& config) {
        assert(config["type"].asString() == "bezier");
        this->name = config["name"].asString();
        this->pos = jArr2Vec3(config["pos"]);
        int n = config["n"].asInt();
        d_t px[n];
        for (int i = 0; i < n; ++i) { px[i] = config["px"][i].asDouble(); }
        d_t py[n];
        for (int i = 0; i < n; ++i) { py[i] = config["py"][i].asDouble(); }
        this->bezier.build(px, py, n, config["d_num"].asInt(), config["d_r"].asDouble());
        Object::loadMaterial(config["material"]);
        computeAABB();
    }

    void computeAABB() {
        aabb.set(Vec3(pos.x + bezier.max, pos.y + bezier.max, pos.z + bezier.height), Vec3(pos.x - bezier.max, pos.y - bezier.max, pos.z));
    }

    bool intersect(const Light& l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }

    bool intersect(const Light &l, Light &rl, Light &tl) const { return false; }
    
    bool intersect(const Light &light, Vec3 &p, Vec3 &n) const {
        d_t final_dis = INF;
        if (absd(light.norm.z) < 5e-4) {
            d_t dis2Axis = distance(Vec3(pos.x, pos.y, light.p0.z), light.p0);
            Vec3 hitP;
            for (int i = 0; i < 2; ++i) {
                d_t hitZ = light.lightPoint(dis2Axis).z;
                if (hitZ < pos.z + PZ || hitZ > pos.z + bezier.height - PZ)
                    return false;
                d_t t = solve_t(hitZ - pos.z);
                if (t < 0 || t > 1)
                    //outside
                    return false;
                Vec3 bPos = bezier.getpos(t);
                d_t err = pos.z + bPos.y - hitZ;
                if (err < NZ || err > PZ)
                    return false;
                final_dis = get_sphere_intersect(light, Vec3(pos.x, pos.y, pos.z + bPos.y), bPos.x);
                if (final_dis < 0)
                    return false;
                hitP = light.lightPoint(final_dis);
                if (absd(squareDistance(hitP, Vec3(pos.x, pos.y, hitP.z)) - bPos.x * bPos.x) > 1e-1)
                    return false;
            }
            p = hitP;
            n = norm(p);
            return true;
        }

        d_t a = 0, b = 0, c = 0, t1 = 0, t2 = 0;
        t1 = light.p0.x - pos.x - light.norm.x / light.norm.z * light.p0.z;
        t2 = light.norm.x / light.norm.z;
        a += t2 * t2;
        b += 2 * t1 * t2;
        c += t1 * t1;
        t1 = light.p0.y - pos.y - light.norm.y / light.norm.z * light.p0.z;
        t2 = light.norm.y / light.norm.z;
        a += t2 * t2;
        b += 2 * t1 * t2;
        c += t1 * t1;
        c = c - b * b / 4 / a;
        b = -b / 2 / a - pos.z;
        if ((0 <= b && b <= bezier.height && c > bezier.max2) 
        || ((b < 0 || b > bezier.height) && mind(b * b, (bezier.height - b) * (bezier.height - b)) * a + c > bezier.max2))
        {
            // 二次函数问题 满满的回忆...
            return false;
        }
        for (int i = 0; i <= bezier.divideNum; ++i) {
            d_t t0 = bezier.data[i].t0, t1 = bezier.data[i].t1;
            check(t0, t1, (1 * t0 + 2 * t1) / 3, light, a, b, c, final_dis);
            check(t0, t1, (2 * t0 + 1 * t1) / 3, light, a, b, c, final_dis);
        }
        if (final_dis < INF / 2) {
            p = light.lightPoint(final_dis);
            n = norm(p); 
            return true;
        }
        else {
            return false;
        }
    }

    Color getColor(const Vec3& p) const {
        if (!hasTexture) {
            return this->c;
        }
        Vec3 pos = change_for_bezier(p); // theta t
        if (pos.x > PI) {
            pos.x -= 2 * PI;
        }
        pos.x += PI;
        return Object::getColor(pos.x / (2 * PI), pos.y);
    }
};


#endif