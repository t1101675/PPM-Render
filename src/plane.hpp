#ifndef PLANE_H_
#define PLANE_H_

#include <assert.h>

#include "object.h"
#include "vec3.hpp"
#include "utils.h"

class Plane: public Object {
public:
    Vec3 _p0;
    Vec3 _norm;
    Vec3 _textureXDir;
    d_t _textureWidth;
    d_t _textureHeight;
    Plane(const Vec3& norm = Vec3(), const Vec3& p0 = Vec3(), const Color& co = BLACK, d_t wr = 1.0, d_t wt = 0.0, d_t eda = 0.5): Object(wr, wt, eda, co){
        this->_norm = norm.normal();
        this->_p0 = p0;
        this->_textureXDir = Vec3();
        this->_textureHeight = 0.0;
        this->_textureWidth = 0.0;
    }

    void load(const Json::Value& config) {
        assert(config["type"].asString() == "plane");
        this->name = config["name"].asString();
        this->_p0 = jArr2Vec3(config["p"]);
        this->_norm = jArr2Vec3(config["n"]).normal();
        this->_textureXDir = jArr2Vec3(config["material"]["texture_x_dir"]);
        this->_textureHeight = config["material"]["texture_h"].asDouble();
        this->_textureWidth = config["material"]["texture_w"].asDouble();
        Object::loadMaterial(config["material"]);
        computeAABB();
    }

    virtual void computeAABB() {
        aabb.set(Vec3(INF, INF, INF), -Vec3(INF, INF, INF));
    }

    virtual bool intersect(const Light& l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }
    virtual bool intersect(const Light& l, Light& rl, Light& tl) const { return false; }
    virtual bool intersect(const Light& l, Vec3& p, Vec3& n) const {
        d_t temp = _norm.dot(l.norm);
        if ((temp < NZ) || (temp > PZ)) {
            d_t t = (_norm.dot(_p0 - l.p0)) / temp;
            if (t > PZ && t < l.length) {
                p = l.lightPoint(t); //plane side
                n = _norm;
                return true;
            }
        }
        return false;
    }

    Color getColor(const Vec3& p) const {
        if (!hasTexture) {
            return this->c;
        }
        Vec3 q = p - _p0;
        Vec3 w = _norm;
        Vec3 u = _textureXDir.normal();
        Vec3 v = w.cross(u);
        d_t x = u.dot(q);
        d_t y = v.dot(q);
        return Object::getColor(x / _textureHeight, y / _textureWidth);
    }
};

class TrianglePlane: public Plane {
private:
    Vec3 v[3];
    Color color[3];
    d_t uu[3];
    d_t vv[3];
    cv::Mat* p_texture;
public:
    TrianglePlane(const Vec3& v0, const Vec3& v1, const Vec3& v2, const Color& c0, const Color&c1, const Color& c2) : Plane((v0 - v1).cross(v0 - v2), v0) {
        this->v[0] = v0;
        this->v[1] = v1;
        this->v[2] = v2;
        this->color[0] = c0;
        this->color[1] = c1;
        this->color[2] = c2;
        this->p_texture = nullptr;
    }

    bool inface(const Vec3& p) const {
        d_t A0 = (v[1] - v[0]).cross(v[2] - v[0]).length();
        d_t S0 = (v[1] - p).cross(v[2] - p).length();
        d_t S1 = (v[2] - p).cross(v[0] - p).length();
        d_t S2 = (v[0] - p).cross(v[1] - p).length();
        if (S0 + S1 + S2 > A0) {
            return false;
        }
        else {
            return true;
        }
    }

    void load(const Json::Value &config) {
        assert((config["type"].asString() == "tr_plane") || config["type"].asString() == "mesh");
        this->name = config["name"].asString();
        Object::loadMaterial(config["material"]);
        computeAABB();
    }

    void addMeshTexture(d_t uu[], d_t vv[], cv::Mat *p_texture) {
        for (int i = 0; i < 3; ++i) {
            this->uu[i] = uu[i];
            this->vv[i] = vv[i];
        }
        if (p_texture == nullptr) {
            std::cout << "null" << std::endl;
        }
        this->p_texture = p_texture;
    }

    void computeAABB() {
        aabb.set(eachMax(v[0], eachMax(v[1], v[2])), eachMin(v[0], eachMin(v[1], v[2])));
    }

    virtual bool intersect(const Light& l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }
    virtual bool intersect(const Light& l, Light& rl, Light& tl) const { return false; }
    virtual bool intersect(const Light& l, Vec3& p, Vec3& n) const {
        Vec3 E1 = v[0] - v[1], E2 = v[0] - v[2], S = v[0] - l.p0;
        d_t t = det(S, E1, E2);
        d_t beta = det(l.norm, S, E2);
        d_t gamma = det(l.norm, E1, S);
        d_t m = det(l.norm, E1, E2);
        t /= m;
        beta /= m;
        gamma /= m;
        if (!((NZ <= beta) && (beta <= 1 + PZ) &&
              (NZ <= gamma) && (gamma <= 1 + PZ) && 
              (beta + gamma <= 1 + PZ)))
        {
            return false;
        }
        if (t < NZ) {
            return false;
        }
        p = l.lightPoint(t);
        n = _norm;
        return true;
    }

    Color getColor(const Vec3 &p) const {
        if (this->p_texture) {
            Vec3 va = v[1] - v[0], vb = v[2] - v[0], vc = p - v[0];
            d_t m = va.x * vb.y - va.y * vb.x, t1 = 0.0, t2 = 0.0;
            if ((m < NZ) || (m > PZ)) {
                t1 = (vc.x * vb.y - vc.y * vb.x) / m;
                t2 = (va.x * vc.y - va.y * vc.x) / m;
            }
            else {
                m = va.y * vb.z - va.z * vb.y;
                if ((m < NZ) || (m > PZ)) {
                    t1 = (vc.y * vb.z - vc.z * vb.y) / m;
                    t2 = (va.y * vc.z - va.z * vc.y) / m;
                }
                else {
                    m = va.x * vb.z - va.z * vb.x;
                    if ((m < NZ) || (m > PZ)) {
                        t1 = (vc.x * vb.z - vc.z * vb.x) / m;
                        t2 = (va.x * vc.z - va.z * vc.x) / m;
                    }
                    else {
                        t1 = t2 = 0.3;
                        assert(1 == 0);
                    }
                }
            }
            int x = int(p_texture->cols * (t1 * uu[1] + t2 * uu[2] + (1 - t1 - t2) * uu[0]));
            int y = int(p_texture->rows * (1 - (t1 * vv[1] + t2 * vv[2] + (1 - t1 - t2) * vv[0])));
            if ((x < p_texture->cols) && (y < p_texture->rows)) {
                Color co;
                co.x = p_texture->at<cv::Vec3b>(y, x)[0] / 255.0;
                co.y = p_texture->at<cv::Vec3b>(y, x)[1] / 255.0;
                co.z = p_texture->at<cv::Vec3b>(y, x)[2] / 255.0;
                return c * co;
            }
            else {
                Color co;
                co.x = p_texture->at<cv::Vec3b>(0, 0)[0] / 255.0;
                co.y = p_texture->at<cv::Vec3b>(0, 0)[1] / 255.0;
                co.z = p_texture->at<cv::Vec3b>(0, 0)[2] / 255.0;
                return c * co;
            }
        }
        return c * (color[0] + color[1] + color[2]) / 3;
    }
};

class DiskPlane : public Plane {
private:
    d_t r;

public:
    DiskPlane(const Vec3& norm = Vec3(), const Vec3& p0 = Vec3(), d_t r = 0) : Plane(norm, p0) {
        this->r = r;
    }

    bool inface(const Vec3& p) const {
        return p.squareDistance(this->_p0) < r * r;
    }

    void load(const Json::Value &config) {
        assert((config["type"].asString() == "disk_plane") || (config["type"].asString() == "source"));
        this->_p0 = jArr2Vec3(config["p"]);
        this->_norm = jArr2Vec3(config["n"]);
        this->r = config["r"].asDouble();
        this->name = config["name"].asString();
        Object::loadMaterial(config["material"]);
        computeAABB();
    }

    void computeAABB() {
        aabb.set(_p0 + r, _p0 - r);
    }

    virtual bool intersect(const Light& l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }
    virtual bool intersect(const Light& l, Light& rl, Light& tl) const { return false; }
    virtual bool intersect(const Light& l, Vec3& p, Vec3& n) const {
        bool b = Plane::intersect(l, p, n);
        if (b) {
            return inface(p);
        }
        else {
            return false;
        }
    }

    Color getColor(const Vec3 &p) const {
        return c;
    }

    Light generateLight(unsigned short *X) const {
        Vec3 w = _norm; //u, v, w, 坐标系
        Vec3 u = absd(w.x) > 0.1 ? w.cross(Vec3(0, 1, 0)).normal() : w.cross(Vec3(1, 0, 0)).normal();
        Vec3 v = w.cross(u);
        d_t alpha = erand48(X) * 2 * PI;
        Vec3 s = _p0 + r * (u * cos(alpha) + v * sin(alpha));
        Vec3 d = randDir(_norm, X);
        return Light(s + 0.001 * d, d);
    }
};

#endif //PLANE_H_