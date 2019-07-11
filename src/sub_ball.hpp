#ifndef _SUB_BALL
#define _SUB_BALL

#include <assert.h>

#include "ball.hpp"

class SubBall: public Ball {
private: 
    Vec3 _dir;
    d_t _cos_theta;
    int _texture_x;
    int _texture_y;
    d_t _texture_r;
public:
    SubBall(Vec3 c = Vec3(), d_t r = 0.0, Vec3 dir = Vec3(), d_t cos_theta = 1.0, const Color &co = BLACK, d_t wr = 1.0, d_t wt = 0.0, d_t eda = 0.5): Ball(c, r, co, wr, wt, eda) {
        this->_dir = dir;
        this->_cos_theta = cos_theta;
        this->_texture_r = 0.0;
        this->_texture_x = 0;
        this->_texture_y = 0;
    }

    void load(const Json::Value &config) {
        assert(config["type"].asString() == "sub_ball");
        this->_dir = jArr2Vec3(config["dir"]).normal();
        this->_cos_theta = config["cos_theta"].asDouble();
        this->_texture_x = config["material"]["texture_x"].asInt();
        this->_texture_y = config["material"]["texture_y"].asInt();
        this->_texture_r = config["material"]["texture_r"].asDouble();
        Ball::load(config);
    }
    bool intersect(const Light &l, Light &rl, Light &tl) const { return false; }
    bool intersect(const Light &l) const {
        Vec3 p, n;
        return this->intersect(l, p, n);
    }
    bool intersect(const Light &l, Vec3 &p, Vec3 &n) const {
        Vec3 pp, nn;
        bool inter = Ball::intersect(l, pp, nn);
        if (inter) {
            d_t cos_alpha = _dir.dot(nn);
            if (cos_alpha > _cos_theta) {
                p = pp;
                n = nn;
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    virtual Color getColor(const Vec3& p) const {
        Vec3 n = norm(p);
        d_t cos_alpha = n.dot(_dir);
        d_t alpha = acos(cos_alpha);
        d_t theta = acos(_cos_theta);
        d_t r = alpha / theta * _texture_r;
        Vec3 w = _dir;
        Vec3 u = absd(w.x) > 0.1 ? w.cross(Vec3(0, 1, 0)).normal() : w.cross(Vec3(1, 0, 0)).normal();
        Vec3 v = _dir.cross(n).normal();
        d_t cos_phi = u.dot(v);
        d_t sin_phi = u.cross(v).dot(w);
        int x = _texture_x + int(r * cos_phi);
        int y = _texture_y + int(r * sin_phi);
        return Object::getColor(x, y);
    }
};
#endif