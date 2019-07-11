#ifndef BALL_H_
#define BALL_H_

#include <assert.h>

#include "object.h"
#include "utils.h"

class Ball: public Object {
private:
    Vec3 _c;
    d_t _r;
    
public:
    Ball(Vec3 c = Vec3(), d_t r = 0.0, const Color &co = BLACK, d_t wr = 1.0, d_t wt = 0.0, d_t eda = 0.5): Object(wr, wt, eda, co) {
        this->_c = c;
        this->_r = r;
    }

    virtual void load(const Json::Value& config) {
        assert((config["type"].asString() == "ball") || (config["type"].asString() == "sub_ball"));
        this->name = config["name"].asString();
        this->_c = jArr2Vec3(config["c"]);
        this->_r = config["r"].asDouble();
        Object::loadMaterial(config["material"]);
        computeAABB();
    }

    void computeAABB() {
        aabb.set(_c + _r, _c - _r);
    }

    virtual bool intersect(const Light& l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }
    virtual bool intersect(const Light& l, Light& rl, Light& tl) const { return false; }
    virtual bool intersect(const Light& l, Vec3& p, Vec3& n) const {
        Vec3 R0 = l.p0;
        Vec3 Rl = l.norm;
        d_t Loc2 = squareDistance(R0, _c);
        d_t tac = Rl.dot(_c - R0);
        d_t R2 = _r * _r;
        if ((Loc2 - R2) > PZ) { //Loc2 > R2
            if (tac > PZ) { //tac > 0, maybe yes
                d_t thc2 = R2 - Loc2 + tac * tac;
                if (thc2 < NZ) {//thc2 < 0, no
                    return false;
                }
                else {//yes(intersect or tangent)
                    d_t t = tac - sqrt(thc2 - NZ);
                    if (t < l.length) {
                        p = l.lightPoint(t);
                        n = norm(p).normal();
                        return true;
                    } 
                    else {
                        return false;
                    }  
                }
            }
            else { //tac <=0 false
                return false;
            }
        }
        else if ((Loc2 - R2) < NZ) { //Loc2 < R2
            //in ball
            d_t thc2 = R2 - Loc2 + tac * tac;
            d_t t = sqrt(thc2) + tac;
            if (t < l.length) {
                p = l.lightPoint(t);
                n = norm(p).normal();
                return true;
            }
            else {
                return false;
            }
        }
        else { //Loc2 == R2 on the ball
            if (tac > PZ) {
                d_t t = tac + tac;
                if (t < l.length) {
                    p = l.lightPoint(t);
                    n = norm(p).normal();
                    return true;
                }
                else {
                    return false;
                }
            }
        }
        return false;
    }

    Vec3 norm(const Vec3& v) const { return Vec3(v - _c) / _r; }
};

#endif //BALL_H_