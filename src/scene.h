#ifndef SCENE_H_
#define SCENE_H_

#include <iostream>
#include <vector>
#include <json/json.h>

#include "utils.h"
#include "object.h"
#include "vec3.hpp"
#include "kdtree.hpp"


struct Source {
    Color I;
    Vec3 p;
    bool fromJson; //only load from json should be deleted mannually
    Source(const Color& c = BLACK, const Vec3 &p = Vec3(0, 0, 0), bool fromJson = false) {
        this->I = c;
        this->p = p;
        this->fromJson = fromJson;
    }
};

class Scene {
public:

    Color Ia;
    std::vector<Object*> sources;
 
    std::vector<Object*> objs;
    ObjKdTree objKdTree;

    Scene(const Color& Ia = BLACK);
    ~Scene();

    void load(const Json::Value& config);
    void addObj(Object* obj);
    void addSource(Object* s);
    Light generateLight(unsigned short* X);

};

#endif //SCENE_H_