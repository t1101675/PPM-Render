#include <json/json.h> 

#include "scene.h"
#include "utils.h"
#include "ball.hpp"
#include "plane.hpp"
#include "b_obj.hpp"
#include "sub_ball.hpp"
#include "mesh_obj.hpp"


Scene::Scene(const Color& Ia) {
    this->Ia = Ia;
}

Scene::~Scene() {
    for (int i = 0; i < objs.size(); ++i) {
        if (objs[i]->fromJson) {
            delete objs[i];
        }
    }
}

void Scene::addObj(Object* obj) {
    objs.push_back(obj);
}

void Scene::load(const Json::Value& config) {
    this->Ia = jArr2Vec3(config["eI"]);
    Json::Value sourcesConf = config["sources"];
    for (int i = 0; i < sourcesConf.size(); ++i) {
        DiskPlane* s = new DiskPlane();  
        s->load(sourcesConf[i]);
        s->fromJson = true;    
        this->addSource(s);
    }
    Json::Value objsConf = config["objs"];
    for (int i = 0; i < objsConf.size(); ++i) {
        std::string type = objsConf[i]["type"].asString();
        if (type == "ball") {
            Ball* ball = new Ball();
            ball->load(objsConf[i]);
            ball->fromJson = true;
            this->addObj(ball);
        }
        else if (type == "sub_ball") {
            SubBall* subBall = new SubBall();
            subBall->load(objsConf[i]);
            subBall->fromJson = true;
            this->addObj(subBall);
        }
        else if (type == "plane") {
            Plane* plane = new Plane();
            plane->load(objsConf[i]);
            plane->fromJson = true;
            this->addObj(plane);
        }
        else if (type == "bezier") {
            BezierObj* bObj = new BezierObj();
            bObj->load(objsConf[i]);
            bObj->fromJson = true;
            this->addObj(bObj);
        }
        else if (type == "mesh") {
            MeshObj* meshObj = new MeshObj();
            meshObj->load(objsConf[i], objs);
            meshObj->fromJson = true;
            this->addObj(meshObj);
        }
        else {
            std::cout << "invalid type: " << type << std::endl;
        }
    }
    int* objIndices = new int[objs.size()];
    for (int i = 0; i < objs.size(); ++i) {
        objIndices[i] = i;
    }
    std::cout << objs.size() << std::endl;
    objKdTree.buildTree(objs.data(), objIndices, objs.size());
    std::cout << "[info] Build object kdtree complete" << std::endl;
    delete[] objIndices;
}

void Scene::addSource(Object* s) {
    sources.push_back(s);
    objs.push_back(s);
}

Light Scene::generateLight(unsigned short* X) {
    int r = int(erand48(X) * sources.size());
    Object* source = sources[r];
    return source->generateLight(X);
}

