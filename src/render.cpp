#include <assert.h>
#include <json/json.h>
#include <memory.h>
#include "kdtree.hpp"

#include "render.h"
#include "utils.h"

Render::Render(int M, int N) {
    this->_scene = nullptr;
    this->_viewP = Vec3();
    this->_viewN = Vec3();
    this->_eyePos = Vec3();
    this->_M = M;
    this->_N = N;
    this->_maxIter = 0;
    this->_sampleNum = 0;
    this->_viewPointSamp = 0;
    this->_alpha = 0.;
    this->_gamma = 0.5;
    this->_aperture = 0;
    this->_focus = 0;

    this->result = new Color[M * N];
    this->tempResult = new Color[M * N];
    this->fluxLightResult = new Color[M * N];
    this->radius = new d_t[M * N];
    this->visitTime = new int[M * N];
    this->_vpIndex = new int[MAX_KDNODE];
}

Render::~Render() {
    delete[] this->result;
    delete[] this->tempResult;
    delete[] this->radius;
    delete[] this->visitTime;
    delete[] this->_vpIndex;
}

void Render::load(const Json::Value& config) {
    this->_M = config["width"].asInt();
    this->_N = config["height"].asInt();
    this->_viewP = jArr2Vec3(config["view_p"]);
    this->_viewN = jArr2Vec3(config["view_n"]);
    this->_eyePos = jArr2Vec3(config["eye_pos"]);
    this->_sampleNum = config["sample_num"].asInt();
    this->_maxIter = config["max_iter"].asInt();
    this->_alpha = config["alpha"].asDouble();
    this->_gamma = config["gamma"].asDouble();
    this->_viewPointSamp = config["vp_sample"].asInt();
    this->_aperture = config["aperture"].asDouble();
    this->_focus = config["focus"].asDouble();
    if (this->result) {
        delete[] result;
    }
    result = new Color[this->_M * this->_N];
    if (this->tempResult) {
        delete[] tempResult;
    }
    tempResult = new Color[this->_M * this->_N];
    if (this->fluxLightResult) {
        delete[] fluxLightResult;
    }
    fluxLightResult = new Color[this->_M * this->_N];
    if (this->radius) {
        delete[] radius;
    }
    radius = new d_t[this->_M * this->_N];
    if (this->visitTime) {
        delete[] visitTime;
    }
    visitTime = new int[this->_M * this->_N];
}

void Render::save(const std::string& info) {
    cv::Mat image(height(), width(), CV_64FC3, cv::Scalar(0, 0, 0));
    for (int x = 0; x < width(); ++x) {
        for (int y = 0; y < height(); ++y) {
        image.at<cv::Vec3d>(y, x)[0] = result[x * height() + y].x;
        image.at<cv::Vec3d>(y, x)[1] = result[x * height() + y].y;
        image.at<cv::Vec3d>(y, x)[2] = result[x * height() + y].z;
        }
    }
    cv::Mat new_image;
    image.convertTo(new_image, CV_8UC3, 255);
    cv::imwrite("result/" + info + "_test.png", new_image);
}

void Render::setObserve(const Vec3& eyePos, const Vec3& viewP, const Vec3& viewN) {
    _eyePos = eyePos;
    _viewP = viewP;
    _viewN = viewN;
}

void Render::setScene(Scene *scene) {
    this->_scene = scene;
}

void Render::render() {
    sppmRender();
}

void Render::ptRender() {
    int k = 0;
    for (int x = 0; x < _M; ++x) {
        for (int y = 0; y < _N; ++y) {
            Light primL = getPrimLight(x, y);
            Color color;
            pathTracing(_eyePos, primL, 1.0, false, color, 0);
            result[x * _N + y] = color;
            if (k % 1000 == 0) {
                std::cout << k << std::endl;
            }
            k++;
        }
    }
} 

Light Render::getPrimLight(int x, int y, int sx, int sy, unsigned short* X) {
    d_t r1 = 2 * erand48(X), dx = r1 < 1 ? sqrt(r1) : 2 - sqrt(2 - r1);
    d_t r2 = 2 * erand48(X), dy = r2 < 1 ? sqrt(r2) : 2 - sqrt(2 - r2);

    Vec3 P(_viewP.x, _viewP.y + (d_t(x + sx + dx / 2) / d_t(_M) - 0.5) * d_t(_M) / d_t(_N) * pix2coor, _viewP.z - (d_t(y + sy + dy / 2) / d_t(_N) - 0.5) * pix2coor);
    Light l = Light(P, (P - _eyePos));
    Vec3 hitP, hitN;
    Plane focusPlane(Vec3(-1, 0, 0), Vec3(_eyePos.x - _focus, 0, 0));
    focusPlane.intersect(l, hitP, hitN);
    d_t theta = erand48(X) * 2 * PI;
    l.p0 += Vec3(0, cos(theta), sin(theta)) * _aperture;
    l.norm = (hitP - l.p0).normal();
    return l;
}

Light Render::getPrimLight(int x, int y) {
    Vec3 P(_viewP.x, _viewP.y + (d_t(x) / d_t(_M) - 0.5) * pix2coor, _viewP.z - (d_t(y) / d_t(_N) - 0.5) * pix2coor);
    return Light(P, (P - _eyePos));
}

Object* Render::intersect(const Vec3& start, const Light &l, Vec3 &p, Vec3 &n) {
    d_t minDistance = INF;
    Object* closestObj = nullptr;
    std::vector<Object*> obj_hit_aabb;
    _scene->objKdTree.query(_scene->objKdTree.root, l, obj_hit_aabb);
    for (auto p_obj :  obj_hit_aabb) {
        Vec3 hitP, hitN;
        int res = p_obj->intersect(l, hitP, hitN);
        if (res) {
            d_t d = start.distance(hitP);
            if (d < minDistance) {
                closestObj = p_obj;
                minDistance = d;
                p = hitP;
                n = hitN;
            }
        }
    }
    return closestObj;
}

void Render::pathTracing(const Vec3& start, const Light& l, d_t weight, bool inObject, Color& color, int depth) {
    if (depth > MAX_DEPTH) {
        color = black;
        return;
    }
    if (weight < minWeight) {
        color = black;
        return;
    }
    
    Object* closestObj = nullptr;
    Vec3 closestHitP, closestHitN;
    closestObj = intersect(start, l, closestHitP, closestHitN);
    if (!closestObj) {
        color = black;
        return;
    }
    Color cLocal = local(closestHitP, closestHitN, closestObj);
    Light rl = closestObj->RL(closestHitP, closestHitN, l);
    Color cr(0, 0, 0);
    pathTracing(closestHitP, rl, weight * closestObj->wr, inObject, cr, depth + 1);
    bool valid = false; //全反射
    d_t R = 0.;
    Light tl = closestObj->TL(closestHitP, closestHitN, l, !inObject, valid, R);
    Color tr(0, 0, 0);  
    if (valid) {
        pathTracing(closestHitP, tl, weight * closestObj->wt, !inObject, tr, depth + 1);
    }
    color = cLocal + closestObj->wr * cr + closestObj->wt * tr;
}

Color Render::local(const Vec3& p, const Vec3& n, const Object* obj) {
    Color eI = obj->c * _scene->Ia * obj->Ka; //环境光
    Color rI, tI;
    // for (auto p_source : _scene->sources) {
        // d_t length = distance(p, p_source->p);
        // Vec3 L = (p_source->p - p) / length;
        // bool shadow = false;
        // for (auto p_obj : _scene->objs) {
            // if (p_obj->intersect(Light(p, L, length))) {
                // shadow = true;
                // break;
            // }
        // }
        // if (!shadow) {
            // Vec3 R = obj->RL(p, n, Light(p, L)).norm; 
            // d_t diffuse = obj->Kds * abs(L.dot(n)); //漫反射
            // d_t specular = obj->Ks * pow(R.dot(n), 80); //高光
            // rI += specular * p_source->I + diffuse * p_source->I * obj->getColor(p);
        // }
    // }
    return eI + rI + tI;
}

void Render::initVpIndex(int vpNum) {
    for (int i = 0; i < vpNum; ++i) {
        _vpIndex[i] = i;
    }
}

void Render::addVolumeLight() {
    Color* vLight = new Color[_M * _N];
    int vLightSampTime = 100;
    for (int x = 0; x < _M; ++x) {
        for (int y = 0; y < _N; ++y) {
            unsigned short X[3] = {y, y * x * time(0), y * x * y + time(0)};
            Light primL = getPrimLight(x, y, 0, 0, X);
            Object *closestObj = nullptr;
            Vec3 closestHitP, closestHitN;
            closestObj = intersect(primL.p0, primL, closestHitP, closestHitN);
            if (closestObj == nullptr) {
                continue;
            }
            else {
                double d = closestHitP.distance(primL.p0);
                for (int samp = 0; samp < vLightSampTime; ++samp) {
                    int r = int(erand48(X) * _scene->sources.size());
                    Object* source = _scene->sources[r];
                    Light tempL = source->generateLight(X);
                    vLight[x * _N + y] = volumeLight(primL, tempL.p0, Color(1000.0, 1000.0, 1000.0), d);
                }
            }
        }
    }
    for (int index = 0; index < _M * _N; ++index) {
        result[index] += vLight[index] / vLightSampTime;
    }
    delete[] vLight;
}

Color Render::volumeLight(const Light& l, const Vec3& sourcePos, const Color& lightFlux, double d) {
    Vec3 q = l.p0 - sourcePos;
    d_t b = l.norm.dot(q);
    d_t c = q.squareLength();
    d_t iv = 1.0 / sqrt(c - b * b);
    d_t rate = iv * (atan((d + b) * iv) - atan(b * iv));
    return rate * lightFlux;
}

void Render::sppmRender() {
    Kdtree kdtree;
    Color initWeight = Color(2.5, 2.5, 2.5);
    Color light = Color(4.0, 4.0, 4.0);
    int nth = omp_get_num_procs();
    for (int i = 0; i < _M * _N; ++i) { radius[i] = INIT_R; }
    for (int i = 0; i < _M * _N; ++i) { visitTime[i] = 1; }
    for (int iter = 0; iter < _maxIter; ++iter) {
        std::cout << "[info] iter: " << iter << std::endl;
        std::vector<ViewPoint> totVp;
        std::vector<ViewPoint> vpBuff[nth];

#pragma omp parallel for
        for (int x = 0; x < _M; ++x) {
            int num = omp_get_thread_num();
            for (int y = 0; y < _N; ++y) {
                for (int sx = 0; sx < 2; ++sx) {
                    for (int sy = 0; sy < 2; ++sy) {
                        unsigned short X[3] = {y + sy, y * x * time(0) + sx, y * x * y + time(0) + sy * 2 + sx};
                        Light primL = getPrimLight(x, y, sx, sy, X);
                        sppmRayTrace(primL.p0, primL, 0, x * _N + y, false, Color(1.0, 1.0, 1.0), X, vpBuff[num]);
                    }
                }
            }
        }
        for (int i = 0; i < nth; ++i) {
            totVp.insert(totVp.end(), vpBuff[i].begin(), vpBuff[i].end());
        }
        initVpIndex(totVp.size());
        kdtree.buildTree(totVp.data(), _vpIndex, totVp.size(), radius);

        #pragma omp parallel for schedule(dynamic) 
        for (int samp = 0; samp < _sampleNum; ++samp) {
            unsigned short X[3] = {samp, samp * samp, (samp & (samp * samp)) + iter + time(0)};
            Light l = _scene->generateLight(X);
            sppmPhotonTrace(l.p0, l, 0, kdtree, false, initWeight * light, X);
        }
        kdtree.clear();
        if ((iter + 1) % 500 == 0) {
            for (int index = 0; index < _M * _N; ++index) {
                result[index] = tempResult[index] / (PI * radius[index] * radius[index] * _sampleNum * iter) * 1500 + fluxLightResult[index] / iter;
                result[index] = doGamma(result[index], _gamma);
            }
            save(std::to_string(iter + 1));
        }
    }
    for (int index = 0; index < _M * _N; ++index) {
        result[index] = tempResult[index] / (PI * radius[index] * radius[index] * _sampleNum * _maxIter) * 1000 + fluxLightResult[index] / _maxIter;
        result[index] = doGamma(result[index], _gamma);
    }
}

void Render::sppmRayTrace(const Vec3& start, const Light& l, int depth, int index, bool inObject, const Color& weight, unsigned short* X, std::vector<ViewPoint>& vpBuff) {
    if (depth > MAX_DEPTH) {
        return;
    }
    Object *closestObj = nullptr;
    Vec3 closestHitP, closestHitN;
    closestObj = intersect(start, l, closestHitP, closestHitN);
    if (closestObj == nullptr) {
        return;
    }
    
    d_t s = closestObj->Kds + closestObj->Ks + closestObj->wt;
    d_t ss = closestObj->Kds + closestObj->Ks;
    d_t choice = erand48(X) * ss;
    //diffuse 漫反射面
    if ((closestObj->Kds > PZ) && (choice < closestObj->Kds)) {
        if (closestObj->name == "source") {
            if (l.norm.dot(closestHitN) < 0) {
                fluxLightResult[index] += closestObj->getColor(closestHitP);
                vpBuff.push_back(ViewPoint(closestHitP, l.norm, closestObj->getColor(closestHitP) * weight * s, index));
            }
        }
        else {
            vpBuff.push_back(ViewPoint(closestHitP, l.norm, closestObj->getColor(closestHitP) * weight * s, index));
        }
        return;
    }
    
    //specular 镜面反射面
    if ((closestObj->Ks > PZ) && (closestObj->Kds <= choice)) {
        Light rl = closestObj->RL(closestHitP, closestHitN, l);
        sppmRayTrace(closestHitP, rl, depth + 1, index, inObject, closestObj->getColor(closestHitP) * weight * s, X, vpBuff);
        return;
    }
    
    //refraction 透射面，同时还有反射
    if (closestObj->wt > PZ) {
        bool valid = false; //全反射
        d_t R = 0;
        Light tl = closestObj->TL(closestHitP, closestHitN, l, !inObject, valid, R);
        Light rl = closestObj->RL(closestHitP, closestHitN, l);
        if (valid) {
            if (erand48(X) < R) {
                sppmRayTrace(closestHitP, rl, depth + 1, index, inObject, closestObj->getColor(closestHitP) * weight * s, X, vpBuff);
            }
            else {
                sppmRayTrace(closestHitP, tl, depth + 1, index, !inObject, closestObj->getColor(closestHitP) * weight * s, X, vpBuff);
            }
        }
        else {
            sppmRayTrace(closestHitP, rl, depth + 1, index, inObject, closestObj->getColor(closestHitP) * weight * s, X, vpBuff);
        }
        return;
    }
}

void Render::sppmPhotonTrace(const Vec3& start, const Light& l, int depth, const Kdtree& kdtree, bool inObject, const Color& weight, unsigned short* X) {
    if (depth > MAX_DEPTH) {
        return;
    }
    Object *closestObj = nullptr;
    Vec3 closestHitP, closestHitN;
    closestObj = intersect(start, l, closestHitP, closestHitN);
    if (closestObj == nullptr) {
        return;
    }

    d_t s = closestObj->Kds + closestObj->Ks + closestObj->wt;

    //diffuse 漫反射面
    if (closestObj->Kds > PZ) {
        std::vector<ViewPoint*> p_hit;
        kdtree.query(kdtree.root, closestHitP, p_hit, radius);
        updateViewPoint(p_hit, weight);
        Vec3 diffuse_dir = randDir(closestHitN, X);
        sppmPhotonTrace(closestHitP, Light(closestHitP + 0.001 * diffuse_dir, diffuse_dir), depth + 1, kdtree, inObject, closestObj->getColor(closestHitP) * weight * s, X);
        return;
    }
    
    //speclar 镜面反射面
    if (closestObj->Ks > PZ) {
        Light rl = closestObj->RL(closestHitP, closestHitN, l);
        sppmPhotonTrace(closestHitP, rl, depth + 1, kdtree, inObject, closestObj->getColor(closestHitP) * weight * s, X);
        return;
    }

    //refraction 透射面，同时还有反射
    if (closestObj->wt > PZ) {
        std::vector<ViewPoint *> p_hit;
        kdtree.query(kdtree.root, closestHitP, p_hit, radius);
        bool valid = false; //全反射
        d_t R = 0;
        Light tl = closestObj->TL(closestHitP, closestHitN, l, !inObject, valid, R);
        Light rl = closestObj->RL(closestHitP, closestHitN, l);
        if (valid) {
            if (erand48(X) < R) {
                sppmPhotonTrace(closestHitP, rl, depth + 1, kdtree, inObject, closestObj->getColor(closestHitP) * weight * s, X);
            }
            else {
                sppmPhotonTrace(closestHitP, tl, depth + 1, kdtree, !inObject, closestObj->getColor(closestHitP) * weight * s, X);
            }
        }
        else {
            sppmPhotonTrace(closestHitP, rl, depth + 1, kdtree, inObject, closestObj->getColor(closestHitP) * weight * s, X);
        }
    }
}

void Render::updateViewPoint(const std::vector<ViewPoint*>& p_hit, const Color& weight) {
    for (auto p_vp : p_hit) {
        d_t factor = (visitTime[p_vp->index] * _alpha + _alpha) / (visitTime[p_vp->index] * _alpha + 1.);
        tempResult[p_vp->index] += p_vp->weight * weight * factor;
        radius[p_vp->index] *= factor;
        visitTime[p_vp->index] += 1;
    }
}

