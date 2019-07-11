#ifndef RENDER_H_
#define RENDER_H_

#include <json/json.h>

#include "plane.hpp"
#include "object.h"
#include "scene.h"
#include "light.hpp"
#include "vec3.hpp"
#include "kdtree.hpp"

typedef Vec3 Color;

class Render {
private:
    Scene *_scene;
    int _M, _N;
    Vec3 _eyePos;
    Vec3 _viewP;
    Vec3 _viewN;
    d_t _focus;
    d_t _aperture;
    Color* tempResult;
    Color* fluxLightResult;

    int _sampleNum;
    int _maxIter;
    d_t _alpha;
    d_t _gamma;
    int _viewPointSamp;
    int* _vpIndex;

    Light getPrimLight(int x, int y, int sx, int sy, unsigned short* X);
    Light getPrimLight(int x, int y);
    Object* intersect(const Vec3& start, const Light& l, Vec3& p, Vec3& n);
    void pathTracing(const Vec3& start, const Light& l, d_t weight, bool inObject, Color& color, int depth);
    void ptRender();
    void sppmRender();
    Color volumeLight(const Light& l, const Vec3& sourcePos, const Color& lightFlux, double d);
    void addVolumeLight();
    void sppmRayTrace(const Vec3& start, const Light& l, int depth, int index, bool inObject, const Color& weight, unsigned short* X, std::vector<ViewPoint>& vpBuff);
    void sppmPhotonTrace(const Vec3 &start, const Light &l, int depth, const Kdtree &kdtree, bool inObject, const Color& weight, unsigned short* X);
    void updateViewPoint(const std::vector<ViewPoint*>& p_hit, const Color& weight);
    void initVpIndex(int vpNum);
    Color local(const Vec3 &p, const Vec3 &n, const Object *obj);

public:
    Render(int M = 100, int N = 100);
    ~Render();
    Color* result;
    d_t* radius;
    int* visitTime;
    int width() const { return _M; }
    int height() const { return _N; }
    void load(const Json::Value &config);
    void setObserve(const Vec3 &eyePos, const Vec3 &viewP, const Vec3 &viewN);
    void setScene(Scene * scene);
    void render();
    void save(const std::string& info);
};

static const d_t minWeight = 0.1;
static const int MAX_DEPTH = 10;
static const Color black(0, 0, 0);

#endif //RENDER_H_