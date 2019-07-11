#include "object.h"

#include <json/json.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <assert.h>

#include "vec3.hpp"
#include "light.hpp"
#include "utils.h"

void Object::loadMaterial(const Json::Value& config) {
    this->c = jArr2Vec3(config["color"]);
    this->wr = config["wr"].asDouble();
    this->wt = config["wt"].asDouble();
    this->eda = config["eda"].asDouble();
    this->Ks = config["Ks"].asDouble();
    this->Kt = config["Kt"].asDouble();
    this->Kds = config["Kds"].asDouble();
    this->Kdt = config["Kdt"].asDouble();
    this->Ka = config["Ka"].asDouble();
    std::string textureFile = config["texture"].asString();
    loadTexture(textureFile);
}

void Object::loadTexture(const std::string& textureFile) {
    if (!textureFile.empty()) {
        std::cout << textureFile << std::endl;
        texture = cv::imread(textureFile, cv::IMREAD_COLOR);
        hasTexture = true;
    }
}

Object::~Object() {
    
}

Light Object::RL(const Vec3& p, const Vec3& norm, const Light& l) const {
    Vec3 n = l.norm - 2 * norm * (norm.dot(l.norm));
    return Light(p + n * 0.001, n);
}

Light Object::TL(const Vec3& p, const Vec3& norm, const Light& l, bool in, bool& valid, d_t& R) const {
    d_t newEda = in ? eda : 1.0 / eda;
    d_t cos_i = norm.dot(l.norm);
    Vec3 newNorm = cos_i > 0 ? norm : -norm;
    cos_i = absd(cos_i);
    Vec3 t = l.norm - newNorm * cos_i;
    d_t sin_i = t.length();
    if (sin_i > 1.0) {
        std::cout << sin_i << " " << norm.length() << " " << std::endl;
    }
    assert(sin_i <= 1.0);
    if (sin_i > newEda) {
        valid = false;
        return Light();
    }
    valid = true;
    Vec3 lt_n = 1.0 / newEda * (newNorm * sqrt(newEda * newEda - sin_i * sin_i) + t);

    //R
    d_t R0 = (1 - eda) / (1 + eda);
    R0 = R0 * R0;
    d_t a = 1 - cos_i;
    R = R0 + (1 - R0) * a * a * a * a * a;

    return Light(p + 0.001 * lt_n, lt_n);
}

Color Object::getColor(const Vec3& p) const {
    return c;
}

Color Object::getColor(int xx, int yy) const {
    if (!hasTexture) {
        return this->c;
    }
    if (xx < 0 || yy < 0 || xx >= texture.cols || yy >= texture.rows) {
        return this->c;
    }
    d_t r = texture.at<cv::Vec3b>(yy, xx)[0] / 255.0;
    d_t g = texture.at<cv::Vec3b>(yy, xx)[1] / 255.0;
    d_t b = texture.at<cv::Vec3b>(yy, xx)[2] / 255.0;
    return Color(r, g, b);
}

Color Object::getColor(d_t xx, d_t yy) const {
    if (!hasTexture) {
        return this->c;
    }
    if (xx < 0 || yy < 0 || xx > 1 || yy > 1) {
        return this->c;
    }
    int x = int(xx * (texture.cols - 1)), y = int(yy * (texture.rows - 1));
    assert((x >= 0) && (y >= 0));
    d_t r = texture.at<cv::Vec3b>(y, x)[0] / 255.0;
    d_t g = texture.at<cv::Vec3b>(y, x)[1] / 255.0;
    d_t b = texture.at<cv::Vec3b>(y, x)[2] / 255.0;
    return Color(r, g, b);
}