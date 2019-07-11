#ifndef MESH_OBJ_H_
#define MESH_OBJ_H_

#include <string>
#include <fstream>
#include <sstream>

#include "object.h"
#include "plane.hpp"

typedef TrianglePlane Face;

struct Vertex {
    Vec3 p;
    Color c;
    Vertex(const Vec3& pp, const Color& cc) : p(pp), c(cc) { }
};

class MeshObj : public Object {
private:
    std::vector<Vertex> vertices;
    std::vector<Face*> faces;
    int vNum;
    int fNum;
    Vec3 p;
    std::string meshPath;
    std::string textureDir;
    std::vector<cv::Mat> textures;

public:
    MeshObj() {
        vNum = 0;
        fNum = 0;
    }

    void loadPly(const std::string& path, const std::string& textureDir) {
        std::ifstream fin(path);
        if (!fin) {
            std::cout << path << " : no such file" << std::endl;
            return;
        }
        std::istringstream istr;
        std::string line, buff, first, second;
        while (first != "end_header") {
            getline(fin, line);
            istr.str(line);
            istr >> first;
            if (first == "comment") {
                istr >> second;
                if (second == "TextureFile") {
                    std::string filename;
                    istr >> filename;
                    std::stringstream parser(filename);
                    std::string token;
                    while (getline(parser, token, '\\')) { }
                    std::cout << token << std::endl;
                    cv::Mat image = cv::imread(textureDir + "/" + token, cv::IMREAD_COLOR);
                    textures.push_back(image);
                }
            }
            else if (first == "element") {
                istr >> second;
                if (second == "vertex") {
                    istr >> vNum;
                }
                else if (second == "face") {
                    istr >> fNum;
                }
            }
            istr.clear();
        }
        std::cout << vNum << " " << fNum << std::endl;

        for (int v = 0; v < vNum; ++v) {
            getline(fin, line);
            istr.str(line);
            Vec3 pp;
            int c[3];
            istr >> pp.x >> pp.y >> pp.z >> c[0] >> c[1] >> c[2];
            istr.clear();
            vertices.push_back(Vertex(pp + p, Color(c[2] / 255.0, c[1] / 255.0, c[0] / 255.0)));
        }

        for (int f = 0; f < fNum; ++f) {
            getline(fin, line);
            istr.str(line);
            int v[3];
            istr >> buff >> v[0] >> v[1] >> v[2];
            Face* face = new Face(vertices[v[0]].p, vertices[v[1]].p, vertices[v[2]].p, 
                                vertices[v[0]].c, vertices[v[1]].c, vertices[v[2]].c);
            if (!textures.empty()) {
                istr >> buff;
                d_t uu[3], vv[3];
                for (int i = 0; i < 3; ++i) {
                    istr >> uu[i] >> vv[i];
                    if (uu[i] < 0) uu[i] = -uu[i];
                    if (vv[i] < 0) vv[i] = -vv[i];
                }
                if (textures.size() > 1) {
                    int textureIndex;
                    istr >> textureIndex;
                    if (textureIndex >= 0) {
                        face->addMeshTexture(uu, vv, &textures[textureIndex]);
                    }
                    else {
                        std::cout << textureIndex << std::endl;
                    }
                }
                else {
                    face->addMeshTexture(uu, vv, &textures[0]);
                }
            }
            istr.clear();
            faces.push_back(face);
        }
        fin.close();
    }

    void load(const Json::Value& config, std::vector<Object*>& objs) {
        assert((config["type"].asString() == "mesh"));
        this->name = config["name"].asString();
        this->meshPath = config["mesh_path"].asString();
        this->textureDir = config["texture_dir"].asString();
        this->p = jArr2Vec3(config["p"]);
        loadPly(this->meshPath, this->textureDir);
        for (auto face : faces) {
            face->load(config);
            face->fromJson = true;
        }
        objs.insert(objs.end(), faces.begin(), faces.end());
    }

    void computeAABB() {
        aabb.set(Vec3(), Vec3());
    }

    virtual bool intersect(const Light &l) const {
        Vec3 p, n;
        return intersect(l, p, n);
    }

    virtual bool intersect(const Light &l, Light &rl, Light &tl) const { return false; }
    virtual bool intersect(const Light &l, Vec3 &p, Vec3 &n) const { return false; }
};

#endif