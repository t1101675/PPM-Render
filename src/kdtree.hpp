#ifndef KDTREE_H_
#define KDTREE_H_

#include <algorithm>
#include <iostream>
#include <vector>
#include <algorithm>

#include "vec3.hpp"
#include "utils.h"
#include "light.hpp"
#include "object.h"

struct Node {
    int vpIndex;
    int left;
    int right;
    Vec3 minBound;
    Vec3 maxBound;
    int dim;
    Node() {
        this->vpIndex = 0;
        this->left = 0;
        this->right = 0;
        this->minBound = Vec3();
        this->maxBound = Vec3();
        this->dim = 0;
    }
};


class Kdtree {
private:
    Node* nodes;
    int offset;
    ViewPoint* vpB;
    int vpNum;
public:
    int root;

    Kdtree() {
        offset = 1; //0 for null
        root = 0;
        nodes = new Node[MAX_KDNODE];
        this->vpB = nullptr;
        this->vpNum = -1;
    }

    ~Kdtree() {
        delete[] nodes;
    }

    int buildLayer(int* vpIndex, int l, int r, int dim, d_t radius[]) {
        if (l >= r) return 0;
        int mid = (l + r) >> 1;
        std::nth_element(vpIndex + l, vpIndex + mid, vpIndex + r, ViewPointCmp(dim, vpB));
        int index = offset;
        ++offset;
        int midVpIndex = vpIndex[mid];
        nodes[index].vpIndex = midVpIndex;
        nodes[index].dim = dim;
        nodes[index].maxBound = vpB[midVpIndex].p + radius[vpB[midVpIndex].index];
        nodes[index].minBound = vpB[midVpIndex].p - radius[vpB[midVpIndex].index]; 
        nodes[index].left = buildLayer(vpIndex, l, mid, (dim + 1) % 3, radius);
        if (nodes[index].left) {
            nodes[index].maxBound = eachMax(nodes[nodes[index].left].maxBound, nodes[index].maxBound);
            nodes[index].minBound = eachMin(nodes[nodes[index].left].minBound, nodes[index].minBound);
        }
        nodes[index].right = buildLayer(vpIndex, mid + 1, r, (dim + 1) % 3, radius);
        if (nodes[index].right) {
            nodes[index].maxBound = eachMax(nodes[nodes[index].right].maxBound, nodes[index].maxBound);
            nodes[index].minBound = eachMin(nodes[nodes[index].right].minBound, nodes[index].minBound);
        }
        return index;
    }

    void buildTree(ViewPoint* vpB, int* vpIndex, int vpNum, d_t radius[]) {
        this->vpB = vpB;
        this->vpNum = vpNum;
        root = buildLayer(vpIndex, 0, this->vpNum, 0, radius);
    }

    void query(int node, const Vec3 &pos, std::vector<ViewPoint *>& p_hit, d_t radius[]) const {
        if (pos.x > nodes[node].maxBound.x || pos.x < nodes[node].minBound.x ||
            pos.y > nodes[node].maxBound.y || pos.y < nodes[node].minBound.y ||
            pos.z > nodes[node].maxBound.z || pos.z < nodes[node].minBound.z)
        {
            return;
        }
        int vpIndex = nodes[node].vpIndex;
        d_t r = radius[vpB[vpIndex].index];
        if ((vpB[vpIndex].p - pos).squareLength() <= r * r)
            p_hit.push_back(&vpB[vpIndex]);
        
        if (nodes[node].left)
            query(nodes[node].left, pos, p_hit, radius);
       
        if (nodes[node].right)
            query(nodes[node].right, pos, p_hit, radius);
    }

    void clear() {
        offset = 1;
        root = 0;    
    }
};

struct ObjNode {
    int objIndex;
    int left;
    int right;
    int dim;
    AABB nodeAABB;
    ObjNode() {
        objIndex = 0;
        left = 0;
        right = 0;
        dim = 0;
    }
    bool inAABB(int objIndex, const Object *objs);
};

class ObjKdTree {
private:
    ObjNode* nodes;
    int offset;
    Object** objs;
    int objsNum;


public:
    int root;

    ObjKdTree() {
        this->nodes = new ObjNode[MAX_OBJNODE];
        this->offset = 1;
        this->objs = nullptr;
        this->objsNum = -1;
    }

    ~ObjKdTree() {
        delete[] nodes;
    }

    int buildLayer(int* objIndex, int l, int r, int dim) {
        if (l >= r) return 0;
        int mid = (l + r) >> 1;
        std::nth_element(objIndex + l, objIndex + mid, objIndex + r, ObjCmp(dim, objs));
        int nodeIndex = offset;
        ++offset;
        int midObjIndex = objIndex[mid];
        nodes[nodeIndex].objIndex = midObjIndex;
        nodes[nodeIndex].dim = dim;
        nodes[nodeIndex].nodeAABB.set(objs[midObjIndex]->aabb.maxBound, objs[midObjIndex]->aabb.minBound);

        nodes[nodeIndex].left = buildLayer(objIndex, l, mid, (dim + 1) % 3);
        if (nodes[nodeIndex].left) {
            nodes[nodeIndex].nodeAABB.set(eachMax(nodes[nodes[nodeIndex].left].nodeAABB.maxBound, nodes[nodeIndex].nodeAABB.maxBound),
                                            eachMin(nodes[nodes[nodeIndex].left].nodeAABB.minBound, nodes[nodeIndex].nodeAABB.minBound));
        }
        nodes[nodeIndex].right = buildLayer(objIndex, mid + 1, r, (dim + 1) % 3);
        if (nodes[nodeIndex].right) {
            nodes[nodeIndex].nodeAABB.set(eachMax(nodes[nodes[nodeIndex].right].nodeAABB.maxBound, nodes[nodeIndex].nodeAABB.maxBound), 
                                            eachMin(nodes[nodes[nodeIndex].right].nodeAABB.minBound, nodes[nodeIndex].nodeAABB.minBound));
        }
        return nodeIndex;
    }

    void buildTree(Object** objs, int *objIndex, int objNum) {
        this->objs = objs;
        this->objsNum = objNum;
        root = buildLayer(objIndex, 0, this->objsNum, 0);
    }

    void query(int node, const Light& l, std::vector<Object*>& obj_hit_aabb) const {
        if (!nodes[node].nodeAABB.intersect(l)) {
            return;
        }
        int objIndex = nodes[node].objIndex;
        if (objs[objIndex]->aabb.intersect(l)) {
            obj_hit_aabb.push_back(objs[objIndex]);
        }

        if (nodes[node].left) {
            query(nodes[node].left, l, obj_hit_aabb);
        }

        if (nodes[node].right)
            query(nodes[node].right, l, obj_hit_aabb);
    }

    void clear() {
        offset = 1;
        root = 0;
    }
};

#endif //KDTREE