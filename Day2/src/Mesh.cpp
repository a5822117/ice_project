//
// Mesh.cpp
// OBJ mesh support with BVH acceleration
//

#include "Mesh.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

bool Mesh::loadOBJ(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return false;
    }

    vertices.clear();
    triangles.clear();
    bvhBuilt = false;

    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            // 頂点
            double x, y, z;
            iss >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        } else if (prefix == "f") {
            // 面（v/vt/vn形式に対応）
            std::vector<int> faceIndices;
            std::string vertex;
            while (iss >> vertex) {
                std::istringstream viss(vertex);
                std::string indexStr;
                std::getline(viss, indexStr, '/');
                int idx = std::stoi(indexStr) - 1; // OBJは1始まり
                faceIndices.push_back(idx);
            }

            // 三角形に分割（四角形以上にも対応）
            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                Triangle tri(
                    vertices[faceIndices[0]],
                    vertices[faceIndices[i]],
                    vertices[faceIndices[i + 1]]
                );
                triangles.push_back(tri);
            }
        }
    }

    file.close();
    updateBoundingBox();

    // BVHを構築
    buildBVH();

    std::cout << "Loaded OBJ: " << vertices.size() << " vertices, "
              << triangles.size() << " triangles" << std::endl;
    std::cout << "BVH built for mesh acceleration" << std::endl;

    return true;
}

void Mesh::buildBVH() {
    if (triangles.empty()) {
        bvhBuilt = false;
        return;
    }

    bvh.build(triangles);
    bvhBuilt = true;
}

bool Mesh::hit(const Ray &ray, RayHit &hit) const {
    // まずバウンディングボックスとの交差判定（高速な事前チェック）
    double tMin = 0.0, tMax = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; ++i) {
        double invD = 1.0 / ray.dir[i];
        double t0 = (bboxMin[i] - ray.org[i]) * invD;
        double t1 = (bboxMax[i] - ray.org[i]) * invD;
        if (invD < 0.0) std::swap(t0, t1);
        tMin = t0 > tMin ? t0 : tMin;
        tMax = t1 < tMax ? t1 : tMax;
        if (tMax <= tMin) return false;
    }

    // BVHを使用した高速な交差判定
    if (bvhBuilt && bvh.root) {
        return bvh.intersect(triangles, ray, hit);
    }

    // フォールバック：線形探索（BVHがない場合）
    hit.t = std::numeric_limits<double>::max();
    hit.idx = -1;
    bool hitAny = false;

    for (size_t i = 0; i < triangles.size(); ++i) {
        RayHit tempHit;
        if (triangles[i].hit(ray, tempHit) && tempHit.t < hit.t && tempHit.t > 1e-6) {
            hit = tempHit;
            hitAny = true;
        }
    }

    return hitAny;
}

void Mesh::translate(const Eigen::Vector3d &offset) {
    for (auto &v : vertices) {
        v += offset;
    }
    for (auto &tri : triangles) {
        tri.v0 += offset;
        tri.v1 += offset;
        tri.v2 += offset;
    }
    updateBoundingBox();

    // 変換後はBVHを再構築
    buildBVH();
}

void Mesh::scale(double s) {
    Eigen::Vector3d center = (bboxMin + bboxMax) / 2.0;
    for (auto &v : vertices) {
        v = center + (v - center) * s;
    }
    for (auto &tri : triangles) {
        tri.v0 = center + (tri.v0 - center) * s;
        tri.v1 = center + (tri.v1 - center) * s;
        tri.v2 = center + (tri.v2 - center) * s;
    }
    updateBoundingBox();

    // 変換後はBVHを再構築
    buildBVH();
}

void Mesh::updateBoundingBox() {
    if (vertices.empty()) return;
    bboxMin = vertices[0];
    bboxMax = vertices[0];
    for (const auto &v : vertices) {
        bboxMin = bboxMin.cwiseMin(v);
        bboxMax = bboxMax.cwiseMax(v);
    }
}

double Mesh::getMinY() const {
    return bboxMin.y();
}