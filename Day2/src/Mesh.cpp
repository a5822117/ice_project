//
// Mesh.cpp
// OBJ mesh support with BVH acceleration
// + Y軸回転サポート追加
//
#define _USE_MATH_DEFINES

#include "Mesh.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>
#include <cmath>

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

//==============================================================================
// ★追加: Y軸周りの回転
// バウンディングボックスの中心を原点として回転
//==============================================================================
void Mesh::rotateY(double degrees) {
    // 度数法からラジアンに変換
    double radians = degrees * M_PI / 180.0;
    double cosA = std::cos(radians);
    double sinA = std::sin(radians);

    // バウンディングボックスの中心を回転の原点とする
    Eigen::Vector3d center = (bboxMin + bboxMax) / 2.0;

    std::cout << "Rotating mesh " << degrees << " degrees around Y axis" << std::endl;
    std::cout << "  Rotation center: " << center.transpose() << std::endl;

    // 頂点を回転
    for (auto &v : vertices) {
        Eigen::Vector3d local = v - center;
        double newX = local.x() * cosA + local.z() * sinA;
        double newZ = -local.x() * sinA + local.z() * cosA;
        v.x() = center.x() + newX;
        v.z() = center.z() + newZ;
    }

    // 三角形の頂点も回転し、法線を再計算
    for (auto &tri : triangles) {
        // v0
        Eigen::Vector3d local0 = tri.v0 - center;
        tri.v0.x() = center.x() + local0.x() * cosA + local0.z() * sinA;
        tri.v0.z() = center.z() - local0.x() * sinA + local0.z() * cosA;

        // v1
        Eigen::Vector3d local1 = tri.v1 - center;
        tri.v1.x() = center.x() + local1.x() * cosA + local1.z() * sinA;
        tri.v1.z() = center.z() - local1.x() * sinA + local1.z() * cosA;

        // v2
        Eigen::Vector3d local2 = tri.v2 - center;
        tri.v2.x() = center.x() + local2.x() * cosA + local2.z() * sinA;
        tri.v2.z() = center.z() - local2.x() * sinA + local2.z() * cosA;

        // 法線を再計算
        tri.normal = (tri.v1 - tri.v0).cross(tri.v2 - tri.v0).normalized();
    }

    updateBoundingBox();

    // 変換後はBVHを再構築
    buildBVH();

    std::cout << "  New bounding box:" << std::endl;
    std::cout << "    Min: " << bboxMin.transpose() << std::endl;
    std::cout << "    Max: " << bboxMax.transpose() << std::endl;
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