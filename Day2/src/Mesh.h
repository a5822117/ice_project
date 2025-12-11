//
// Created by MitaniRyota on 2025/12/11.
//

#ifndef MESH_H
#define MESH_H

#endif //MESH_H
//
// Created for OBJ mesh support
//

#ifndef DAY_2_MESH_H
#define DAY_2_MESH_H

#include "Triangle.h"
#include "Ray.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

class Mesh {
public:
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Triangle> triangles;

    // バウンディングボックス
    Eigen::Vector3d bboxMin, bboxMax;

    Mesh() = default;

    bool loadOBJ(const std::string &filename);
    bool hit(const Ray &ray, RayHit &hit) const;
    void translate(const Eigen::Vector3d &offset);
    void scale(double scale);
    void updateBoundingBox();
    double getMinY() const;
};

#endif //DAY_2_MESH_H