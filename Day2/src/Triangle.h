//
// Created for OBJ mesh support
//

#ifndef DAY_2_TRIANGLE_H
#define DAY_2_TRIANGLE_H

#include "Ray.h"
#include <Eigen/Dense>

class Triangle {
public:
    Eigen::Vector3d v0, v1, v2;
    Eigen::Vector3d normal;

    Triangle() = default;
    Triangle(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

    bool hit(const Ray &ray, RayHit &hit) const;

    Eigen::Vector3d getCenter() const;
};

#endif //DAY_2_TRIANGLE_H