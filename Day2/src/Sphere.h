//
// Created by kango on 2023/03/22.
//

#ifndef DAY_2_SPHERE_H
#define DAY_2_SPHERE_H


#include "Eigen/Dense"
#include "Ray.h"

class Sphere {

public:
    double radius{};
    Eigen::Vector3d center;

    Sphere() = default;
    Sphere(double radius, Eigen::Vector3d center);
    bool hit(const Ray &ray, RayHit &hit) const;
};


#endif //DAY_2_SPHERE_H