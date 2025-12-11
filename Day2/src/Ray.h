//
// Created by kango on 2023/03/22.
//

#ifndef DAY_2_RAY_H
#define DAY_2_RAY_H

#include <utility>
#include <iostream>

#include "Eigen/Dense"

struct Ray {
    Eigen::Vector3d org;
    Eigen::Vector3d dir;

    Ray() = default;

    Ray(const Eigen::Vector3d &org, const Eigen::Vector3d &dir) : org(org), dir(dir.normalized()) {}

    Eigen::Vector3d at(const double &t) const {
        return org + t * dir;
    }
};

struct RayHit {
    int idx = -1;
    double t = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    RayHit() = default;

    bool isHit() const {
        return idx != -1;
    }

    void show() const {
        std::cout << "idx:\t" << idx << std::endl;
        std::cout << "t:\t" << t << std::endl;
        std::cout << "point:\t" << point.transpose() << std::endl;
        std::cout << "normal:\t" << normal.transpose() << std::endl;
    }
};

#endif //DAY_2_RAY_H