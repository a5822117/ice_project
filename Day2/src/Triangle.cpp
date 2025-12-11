//
// Created for OBJ mesh support
//

#include "Triangle.h"

Triangle::Triangle(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    : v0(v0), v1(v1), v2(v2) {
    // 法線を計算（反時計回りを表とする）
    normal = (v1 - v0).cross(v2 - v0).normalized();
}

bool Triangle::hit(const Ray &ray, RayHit &hit) const {
    // Möller–Trumbore intersection algorithm
    const double EPSILON = 1e-8;

    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d h = ray.dir.cross(edge2);
    double a = edge1.dot(h);

    // レイが三角形と平行な場合
    if (a > -EPSILON && a < EPSILON)
        return false;

    double f = 1.0 / a;
    Eigen::Vector3d s = ray.org - v0;
    double u = f * s.dot(h);

    if (u < 0.0 || u > 1.0)
        return false;

    Eigen::Vector3d q = s.cross(edge1);
    double v = f * ray.dir.dot(q);

    if (v < 0.0 || u + v > 1.0)
        return false;

    double t = f * edge2.dot(q);

    if (t > EPSILON) {
        hit.t = t;
        hit.point = ray.at(t);
        // レイの方向と法線の向きを考慮
        if (normal.dot(ray.dir) < 0)
            hit.normal = normal;
        else
            hit.normal = -normal;
        return true;
    }

    return false;
}

Eigen::Vector3d Triangle::getCenter() const {
    return (v0 + v1 + v2) / 3.0;
}