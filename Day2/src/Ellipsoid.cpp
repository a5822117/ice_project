//
// Ellipsoid.cpp
// 楕円体のレイ交差判定
// 楕円体方程式: (x/a)² + (y/b)² + (z/c)² = 1
//

#define _USE_MATH_DEFINES
#include "Ellipsoid.h"
#include <cmath>
#include <algorithm>

Ellipsoid::Ellipsoid(const Eigen::Vector3d &center, const Eigen::Vector3d &radii)
    : center(center), radii(radii) {}

Ellipsoid::Ellipsoid(const Eigen::Vector3d &center, double baseRadius, double elongationY)
    : center(center) {
    // Y軸方向に elongationY 倍伸びた楕円体
    // 体積を保存する場合: rx * ry * rz = r³
    // ry = r * elongationY とすると、rx = rz = r / sqrt(elongationY)
    double rx = baseRadius / std::sqrt(elongationY);
    double ry = baseRadius * elongationY;
    double rz = rx;
    radii = Eigen::Vector3d(rx, ry, rz);
}

bool Ellipsoid::hit(const Ray &ray, RayHit &hit) const {
    //==========================================================================
    // 楕円体とレイの交差判定
    // 楕円体: ((x-cx)/a)² + ((y-cy)/b)² + ((z-cz)/c)² = 1
    // レイ: P(t) = O + t*D
    //
    // 代入して整理: A*t² + B*t + C = 0
    //==========================================================================

    // レイの原点を楕円体中心基準に
    Eigen::Vector3d oc = ray.org - center;

    // 各軸をradiiで正規化
    double a = radii.x(), b = radii.y(), c = radii.z();
    double a2 = a * a, b2 = b * b, c2 = c * c;

    // 二次方程式の係数
    double A = (ray.dir.x() * ray.dir.x()) / a2 +
               (ray.dir.y() * ray.dir.y()) / b2 +
               (ray.dir.z() * ray.dir.z()) / c2;

    double B = 2.0 * (oc.x() * ray.dir.x() / a2 +
                      oc.y() * ray.dir.y() / b2 +
                      oc.z() * ray.dir.z() / c2);

    double C = (oc.x() * oc.x()) / a2 +
               (oc.y() * oc.y()) / b2 +
               (oc.z() * oc.z()) / c2 - 1.0;

    // 判別式
    double discriminant = B * B - 4.0 * A * C;

    if (discriminant < 0.0) {
        return false;
    }

    double sqrtD = std::sqrt(discriminant);
    double t0 = (-B - sqrtD) / (2.0 * A);
    double t1 = (-B + sqrtD) / (2.0 * A);

    // 有効な t を選択（近い方で正の値）
    double t;
    const double tMin = 1e-6;

    if (t0 > tMin) {
        t = t0;
    } else if (t1 > tMin) {
        t = t1;
    } else {
        return false;
    }

    // 交点と法線を計算
    hit.t = t;
    hit.point = ray.at(t);

    // 楕円体表面の法線
    // ∇f = (2(x-cx)/a², 2(y-cy)/b², 2(z-cz)/c²)
    Eigen::Vector3d localP = hit.point - center;
    Eigen::Vector3d normal(
        localP.x() / a2,
        localP.y() / b2,
        localP.z() / c2
    );
    normal.normalize();

    // レイの方向と法線が同じ向きなら裏面からのヒット
    if (normal.dot(ray.dir) > 0) {
        hit.normal = -normal;
    } else {
        hit.normal = normal;
    }

    return true;
}

double Ellipsoid::maxRadius() const {
    return radii.maxCoeff();
}