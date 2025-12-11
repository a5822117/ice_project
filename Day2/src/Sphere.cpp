//
// Created by kango on 2023/03/22.
//

#include "Sphere.h"

Sphere::Sphere(double radius, Eigen::Vector3d center) : radius(radius), center(std::move(center)) {}

bool Sphere::hit(const Ray &ray, RayHit &hit) const {
    /// ２次方程式を解く
    const double b = (ray.org - center).dot(ray.dir);
    const double c = (ray.org - center).squaredNorm() - radius * radius;
    const double discriminant = b * b - c;

    /// 判別式が0未満なら解無し
    if(discriminant < 0.0) return false;

    const Eigen::Array2d distances{(-b - sqrt(discriminant)),
                                   (-b + sqrt(discriminant))
    };

    /// どちらの解も0以下なら後ろに衝突とみなしてFalseを返す
    if((distances < 1e-6).all()) return false;
    /// 小さい方の解が0以上なら小さい方、そうでなければ大きい方をを格納する
    hit.t = distances[0] > 1e-6 ? distances[0] : distances[1];
    hit.point = ray.at(hit.t);
    const Eigen::Vector3d outwardNormal = (hit.point - center).normalized();

    /// レイの方向と外向き法線の内積が０未満なら球の内側に衝突しているので内向きの法線を格納する
    if(outwardNormal.dot(ray.dir) < 0.0) {
        hit.normal = outwardNormal;
    } else {
        hit.normal = -outwardNormal;
    }
    return true;
}