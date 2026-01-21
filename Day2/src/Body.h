//
// Created by kango on 2023/04/03.
// Extended for OBJ mesh and glass materials
//

#ifndef DAY_3_BODY_H
#define DAY_3_BODY_H

#include "Sphere.h"
#include "Mesh.h"
#include "Material.h"
#include <memory>
#include <variant>

struct Body {
    std::variant<Sphere, std::shared_ptr<Mesh>> geometry;
    Material material;

    // Sphere用コンストラクタ
    Body(Sphere sphere, Material material)
        : geometry(std::move(sphere)), material(std::move(material)) {}

    // Mesh用コンストラクタ
    Body(std::shared_ptr<Mesh> mesh, Material material)
        : geometry(std::move(mesh)), material(std::move(material)) {}

    bool hit(const Ray &ray, RayHit &hit) const {
        return std::visit([&ray, &hit](const auto &geo) -> bool {
            using T = std::decay_t<decltype(geo)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                return geo.hit(ray, hit);
            } else if constexpr (std::is_same_v<T, std::shared_ptr<Mesh>>) {
                return geo->hit(ray, hit);
            }
            return false;
        }, geometry);
    }

    Eigen::Vector3d getEmission() const {
        return material.emission * material.color;
    }

    Eigen::Vector3d getKd() const {
        return material.kd * material.color;
    }

    Eigen::Vector3d getNormal(const Eigen::Vector3d &p) const {
        return std::visit([&p](const auto &geo) -> Eigen::Vector3d {
            using T = std::decay_t<decltype(geo)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                return (p - geo.center).normalized();
            } else {
                // Meshの場合は交差判定時に法線が設定される
                return Eigen::Vector3d::UnitY();
            }
        }, geometry);
    }

    bool isLight() const {
        return material.emission > 0.0;
    }

    bool isGlass() const {
        return material.isGlass();
    }

    double getIOR() const {
        return material.ior;
    }

    // バウンディングボックスを取得（Mesh用）
    bool getBoundingBox(Eigen::Vector3d &bboxMin, Eigen::Vector3d &bboxMax) const {
        return std::visit([&bboxMin, &bboxMax](const auto &geo) -> bool {
            using T = std::decay_t<decltype(geo)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                bboxMin = geo.center - Eigen::Vector3d::Ones() * geo.radius;
                bboxMax = geo.center + Eigen::Vector3d::Ones() * geo.radius;
                return true;
            } else if constexpr (std::is_same_v<T, std::shared_ptr<Mesh>>) {
                bboxMin = geo->bboxMin;
                bboxMax = geo->bboxMax;
                return true;
            }
            return false;
        }, geometry);
    }

    // Sphere用: 光源サンプリング用に半径と中心を取得
    bool getSphereInfo(Eigen::Vector3d &center, double &radius) const {
        return std::visit([&center, &radius](const auto &geo) -> bool {
            using T = std::decay_t<decltype(geo)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                center = geo.center;
                radius = geo.radius;
                return true;
            }
            return false;
        }, geometry);
    }
};

#endif //DAY_3_BODY_H