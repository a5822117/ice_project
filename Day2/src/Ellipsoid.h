//
// Ellipsoid.h
// 楕円体クラス - 方向性のある気泡を表現するため
// 氷の凍結時、気泡は特定の方向（通常は上向き）に引き伸ばされる
//

#ifndef DAY_2_ELLIPSOID_H
#define DAY_2_ELLIPSOID_H

#include "Eigen/Dense"
#include "Ray.h"

class Ellipsoid {
public:
    Eigen::Vector3d center;   // 中心位置
    Eigen::Vector3d radii;    // 各軸の半径 (rx, ry, rz)

    Ellipsoid() = default;

    /// 楕円体コンストラクタ
    /// @param center 中心位置
    /// @param radii 各軸の半径 (rx, ry, rz)
    Ellipsoid(const Eigen::Vector3d &center, const Eigen::Vector3d &radii);

    /// 球と同じ半径で楕円体を作成（伸長率を適用）
    /// @param center 中心位置
    /// @param baseRadius 基本半径
    /// @param elongationY Y軸方向の伸長率（1.0 = 球、2.0 = 2倍に伸びる）
    Ellipsoid(const Eigen::Vector3d &center, double baseRadius, double elongationY);

    /// レイとの交差判定
    bool hit(const Ray &ray, RayHit &hit) const;

    /// バウンディングボックス用の最大半径
    double maxRadius() const;
};

#endif // DAY_2_ELLIPSOID_H