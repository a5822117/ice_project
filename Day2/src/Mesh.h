//
// Mesh.h
// OBJ mesh support with BVH acceleration
// + Y軸回転サポート追加
//

#ifndef DAY_2_MESH_H
#define DAY_2_MESH_H

#include "Triangle.h"
#include "Ray.h"
#include "BVH.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

class Mesh {
public:
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Triangle> triangles;

    // バウンディングボックス
    Eigen::Vector3d bboxMin, bboxMax;

    // BVH acceleration structure
    TriangleBVH bvh;
    bool bvhBuilt = false;

    Mesh() = default;

    bool loadOBJ(const std::string &filename);

    // BVHを構築（loadOBJ後に自動で呼ばれる）
    void buildBVH();

    // 交差判定（BVHを使用）
    bool hit(const Ray &ray, RayHit &hit) const;

    // 変換操作
    void translate(const Eigen::Vector3d &offset);
    void scale(double scale);
    void rotateY(double degrees);  // ★追加: Y軸周りの回転
    void updateBoundingBox();
    double getMinY() const;

    // BVHの再構築が必要かどうか
    void invalidateBVH() { bvhBuilt = false; }
};

#endif //DAY_2_MESH_H