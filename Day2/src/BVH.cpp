//
// BVH.cpp
// BVH implementation for triangle meshes
//

#include "BVH.h"
#include "Triangle.h"

//==============================================================================
// TriangleBVH Implementation
//==============================================================================

void TriangleBVH::build(const std::vector<Triangle>& triangles) {
    if (triangles.empty()) {
        root = nullptr;
        return;
    }

    // 各三角形のBVH用情報を作成
    std::vector<BVHPrimitiveInfo> primitiveInfos;
    primitiveInfos.reserve(triangles.size());

    for (size_t i = 0; i < triangles.size(); ++i) {
        const Triangle& tri = triangles[i];

        // 三角形のAABBを計算
        AABB bounds;
        bounds.extend(tri.v0);
        bounds.extend(tri.v1);
        bounds.extend(tri.v2);

        // 数値誤差を考慮して少し拡張
        Eigen::Vector3d epsilon = Eigen::Vector3d::Constant(1e-6);
        bounds.minPoint -= epsilon;
        bounds.maxPoint += epsilon;

        primitiveInfos.emplace_back(static_cast<int>(i), bounds);
    }

    // BVHを構築
    root = BVHBuilder::build(primitiveInfos, orderedTriangleIndices);
}

bool TriangleBVH::intersect(const std::vector<Triangle>& triangles,
                           const Ray& ray, RayHit& hit) const {
    if (!root) return false;

    hit.t = std::numeric_limits<double>::max();
    hit.idx = -1;
    double tMin = std::numeric_limits<double>::max();

    return intersectRecursive(triangles, root.get(), ray, hit, tMin);
}

bool TriangleBVH::intersectRecursive(const std::vector<Triangle>& triangles,
                                     const BVHNode* node,
                                     const Ray& ray,
                                     RayHit& hit,
                                     double& tMin) const {
    // まずバウンディングボックスとの交差をチェック
    double tBoxMin, tBoxMax;
    if (!node->bounds.intersect(ray, tBoxMin, tBoxMax)) {
        return false;
    }

    // 既に見つかった交点より遠いノードはスキップ
    if (tBoxMin > tMin) {
        return false;
    }

    bool hitAny = false;

    if (node->isLeaf()) {
        // 葉ノード：含まれる三角形すべてと交差判定
        for (int i = 0; i < node->primitiveCount; ++i) {
            int triIdx = orderedTriangleIndices[node->primitiveStart + i];
            RayHit tempHit;

            if (triangles[triIdx].hit(ray, tempHit) &&
                tempHit.t > 1e-6 && tempHit.t < tMin) {
                tMin = tempHit.t;
                hit = tempHit;
                hit.idx = triIdx;  // 三角形のインデックスを保存
                hitAny = true;
            }
        }
    } else {
        // 内部ノード：両方の子ノードを探索
        // 近い方のノードから先に探索することで効率化
        const BVHNode* firstNode = node->left.get();
        const BVHNode* secondNode = node->right.get();

        // レイの方向に基づいて探索順序を決定
        double tLeftMin, tLeftMax, tRightMin, tRightMax;
        bool hitLeft = firstNode->bounds.intersect(ray, tLeftMin, tLeftMax);
        bool hitRight = secondNode->bounds.intersect(ray, tRightMin, tRightMax);

        // より近いノードを先に探索
        if (hitLeft && hitRight && tLeftMin > tRightMin) {
            std::swap(firstNode, secondNode);
        }

        if (intersectRecursive(triangles, firstNode, ray, hit, tMin)) {
            hitAny = true;
        }
        if (intersectRecursive(triangles, secondNode, ray, hit, tMin)) {
            hitAny = true;
        }
    }

    return hitAny;
}