//
// SceneBVH.h
// Scene-level BVH for accelerating intersection tests with all objects
// Particularly effective for scenes with many bubbles (hundreds of spheres)
//

#ifndef DAY_2_SCENE_BVH_H
#define DAY_2_SCENE_BVH_H

#include "BVH.h"
#include "Body.h"
#include <vector>

//==============================================================================
// SceneBVH
// シーン全体のオブジェクト（球体、メッシュ）に対するBVH
// 気泡が多数存在するシーンで効果的に高速化
//==============================================================================
class SceneBVH {
public:
    std::unique_ptr<BVHNode> root;
    std::vector<int> orderedBodyIndices;  // BVH順に並んだボディインデックス

    SceneBVH() = default;

    // シーン内のすべてのオブジェクトからBVHを構築
    void build(const std::vector<Body>& bodies);

    // レイとシーンの交差判定（高速版）
    // bodies: シーン内のすべてのオブジェクト
    // ray: 判定するレイ
    // hit: 交差情報（出力）
    // 戻り値: 交差した場合はtrue、hit.idxには元のボディインデックスが設定される
    bool intersect(const std::vector<Body>& bodies,
                   const Ray& ray, RayHit& hit) const;

private:
    // 再帰的な交差判定
    bool intersectRecursive(const std::vector<Body>& bodies,
                           const BVHNode* node,
                           const Ray& ray,
                           RayHit& hit,
                           double& tMin) const;
};

//==============================================================================
// Implementation
//==============================================================================

inline void SceneBVH::build(const std::vector<Body>& bodies) {
    if (bodies.empty()) {
        root = nullptr;
        return;
    }

    // 各ボディのBVH用情報を作成
    std::vector<BVHPrimitiveInfo> primitiveInfos;
    primitiveInfos.reserve(bodies.size());

    for (size_t i = 0; i < bodies.size(); ++i) {
        Eigen::Vector3d bboxMin, bboxMax;

        if (bodies[i].getBoundingBox(bboxMin, bboxMax)) {
            AABB bounds(bboxMin, bboxMax);

            // 数値誤差を考慮して少し拡張
            Eigen::Vector3d epsilon = Eigen::Vector3d::Constant(1e-5);
            bounds.minPoint -= epsilon;
            bounds.maxPoint += epsilon;

            primitiveInfos.emplace_back(static_cast<int>(i), bounds);
        }
    }

    // BVHを構築
    root = BVHBuilder::build(primitiveInfos, orderedBodyIndices);
}

inline bool SceneBVH::intersect(const std::vector<Body>& bodies,
                                const Ray& ray, RayHit& hit) const {
    if (!root) return false;

    hit.t = std::numeric_limits<double>::max();
    hit.idx = -1;
    double tMin = std::numeric_limits<double>::max();

    bool result = intersectRecursive(bodies, root.get(), ray, hit, tMin);

    return result;
}

inline bool SceneBVH::intersectRecursive(const std::vector<Body>& bodies,
                                         const BVHNode* node,
                                         const Ray& ray,
                                         RayHit& hit,
                                         double& tMin) const {
    // まずバウンディングボックスとの交差をチェック
    double tBoxMin, tBoxMax;
    if (!node->bounds.intersect(ray, tBoxMin, tBoxMax)) {
        return false;
    }

    // 既に見つかった交点より遠いノードはスキップ（early termination）
    if (tBoxMin > tMin) {
        return false;
    }

    bool hitAny = false;

    if (node->isLeaf()) {
        // 葉ノード：含まれるオブジェクトすべてと交差判定
        for (int i = 0; i < node->primitiveCount; ++i) {
            int bodyIdx = orderedBodyIndices[node->primitiveStart + i];
            RayHit tempHit;

            if (bodies[bodyIdx].hit(ray, tempHit) &&
                tempHit.t > 1e-6 && tempHit.t < tMin) {
                tMin = tempHit.t;
                hit = tempHit;
                hit.idx = bodyIdx;  // 元のボディインデックスを保存
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

        // より近いノードを先に探索（早期終了のため重要）
        if (hitLeft && hitRight && tLeftMin > tRightMin) {
            std::swap(firstNode, secondNode);
        }

        // 再帰的に探索
        if (intersectRecursive(bodies, firstNode, ray, hit, tMin)) {
            hitAny = true;
        }
        if (intersectRecursive(bodies, secondNode, ray, hit, tMin)) {
            hitAny = true;
        }
    }

    return hitAny;
}

#endif // DAY_2_SCENE_BVH_H