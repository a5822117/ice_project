//
// BVH.h
// Bounding Volume Hierarchy for accelerated ray tracing
// Using Surface Area Heuristic (SAH) for optimal tree construction
//

#ifndef DAY_2_BVH_H
#define DAY_2_BVH_H

#include <vector>
#include <algorithm>
#include <limits>
#include <memory>
#include <Eigen/Dense>
#include "Ray.h"

//==============================================================================
// AABB (Axis-Aligned Bounding Box)
// レイとの交差判定を高速に行うための軸平行バウンディングボックス
//==============================================================================
struct AABB {
    Eigen::Vector3d minPoint;
    Eigen::Vector3d maxPoint;

    AABB()
        : minPoint(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
          maxPoint(Eigen::Vector3d::Constant(-std::numeric_limits<double>::max())) {}

    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
        : minPoint(min), maxPoint(max) {}

    // 2つのAABBを結合
    static AABB merge(const AABB& a, const AABB& b) {
        return AABB(
            a.minPoint.cwiseMin(b.minPoint),
            a.maxPoint.cwiseMax(b.maxPoint)
        );
    }

    // 点を含むようにAABBを拡張
    void extend(const Eigen::Vector3d& point) {
        minPoint = minPoint.cwiseMin(point);
        maxPoint = maxPoint.cwiseMax(point);
    }

    // AABBを結合
    void extend(const AABB& other) {
        minPoint = minPoint.cwiseMin(other.minPoint);
        maxPoint = maxPoint.cwiseMax(other.maxPoint);
    }

    // 表面積を計算（SAHで使用）
    double surfaceArea() const {
        Eigen::Vector3d d = maxPoint - minPoint;
        return 2.0 * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
    }

    // 中心点を取得
    Eigen::Vector3d center() const {
        return 0.5 * (minPoint + maxPoint);
    }

    // レイとの交差判定（高速版: slab method）
    // 戻り値: 交差する場合はtrue、tMinとtMaxに交差区間を設定
    bool intersect(const Ray& ray, double& tMin, double& tMax) const {
        tMin = 0.0;
        tMax = std::numeric_limits<double>::max();

        for (int i = 0; i < 3; ++i) {
            double invD = 1.0 / ray.dir[i];
            double t0 = (minPoint[i] - ray.org[i]) * invD;
            double t1 = (maxPoint[i] - ray.org[i]) * invD;

            if (invD < 0.0) std::swap(t0, t1);

            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;

            if (tMax < tMin) return false;
        }

        return true;
    }

    // 簡易版：交差するかどうかだけを判定
    bool intersect(const Ray& ray) const {
        double tMin, tMax;
        return intersect(ray, tMin, tMax);
    }
};

//==============================================================================
// BVHNode
// BVHの各ノード。葉ノードはプリミティブを保持、内部ノードは子ノードを保持
//==============================================================================
struct BVHNode {
    AABB bounds;                          // このノードのバウンディングボックス
    std::unique_ptr<BVHNode> left;        // 左の子ノード（内部ノードのみ）
    std::unique_ptr<BVHNode> right;       // 右の子ノード（内部ノードのみ）
    int primitiveStart;                   // 葉ノード：プリミティブの開始インデックス
    int primitiveCount;                   // 葉ノード：プリミティブの数

    BVHNode() : primitiveStart(-1), primitiveCount(0) {}

    bool isLeaf() const {
        return primitiveCount > 0;
    }
};

//==============================================================================
// BVH構築用のプリミティブ情報
//==============================================================================
struct BVHPrimitiveInfo {
    int index;              // 元のプリミティブ配列でのインデックス
    AABB bounds;            // プリミティブのバウンディングボックス
    Eigen::Vector3d center; // バウンディングボックスの中心

    BVHPrimitiveInfo(int idx, const AABB& b)
        : index(idx), bounds(b), center(b.center()) {}
};

//==============================================================================
// BVHBuilder
// SAH（Surface Area Heuristic）を使用したBVH構築クラス
//==============================================================================
class BVHBuilder {
public:
    // SAHパラメータ
    static constexpr double TRAVERSAL_COST = 1.0;      // ノード通過コスト
    static constexpr double INTERSECTION_COST = 1.0;    // 交差判定コスト
    static constexpr int MAX_LEAF_SIZE = 4;             // 葉ノードの最大プリミティブ数
    static constexpr int SAH_BUCKET_COUNT = 12;         // SAHのバケット数

    // BVHを構築
    // primitiveInfos: 各プリミティブの情報
    // orderedIndices: 出力 - BVH順に並べ替えられたプリミティブインデックス
    static std::unique_ptr<BVHNode> build(
        std::vector<BVHPrimitiveInfo>& primitiveInfos,
        std::vector<int>& orderedIndices)
    {
        orderedIndices.clear();
        orderedIndices.reserve(primitiveInfos.size());

        if (primitiveInfos.empty()) {
            return nullptr;
        }

        return buildRecursive(primitiveInfos, 0, primitiveInfos.size(), orderedIndices);
    }

private:
    // 再帰的にBVHを構築
    static std::unique_ptr<BVHNode> buildRecursive(
        std::vector<BVHPrimitiveInfo>& primitiveInfos,
        int start, int end,
        std::vector<int>& orderedIndices)
    {
        auto node = std::make_unique<BVHNode>();

        // このノードの全プリミティブを含むAABBを計算
        AABB bounds;
        for (int i = start; i < end; ++i) {
            bounds.extend(primitiveInfos[i].bounds);
        }
        node->bounds = bounds;

        int primitiveCount = end - start;

        // プリミティブが少ない場合は葉ノードを作成
        if (primitiveCount <= MAX_LEAF_SIZE) {
            node->primitiveStart = static_cast<int>(orderedIndices.size());
            node->primitiveCount = primitiveCount;
            for (int i = start; i < end; ++i) {
                orderedIndices.push_back(primitiveInfos[i].index);
            }
            return node;
        }

        // 中心点のバウンディングボックスを計算
        AABB centroidBounds;
        for (int i = start; i < end; ++i) {
            centroidBounds.extend(primitiveInfos[i].center);
        }

        // 最も広がりのある軸を選択
        Eigen::Vector3d diagonal = centroidBounds.maxPoint - centroidBounds.minPoint;
        int axis = 0;
        if (diagonal.y() > diagonal.x()) axis = 1;
        if (diagonal.z() > diagonal[axis]) axis = 2;

        // 中心点がすべて同じ位置にある場合は葉ノードを作成
        if (centroidBounds.maxPoint[axis] == centroidBounds.minPoint[axis]) {
            node->primitiveStart = static_cast<int>(orderedIndices.size());
            node->primitiveCount = primitiveCount;
            for (int i = start; i < end; ++i) {
                orderedIndices.push_back(primitiveInfos[i].index);
            }
            return node;
        }

        // SAH（Surface Area Heuristic）を使用して最適な分割位置を決定
        int mid = partitionBySAH(primitiveInfos, start, end, axis, bounds, centroidBounds);

        // 再帰的に子ノードを構築
        node->left = buildRecursive(primitiveInfos, start, mid, orderedIndices);
        node->right = buildRecursive(primitiveInfos, mid, end, orderedIndices);
        node->primitiveStart = -1;
        node->primitiveCount = 0;

        return node;
    }

    // SAHを使用してプリミティブを分割
    static int partitionBySAH(
        std::vector<BVHPrimitiveInfo>& primitiveInfos,
        int start, int end, int axis,
        const AABB& bounds, const AABB& centroidBounds)
    {
        int primitiveCount = end - start;

        // プリミティブが少ない場合は単純に中央で分割
        if (primitiveCount <= 2) {
            int mid = (start + end) / 2;
            std::nth_element(
                primitiveInfos.begin() + start,
                primitiveInfos.begin() + mid,
                primitiveInfos.begin() + end,
                [axis](const BVHPrimitiveInfo& a, const BVHPrimitiveInfo& b) {
                    return a.center[axis] < b.center[axis];
                }
            );
            return mid;
        }

        // バケットを初期化
        struct Bucket {
            int count = 0;
            AABB bounds;
        };
        std::array<Bucket, SAH_BUCKET_COUNT> buckets;

        double extent = centroidBounds.maxPoint[axis] - centroidBounds.minPoint[axis];

        // 各プリミティブをバケットに割り当て
        for (int i = start; i < end; ++i) {
            double offset = primitiveInfos[i].center[axis] - centroidBounds.minPoint[axis];
            int b = static_cast<int>(SAH_BUCKET_COUNT * offset / extent);
            if (b >= SAH_BUCKET_COUNT) b = SAH_BUCKET_COUNT - 1;
            buckets[b].count++;
            buckets[b].bounds.extend(primitiveInfos[i].bounds);
        }

        // 各分割位置でのコストを計算
        double costs[SAH_BUCKET_COUNT - 1];
        for (int i = 0; i < SAH_BUCKET_COUNT - 1; ++i) {
            AABB b0, b1;
            int count0 = 0, count1 = 0;

            for (int j = 0; j <= i; ++j) {
                b0.extend(buckets[j].bounds);
                count0 += buckets[j].count;
            }
            for (int j = i + 1; j < SAH_BUCKET_COUNT; ++j) {
                b1.extend(buckets[j].bounds);
                count1 += buckets[j].count;
            }

            double cost = TRAVERSAL_COST + INTERSECTION_COST * (
                count0 * b0.surfaceArea() + count1 * b1.surfaceArea()
            ) / bounds.surfaceArea();

            costs[i] = cost;
        }

        // 最小コストの分割位置を見つける
        double minCost = costs[0];
        int minCostSplit = 0;
        for (int i = 1; i < SAH_BUCKET_COUNT - 1; ++i) {
            if (costs[i] < minCost) {
                minCost = costs[i];
                minCostSplit = i;
            }
        }

        // 分割しない場合のコストと比較
        double leafCost = INTERSECTION_COST * primitiveCount;

        if (minCost < leafCost || primitiveCount > MAX_LEAF_SIZE) {
            // 分割を実行
            auto midIter = std::partition(
                primitiveInfos.begin() + start,
                primitiveInfos.begin() + end,
                [&](const BVHPrimitiveInfo& pi) {
                    double offset = pi.center[axis] - centroidBounds.minPoint[axis];
                    int b = static_cast<int>(SAH_BUCKET_COUNT * offset / extent);
                    if (b >= SAH_BUCKET_COUNT) b = SAH_BUCKET_COUNT - 1;
                    return b <= minCostSplit;
                }
            );
            int mid = static_cast<int>(midIter - primitiveInfos.begin());

            // 分割が片側に偏った場合は中央で分割
            if (mid == start || mid == end) {
                mid = (start + end) / 2;
                std::nth_element(
                    primitiveInfos.begin() + start,
                    primitiveInfos.begin() + mid,
                    primitiveInfos.begin() + end,
                    [axis](const BVHPrimitiveInfo& a, const BVHPrimitiveInfo& b) {
                        return a.center[axis] < b.center[axis];
                    }
                );
            }

            return mid;
        }

        // 分割しない場合（MAX_LEAF_SIZE以下なら葉ノードになる）
        return (start + end) / 2;
    }
};

//==============================================================================
// TriangleBVH
// 三角形メッシュ専用のBVH
//==============================================================================
class Triangle;  // Forward declaration

class TriangleBVH {
public:
    std::unique_ptr<BVHNode> root;
    std::vector<int> orderedTriangleIndices;  // BVH順に並んだ三角形インデックス

    TriangleBVH() = default;

    // 三角形配列からBVHを構築
    void build(const std::vector<Triangle>& triangles);

    // レイとの交差判定
    // triangles: 元の三角形配列
    // ray: 判定するレイ
    // hit: 交差情報（出力）
    // 戻り値: 交差した場合はtrue
    bool intersect(const std::vector<Triangle>& triangles,
                   const Ray& ray, RayHit& hit) const;

private:
    // 再帰的な交差判定
    bool intersectRecursive(const std::vector<Triangle>& triangles,
                           const BVHNode* node,
                           const Ray& ray,
                           RayHit& hit,
                           double& tMin) const;
};

#endif // DAY_2_BVH_H