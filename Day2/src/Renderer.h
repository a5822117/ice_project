//
// Renderer.h
// Path Tracing Renderer with NEE and BVH Acceleration
// Extended for ice/glass rendering
//

#ifndef DAY_2_RENDERER_H
#define DAY_2_RENDERER_H

#include <vector>
#include <random>
#include "Body.h"
#include "Camera.h"
#include "SceneBVH.h"

class Renderer {
public:
    std::vector<Body> bodies;
    Camera camera;
    Color bgColor;

    // BVH acceleration for scene traversal
    SceneBVH sceneBVH;
    bool useBVH = true;  // BVH使用フラグ

    // 乱数生成器
    mutable std::mt19937_64 engine;
    mutable std::uniform_real_distribution<> dist;

    // パストレーシングパラメータ
    unsigned int maxDepth;

    // 光源インデックス（NEE用）
    std::vector<int> lightIndices;

    Renderer(const std::vector<Body> &bodies, Camera camera,
             Color bgColor = Color::Zero(), unsigned int maxDepth = 20);

    double rand() const;

    // シーンとの交差判定（BVH高速版）
    bool hitScene(const Ray &ray, RayHit &hit) const;

    // BVHを再構築
    void rebuildBVH();

    // 基本レンダリング
    Image render() const;

    // Direct Illuminationレンダリング
    Image directIlluminationRender(const unsigned int &samples) const;

    // パストレーシングレンダリング with NEE
    Image pathTracingRender(const unsigned int &samplesPerPixel) const;

    // Spectral Rendering（波長依存レンダリング）
    Image spectralRender(const unsigned int &samplesPerPixel) const;

    // 再帰的パストレース
    Color tracePath(const Ray &ray, unsigned int depth,
                    bool insideObject = false, double currentIOR = 1.0,
                    bool prevSpecular = true) const;

    // Spectral用: 単一波長でのパストレース
    double tracePathSpectral(const Ray &ray, unsigned int depth,
                             int wavelengthIndex,
                             bool insideObject = false, double currentIOR = 1.0,
                             bool prevSpecular = true) const;

    // NEE: 光源直接サンプリング
    Color sampleDirectLight(const Eigen::Vector3d &hitPoint,
                           const Eigen::Vector3d &normal,
                           const Eigen::Vector3d &wo,
                           const Material &material) const;

    // 球面上の一様サンプリング
    Eigen::Vector3d sampleSphereUniform(const Eigen::Vector3d &center,
                                        double radius, double &pdf) const;

    // コサイン重点サンプリング
    void cosineSample(const Eigen::Vector3d &normal,
                      Eigen::Vector3d &outDir, double &pdf) const;

    // ローカル座標系構築
    static void computeLocalFrame(const Eigen::Vector3d &w,
                                  Eigen::Vector3d &u, Eigen::Vector3d &v);

    // Glass BSDFサンプリング
    double sampleGlassBSDF(const Eigen::Vector3d &wo, const Eigen::Vector3d &normal,
                           double n1, double n2, Eigen::Vector3d &wi,
                           bool &isRefraction) const;

    //==========================================================================
    // Microfacet Model（Ghafari & Park 2017）
    //==========================================================================

    /// Beckmann分布を使用してマイクロファセット法線をサンプリング
    /// @param geometricNormal 幾何学的な法線（メッシュの法線）
    /// @param alpha 表面の粗さパラメータ（0.0 = 完全鏡面）
    /// @return 摂動されたマイクロファセット法線
    Eigen::Vector3d sampleBeckmannNormal(const Eigen::Vector3d &geometricNormal,
                                          double alpha) const;
};

#endif //DAY_2_RENDERER_H