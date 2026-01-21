//
// Renderer.h
// Path Tracing Renderer with NEE and BVH Acceleration
// Extended for ice/glass rendering
// + Hero Wavelength Sampling for efficient spectral rendering
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
    bool useBVH = true;

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

    // Spectral Rendering（従来版 - 7波長独立サンプリング）
    Image spectralRender(const unsigned int &samplesPerPixel) const;

    //==========================================================================
    // Hero Wavelength Sampling（効率化版）
    // 各サンプルで1つの波長をランダムに選択し、その波長でパストレース
    // 計算量が約1/7に削減される
    //==========================================================================
    Image spectralRenderHero(const unsigned int &samplesPerPixel) const;

    // 再帰的パストレース（RGB版）
    Color tracePath(const Ray &ray, unsigned int depth,
                    bool insideObject = false, double currentIOR = 1.0,
                    bool prevSpecular = true) const;

    // Spectral用: 単一波長でのパストレース（離散波長インデックス版）
    double tracePathSpectral(const Ray &ray, unsigned int depth,
                             int wavelengthIndex,
                             bool insideObject = false, double currentIOR = 1.0,
                             bool prevSpecular = true) const;

    //==========================================================================
    // Hero Wavelength用: 連続波長でのパストレース
    // wavelength_nm: 波長（nm単位、400-700の連続値）
    //==========================================================================
    double tracePathHero(const Ray &ray, unsigned int depth,
                         double wavelength_nm,
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
    Eigen::Vector3d sampleBeckmannNormal(const Eigen::Vector3d &geometricNormal,
                                          double alpha) const;
};

#endif //DAY_2_RENDERER_H