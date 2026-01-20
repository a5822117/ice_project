//
// Ice Rendering with OBJ Mesh
// Based on "Realistic Rendering of Ice and Crack Propagations" (2017)
// + "A Geometric Approach to Efficient Modeling and Rendering of Opaque Ice
//    With Directional Air Bubbles" (Kim, 2025)
// + Hero Wavelength Sampling for fast spectral rendering
//

#define _USE_MATH_DEFINES
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include "Body.h"
#include "Camera.h"
#include "Renderer.h"
#include "Mesh.h"
#include "BubbleLoader.h"
#include "Ellipsoid.h"

//==============================================================================
// 方向性のある楕円体気泡データ構造
//==============================================================================
struct DirectionalBubble {
    Eigen::Vector3d center;
    Eigen::Vector3d radii;      // (rx, ry, rz)
    Eigen::Vector3d direction;  // 気泡の主軸方向

    DirectionalBubble(const Eigen::Vector3d &c, const Eigen::Vector3d &r,
                      const Eigen::Vector3d &dir)
        : center(c), radii(r), direction(dir.normalized()) {}
};

//==============================================================================
// 二重正規分布による気泡生成
// 分布1: 方向がバラバラ（ランダム方向）
// 分布2: おおよそ氷の中心を向く（方向性あり）
//==============================================================================
std::vector<DirectionalBubble> generateDirectionalBubbles(
        const Eigen::Vector3d &bboxMin,
        const Eigen::Vector3d &bboxMax,
        double elongation = 2.0,
        int seed = 42) {

    std::vector<DirectionalBubble> bubbles;
    std::mt19937_64 engine(seed);
    std::normal_distribution<double> normal(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Eigen::Vector3d iceCenter = (bboxMin + bboxMax) / 2.0;
    Eigen::Vector3d size = bboxMax - bboxMin;
    double minDim = size.minCoeff();

    //==========================================================================
    // 分布1: 中心に集中、方向がバラバラ（ランダム方向）
    // 白濁効果のための等方性散乱を生む
    //==========================================================================
    const int num_random_dir = 600;  // 増量
    const double pos_std_random = minDim * 0.15;
    const double radius_mean_random = minDim * 0.010;
    const double radius_std_random = minDim * 0.004;

    std::cout << "Generating distribution 1: Random direction bubbles..." << std::endl;

    for (int i = 0; i < num_random_dir; ++i) {
        // 中心付近にガウス分布で配置
        Eigen::Vector3d pos(
            normal(engine) * pos_std_random,
            normal(engine) * pos_std_random,
            normal(engine) * pos_std_random
        );
        pos += iceCenter;

        // 基本半径
        double baseRadius = std::abs(normal(engine)) * radius_std_random + radius_mean_random;
        baseRadius = std::max(minDim * 0.004, std::min(baseRadius, minDim * 0.02));

        // ランダム方向（バラバラ）
        Eigen::Vector3d direction(
            normal(engine),
            normal(engine),
            normal(engine)
        );
        if (direction.norm() < 1e-6) {
            direction = Eigen::Vector3d::UnitY();
        }
        direction.normalize();

        // 伸長率（ランダム方向なので控えめ）
        double localElongation = 1.0 + uniform(engine) * 1.0;  // 1.0 ~ 2.0
        double rMain = baseRadius * localElongation;
        double rSide = baseRadius * 0.5;
        Eigen::Vector3d radii(rSide, rMain, rSide);

        // 境界チェック
        bool inside = true;
        double maxR = radii.maxCoeff();
        for (int j = 0; j < 3; ++j) {
            if (pos[j] - maxR < bboxMin[j] + minDim * 0.03 ||
                pos[j] + maxR > bboxMax[j] - minDim * 0.03) {
                inside = false;
                break;
            }
        }

        if (inside) {
            bubbles.emplace_back(pos, radii, direction);
        }
    }

    //==========================================================================
    // 分布2: 全体に分布、おおよそ氷の中心を向く（方向性あり）
    // Kim(2025): 凍結時に気泡は凍結面から中心へ押し出される
    //==========================================================================
    const int num_toward_center = 350;  // 増量
    const double pos_std_center = minDim * 0.30;  // より広く分布
    const double radius_mean_center = minDim * 0.014;
    const double radius_std_center = minDim * 0.005;

    std::cout << "Generating distribution 2: Center-pointing bubbles..." << std::endl;

    for (int i = 0; i < num_toward_center; ++i) {
        // 全体に広くガウス分布で配置
        Eigen::Vector3d pos(
            normal(engine) * pos_std_center,
            normal(engine) * pos_std_center,
            normal(engine) * pos_std_center
        );
        pos += iceCenter;

        double baseRadius = std::abs(normal(engine)) * radius_std_center + radius_mean_center;
        baseRadius = std::max(minDim * 0.006, std::min(baseRadius, minDim * 0.028));

        // 方向: 気泡の位置から氷の中心へ向かう方向
        Eigen::Vector3d toCenter = iceCenter - pos;
        if (toCenter.norm() < 1e-6) {
            toCenter = Eigen::Vector3d::UnitY();
        }
        toCenter.normalize();

        // 方向に少しランダム性を加える（完全に中心向きではなく「おおよそ」）
        toCenter += Eigen::Vector3d(
            normal(engine) * 0.25,
            normal(engine) * 0.25,
            normal(engine) * 0.25
        );
        toCenter.normalize();

        // 伸長率（方向性があるので強め）
        double localElongation = elongation * (0.7 + uniform(engine) * 0.6);
        double rMain = baseRadius * localElongation;
        double rSide = baseRadius * 0.35;
        Eigen::Vector3d radii(rSide, rMain, rSide);

        bool inside = true;
        double maxR = radii.maxCoeff();
        for (int j = 0; j < 3; ++j) {
            if (pos[j] - maxR < bboxMin[j] + minDim * 0.03 ||
                pos[j] + maxR > bboxMax[j] - minDim * 0.03) {
                inside = false;
                break;
            }
        }

        if (inside) {
            bubbles.emplace_back(pos, radii, toCenter);
        }
    }

    //==========================================================================
    // 追加: 微小な等方性気泡（白濁効果を強化）
    //==========================================================================
    const int num_tiny = 500;  // 多数の小さな気泡
    const double pos_std_tiny = minDim * 0.20;
    const double radius_mean_tiny = minDim * 0.006;
    const double radius_std_tiny = minDim * 0.002;

    std::cout << "Generating tiny isotropic bubbles for cloudiness..." << std::endl;

    for (int i = 0; i < num_tiny; ++i) {
        Eigen::Vector3d pos(
            normal(engine) * pos_std_tiny,
            normal(engine) * pos_std_tiny,
            normal(engine) * pos_std_tiny
        );
        pos += iceCenter;

        double radius = std::abs(normal(engine)) * radius_std_tiny + radius_mean_tiny;
        radius = std::max(minDim * 0.003, std::min(radius, minDim * 0.012));

        // ほぼ球形（等方性）
        double elongFactor = 1.0 + uniform(engine) * 0.2;
        Eigen::Vector3d radii(radius, radius * elongFactor, radius);
        Eigen::Vector3d direction = Eigen::Vector3d::UnitY();

        bool inside = true;
        double maxR = radii.maxCoeff();
        for (int j = 0; j < 3; ++j) {
            if (pos[j] - maxR < bboxMin[j] + minDim * 0.04 ||
                pos[j] + maxR > bboxMax[j] - minDim * 0.04) {
                inside = false;
                break;
            }
        }

        if (inside) {
            bubbles.emplace_back(pos, radii, direction);
        }
    }

    std::cout << "\nBubble generation summary:" << std::endl;
    std::cout << "  - Distribution 1 (random direction): " << num_random_dir << " requested" << std::endl;
    std::cout << "  - Distribution 2 (toward center): " << num_toward_center << " requested" << std::endl;
    std::cout << "  - Tiny isotropic (cloudiness): " << num_tiny << " requested" << std::endl;
    std::cout << "  - Total generated: " << bubbles.size() << std::endl;
    std::cout << "  - Elongation factor: " << elongation << std::endl;

    return bubbles;
}

//==============================================================================
// iceRenderingSpectral: 方向性気泡付きレンダリング（水滴なし）
//==============================================================================
void iceRenderingSpectral(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "============================================" << std::endl;
    std::cout << "Spectral Ice Rendering" << std::endl;
    std::cout << "Based on:" << std::endl;
    std::cout << "  - Ghafari & Park (2017): Ice crack propagation" << std::endl;
    std::cout << "  - Kim (2025): Directional air bubbles" << std::endl;
    std::cout << "  + Hero Wavelength Sampling (~7x speedup)" << std::endl;
    std::cout << "============================================" << std::endl;

    // ice.objを読み込む
    auto iceMesh = std::make_shared<Mesh>();
    bool meshLoaded = iceMesh->loadOBJ(objFilename);

    if (!meshLoaded) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    std::cout << objFilename << " loaded successfully." << std::endl;

    //==========================================================================
    // 氷サイズ設定
    //==========================================================================
    double targetSize = 500.0;
    double floorY = -500.0;
    std::cout << "Ice size: " << targetSize << " units" << std::endl;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    // Y軸周りに回転
    iceMesh->rotateY(75.0);

    // 床に接地
    double meshMinY = iceMesh->getMinY();
    iceMesh->translate(Eigen::Vector3d(0, floorY - meshMinY, 0));

    std::cout << "Bounding box after transform:" << std::endl;
    std::cout << "  Min: " << iceMesh->bboxMin.transpose() << std::endl;
    std::cout << "  Max: " << iceMesh->bboxMax.transpose() << std::endl;

    //==========================================================================
    // 二重正規分布による方向性気泡を生成
    //==========================================================================
    double bubbleElongation = 2.0;
    std::vector<DirectionalBubble> directionalBubbles =
        generateDirectionalBubbles(iceMesh->bboxMin, iceMesh->bboxMax,
                                   bubbleElongation, 42);

    //==========================================================================
    // シーン構築
    //==========================================================================
    const auto room_r = 1e7;
    std::vector<Body> bodies;

    // 部屋の壁
    bodies.emplace_back(
        Sphere(room_r, (room_r - 800) * Eigen::Vector3d::UnitX()),
        Material(codeToColor("#a0522d"), 0.8, 0.0)
    );
    bodies.emplace_back(
        Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitX()),
        Material(codeToColor("#4682b4"), 0.8, 0.0)
    );
    bodies.emplace_back(
        Sphere(room_r, (room_r - 500) * Eigen::Vector3d::UnitY()),
        Material(codeToColor("#b8b8b8"), 0.8, 0.0)
    );
    bodies.emplace_back(
        Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitY()),
        Material(codeToColor("#d8d8d8"), 0.8, 0.0)
    );
    bodies.emplace_back(
        Sphere(room_r, (room_r - 1200) * Eigen::Vector3d::UnitZ()),
        Material(codeToColor("#2e8b57"), 0.8, 0.0)
    );

    // 氷メッシュ（Glassマテリアル + Microfacet）
    const double ice_ior = 1.31;
    const double ice_alpha = 0.05;

    bodies.emplace_back(
        iceMesh,
        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0, ice_alpha)
    );

    //==========================================================================
    // 方向性気泡を追加（空気の楕円体）
    //==========================================================================
    const double air_ior = 1.0;
    for (const auto &db : directionalBubbles) {
        bodies.emplace_back(
            Ellipsoid(db.center, db.radii),
            Material(Color(1.0, 1.0, 1.0), air_ior, MaterialType::Glass, 0.0)
        );
    }

    // 光源
    bodies.emplace_back(
        Sphere(200, Eigen::Vector3d(0, 700, 0)),
        Material(Color(1, 1, 1), 1.0, 20)
    );

    std::cout << "\nScene composition:" << std::endl;
    std::cout << "  - Room walls: 5" << std::endl;
    std::cout << "  - Ice mesh: 1" << std::endl;
    std::cout << "  - Directional bubbles: " << directionalBubbles.size() << std::endl;
    std::cout << "  - Light sources: 1" << std::endl;
    std::cout << "Total objects in scene: " << bodies.size() << std::endl;

    //==========================================================================
    // カメラ設定
    //==========================================================================
    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 400, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);

    std::cout << "Camera position: " << campos.transpose() << std::endl;
    std::cout << "Camera looking at: " << meshCenter.transpose() << std::endl;

    // レンダラー設定
    const unsigned int maxDepth = 20;
    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), maxDepth);

    //==========================================================================
    // Hero Wavelength Spectral パストレーシング
    //==========================================================================
    std::cout << "\nStarting Hero Wavelength Spectral rendering..." << std::endl;
    const unsigned int samples = 3000;
    std::cout << "Samples per pixel: " << samples << std::endl;
    std::cout << "Resolution: " << camera.getFilm().resolution.x() << " x "
              << camera.getFilm().resolution.y() << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    const auto image = renderer.spectralRenderHero(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Rendering completed in " << duration.count() << " seconds." << std::endl;

    image.save("ice_spectral_hero.png");
    std::cout << "Saved: ice_spectral_hero.png" << std::endl;
}

//==============================================================================
// RGBモード（比較用）
//==============================================================================
void iceRenderingWithMesh(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "=== Ice Rendering with OBJ Mesh (RGB Mode) ===" << std::endl;

    auto iceMesh = std::make_shared<Mesh>();
    bool meshLoaded = iceMesh->loadOBJ(objFilename);

    if (!meshLoaded) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    double targetSize = 500.0;
    double floorY = -500.0;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);
    iceMesh->rotateY(75.0);

    double meshMinY = iceMesh->getMinY();
    iceMesh->translate(Eigen::Vector3d(0, floorY - meshMinY, 0));

    // 方向性気泡
    double bubbleElongation = 2.0;
    std::vector<DirectionalBubble> directionalBubbles =
        generateDirectionalBubbles(iceMesh->bboxMin, iceMesh->bboxMax,
                                   bubbleElongation, 42);

    const auto room_r = 1e7;
    std::vector<Body> bodies;

    bodies.emplace_back(Sphere(room_r, (room_r - 800) * Eigen::Vector3d::UnitX()),
                        Material(codeToColor("#a0522d"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitX()),
                        Material(codeToColor("#4682b4"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, (room_r - 500) * Eigen::Vector3d::UnitY()),
                        Material(codeToColor("#b8b8b8"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitY()),
                        Material(codeToColor("#d8d8d8"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, (room_r - 1200) * Eigen::Vector3d::UnitZ()),
                        Material(codeToColor("#2e8b57"), 0.8, 0.0));

    const double ice_ior = 1.31;
    const double ice_alpha = 0.05;
    bodies.emplace_back(iceMesh,
                        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0, ice_alpha));

    const double air_ior = 1.0;
    for (const auto &db : directionalBubbles) {
        bodies.emplace_back(Ellipsoid(db.center, db.radii),
                            Material(Color(1.0, 1.0, 1.0), air_ior, MaterialType::Glass, 0.0));
    }

    bodies.emplace_back(Sphere(200, Eigen::Vector3d(0, 700, 0)),
                        Material(Color(1, 1, 1), 1.0, 20));

    std::cout << "Total objects in scene: " << bodies.size() << std::endl;

    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 400, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);

    const unsigned int maxDepth = 20;
    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), maxDepth);

    std::cout << "Starting path tracing (RGB mode)..." << std::endl;
    const unsigned int samples = 500;

    auto start = std::chrono::high_resolution_clock::now();

    const auto image = renderer.pathTracingRender(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Rendering completed in " << duration.count() << " seconds." << std::endl;

    image.save("ice_rgb_result.png");
    std::cout << "Saved: ice_rgb_result.png" << std::endl;
}

//==============================================================================
// Main
//==============================================================================
int main(int argc, char* argv[]) {
    std::cout << "============================================" << std::endl;
    std::cout << "Ice Rendering with OBJ Mesh" << std::endl;
    std::cout << "Based on:" << std::endl;
    std::cout << "  - Ghafari & Park (2017)" << std::endl;
    std::cout << "  - Kim (2025): Directional Air Bubbles" << std::endl;
    std::cout << "+ Hero Wavelength Sampling (~7x faster)" << std::endl;
    std::cout << "+ BVH Acceleration" << std::endl;
    std::cout << "============================================" << std::endl;

    std::string objFile = "ice.obj";
    std::string csvFile = "";

    if (argc > 1) {
        std::string mode = argv[1];

        if (mode == "rgb") {
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingWithMesh(objFile, csvFile);
        } else if (mode == "spectral") {
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingSpectral(objFile, csvFile);
        } else if (mode == "help") {
            std::cout << "Usage: " << argv[0] << " [mode] [obj_file] [csv_file]" << std::endl;
            std::cout << "Modes:" << std::endl;
            std::cout << "  spectral [obj] [csv]  - Hero Wavelength Spectral (default)" << std::endl;
            std::cout << "  rgb [obj] [csv]       - RGB rendering (for comparison)" << std::endl;
            std::cout << "\nBubble distributions:" << std::endl;
            std::cout << "  1. Random direction (center concentrated)" << std::endl;
            std::cout << "  2. Toward center (spread out)" << std::endl;
            std::cout << "  3. Tiny isotropic (cloudiness effect)" << std::endl;
        } else {
            objFile = mode;
            if (argc > 2) csvFile = argv[2];
            iceRenderingSpectral(objFile, csvFile);
        }
    } else {
        std::cout << "\nUsing default: Hero Wavelength Spectral rendering" << std::endl;
        iceRenderingSpectral(objFile, csvFile);
    }

    return 0;
}