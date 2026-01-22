//
// Ice Rendering with OBJ Mesh
// Based on "Realistic Rendering of Ice and Crack Propagations" (2017)
// + Hero Wavelength Sampling for efficient spectral rendering
//

#include <iostream>
#include <chrono>
#include <random>
#include "Body.h"
#include "Camera.h"
#include "Renderer.h"
#include "Mesh.h"
#include "BubbleLoader.h"

//==============================================================================
// 気泡をプログラムで生成（メッシュのバウンディングボックス内）
//==============================================================================
std::vector<BubbleData> generateBubblesInMesh(
        const Eigen::Vector3d &bboxMin,
        const Eigen::Vector3d &bboxMax,
        int seed = 42) {

    std::vector<BubbleData> bubbles;
    std::mt19937_64 engine(seed);
    std::normal_distribution<double> dist(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Eigen::Vector3d center = (bboxMin + bboxMax) / 2.0;
    Eigen::Vector3d size = bboxMax - bboxMin;
    double minDim = size.minCoeff();

    // 分布1: 中心に集中した気泡
    const int num_center = 200;
    const double pos_std_center = minDim * 0.15;
    const double radius_mean_center = minDim * 0.015;
    const double radius_std_center = minDim * 0.005;

    for (int i = 0; i < num_center; ++i) {
        Eigen::Vector3d pos(dist(engine) * pos_std_center,
                           dist(engine) * pos_std_center,
                           dist(engine) * pos_std_center);
        pos += center;

        double radius = dist(engine) * radius_std_center + radius_mean_center;
        radius = std::max(minDim * 0.005, std::min(radius, minDim * 0.04));

        bool inside = true;
        for (int j = 0; j < 3; ++j) {
            if (pos[j] - radius < bboxMin[j] + minDim * 0.05 ||
                pos[j] + radius > bboxMax[j] - minDim * 0.05) {
                inside = false;
                break;
            }
        }

        if (inside) {
            bubbles.emplace_back(pos, radius);
        }
    }

    // 分布2: 全体に散らばった気泡
    const int num_spread = 100;
    const double pos_std_spread = minDim * 0.3;
    const double radius_mean_spread = minDim * 0.02;
    const double radius_std_spread = minDim * 0.008;

    for (int i = 0; i < num_spread; ++i) {
        Eigen::Vector3d pos(dist(engine) * pos_std_spread,
                           dist(engine) * pos_std_spread,
                           dist(engine) * pos_std_spread);
        pos += center;

        double radius = dist(engine) * radius_std_spread + radius_mean_spread;
        radius = std::max(minDim * 0.005, std::min(radius, minDim * 0.05));

        bool inside = true;
        for (int j = 0; j < 3; ++j) {
            if (pos[j] - radius < bboxMin[j] + minDim * 0.05 ||
                pos[j] + radius > bboxMax[j] - minDim * 0.05) {
                inside = false;
                break;
            }
        }

        if (inside) {
            bubbles.emplace_back(pos, radius);
        }
    }

    std::cout << "Generated " << bubbles.size() << " bubbles inside mesh" << std::endl;
    return bubbles;
}

//==============================================================================
// Spectral Rendering with Hero Wavelength Sampling
// 効率化されたスペクトルレンダリング
//==============================================================================
void iceRenderingSpectral(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "============================================" << std::endl;
    std::cout << "=== Hero Wavelength Spectral Rendering ===" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "Efficiency: ~7x faster than traditional 7-wavelength sampling" << std::endl;
    std::cout << "Features: Beer-Lambert absorption, Microfacet model" << std::endl;
    std::cout << std::endl;

    // ice.objを読み込む
    auto iceMesh = std::make_shared<Mesh>();
    bool meshLoaded = iceMesh->loadOBJ(objFilename);

    if (!meshLoaded) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    std::cout << objFilename << " loaded successfully." << std::endl;
    std::cout << "Original bounding box: " << std::endl;
    std::cout << "  Min: " << iceMesh->bboxMin.transpose() << std::endl;
    std::cout << "  Max: " << iceMesh->bboxMax.transpose() << std::endl;

    // 氷サイズの設定
    double targetSize = 500.0;
    std::cout << "\n*** ICE SIZE: " << targetSize << " ***\n" << std::endl;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    // 床に接地
    double floorY = -500.0;
    double meshMinY = iceMesh->getMinY();
    iceMesh->translate(Eigen::Vector3d(0, floorY - meshMinY, 0));

    std::cout << "Scaled bounding box: " << std::endl;
    std::cout << "  Min: " << iceMesh->bboxMin.transpose() << std::endl;
    std::cout << "  Max: " << iceMesh->bboxMax.transpose() << std::endl;

    // 気泡を生成または読み込み
    std::vector<BubbleData> bubbles;

    if (!csvFilename.empty()) {
        bubbles = BubbleLoader::loadFromCSV(csvFilename);
        Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
        bubbles = BubbleLoader::scaleBubbles(bubbles, scale);
        bubbles = BubbleLoader::translateBubbles(bubbles, meshCenter);
        bubbles = BubbleLoader::filterBubblesInBox(bubbles, iceMesh->bboxMin, iceMesh->bboxMax, 0.5);
    } else {
        bubbles = generateBubblesInMesh(iceMesh->bboxMin, iceMesh->bboxMax);
    }

    // シーン構築
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
    const double ice_alpha = 0.05;  // マイクロファセット粗さ

    bodies.emplace_back(
        iceMesh,
        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0, ice_alpha)
    );

    // 気泡を追加（空気球体）
    const double air_ior = 1.0;
    for (const auto &bubble : bubbles) {
        bodies.emplace_back(
            Sphere(bubble.radius, bubble.center),
            Material(Color(1.0, 1.0, 1.0), air_ior, MaterialType::Glass, 0.0)
        );
    }

    // 光源
    bodies.emplace_back(
        Sphere(200, Eigen::Vector3d(0, 700, 0)),
        Material(Color(1, 1, 1), 1.0, 50)
    );

    std::cout << "Total objects in scene: " << bodies.size() << std::endl;
    std::cout << "  - Walls: 5" << std::endl;
    std::cout << "  - Ice mesh: 1" << std::endl;
    std::cout << "  - Bubbles: " << bubbles.size() << std::endl;
    std::cout << "  - Lights: 1" << std::endl;

    // カメラ設定
    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 300, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);




    // レンダラー設定
    const unsigned int maxDepth = 30;
    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), maxDepth);

    // Hero Wavelength Spectral レンダリング
    std::cout << "\n=== Starting Hero Wavelength Spectral Rendering ===" << std::endl;
    const unsigned int samples = 500;
    std::cout << "Samples per pixel: " << samples << std::endl;
    std::cout << "Resolution: " << camera.getFilm().resolution.x() << " x "
              << camera.getFilm().resolution.y() << std::endl;
    std::cout << "Max depth: " << maxDepth << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    // Hero Wavelength Samplingを使用
    const auto image = renderer.spectralRenderHero(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "\nRendering completed in " << duration.count() << " seconds." << std::endl;

    image.save("ice_spectral_hero.png");
    std::cout << "Saved: ice_spectral_hero.png" << std::endl;
}

//==============================================================================
// RGBパストレーシング（比較用）
//==============================================================================
void iceRenderingRGB(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "=== RGB Path Tracing Mode ===" << std::endl;

    auto iceMesh = std::make_shared<Mesh>();
    if (!iceMesh->loadOBJ(objFilename)) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    double targetSize = 300.0;
    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    double floorY = -500.0;
    iceMesh->translate(Eigen::Vector3d(0, floorY - iceMesh->getMinY(), 0));

    std::vector<BubbleData> bubbles;
    if (!csvFilename.empty()) {
        bubbles = BubbleLoader::loadFromCSV(csvFilename);
        Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
        bubbles = BubbleLoader::scaleBubbles(bubbles, scale);
        bubbles = BubbleLoader::translateBubbles(bubbles, meshCenter);
        bubbles = BubbleLoader::filterBubblesInBox(bubbles, iceMesh->bboxMin, iceMesh->bboxMax, 0.5);
    } else {
        bubbles = generateBubblesInMesh(iceMesh->bboxMin, iceMesh->bboxMax);
    }

    const auto room_r = 1e7;
    std::vector<Body> bodies;

    bodies.emplace_back(Sphere(room_r, (room_r - 800) * Eigen::Vector3d::UnitX()),
                        Material(codeToColor("#faeded"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitX()),
                        Material(codeToColor("#faeded"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, (room_r - 500) * Eigen::Vector3d::UnitY()),
                        Material(codeToColor("#faeded"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitY()),
                        Material(codeToColor("#a0522d"), 0.8, 0.0));
    bodies.emplace_back(Sphere(room_r, (room_r - 1200) * Eigen::Vector3d::UnitZ()),
                        Material(codeToColor("#faeded"), 0.8, 0.0));

    const double ice_ior = 1.31;
    const double ice_alpha = 0.05;
    bodies.emplace_back(iceMesh,
                        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0, ice_alpha));

    const double air_ior = 1.0;
    for (const auto &bubble : bubbles) {
        bodies.emplace_back(Sphere(bubble.radius, bubble.center),
                            Material(Color(1.0, 1.0, 1.0), air_ior, MaterialType::Glass, 0.0));
    }

    bodies.emplace_back(Sphere(200, Eigen::Vector3d(0, 700, 0)),
                        Material(Color(1, 1, 1), 1.0, 18));

    std::cout << "Total objects: " << bodies.size() << std::endl;

    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 300, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);

    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), 20);

    const unsigned int samples = 500;
    std::cout << "Samples per pixel: " << samples << std::endl;

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
    std::cout << "Ice Rendering with Hero Wavelength Sampling" << std::endl;
    std::cout << "Based on \"Realistic Rendering of Ice\"" << std::endl;
    std::cout << "+ Hero Wavelength Sampling (~7x speedup)" << std::endl;
    std::cout << "+ BVH Acceleration" << std::endl;
    std::cout << "+ Beer-Lambert Absorption" << std::endl;
    std::cout << "+ Microfacet Surface Model" << std::endl;
    std::cout << "============================================" << std::endl;

    std::string objFile = "ice.obj";
    std::string csvFile = "";

    if (argc > 1) {
        std::string mode = argv[1];

        if (mode == "spectral" || mode == "hero") {
            // Hero Wavelength Spectral Rendering（デフォルト）
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingSpectral(objFile, csvFile);
        } else if (mode == "rgb") {
            // RGB Path Tracing（比較用）
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingRGB(objFile, csvFile);
        } else if (mode == "help") {
            std::cout << "\nUsage: " << argv[0] << " [mode] [obj_file] [csv_file]" << std::endl;
            std::cout << "\nModes:" << std::endl;
            std::cout << "  spectral [obj] [csv] - Hero Wavelength Spectral (default, recommended)" << std::endl;
            std::cout << "  hero [obj] [csv]     - Same as spectral" << std::endl;
            std::cout << "  rgb [obj] [csv]      - RGB Path Tracing (for comparison)" << std::endl;
            std::cout << "\nExamples:" << std::endl;
            std::cout << "  " << argv[0] << " spectral ice.obj" << std::endl;
            std::cout << "  " << argv[0] << " spectral ice.obj bubbles.csv" << std::endl;
            std::cout << "  " << argv[0] << " rgb ice.obj" << std::endl;
        } else {
            // 引数をOBJファイル名として扱う
            objFile = mode;
            iceRenderingSpectral(objFile, csvFile);
        }
    } else {
        // デフォルト: Hero Wavelength Spectral Rendering
        std::cout << "\nUsing default: ice.obj with Hero Wavelength Spectral Rendering" << std::endl;
        iceRenderingSpectral(objFile, csvFile);
    }

    return 0;
}