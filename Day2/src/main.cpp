//
// Ice Rendering with OBJ Mesh
// Based on "Realistic Rendering of Ice and Crack Propagations" (2017)
//
// 【変更点】氷サイズを30→500に拡大（青色検証用）
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

        // バウンディングボックス内かチェック
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
// ice.obj を氷としてレンダリング（RGBモード）
//==============================================================================
void iceRenderingWithMesh(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "=== Ice Rendering with OBJ Mesh (NEE Enabled) ===" << std::endl;

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

    // ============================================
    // 【変更】氷サイズを大きく設定（青色検証用）
    // 元の値: 30.0 → 新しい値: 500.0
    // 単位を仮にcmとすると、5メートルの氷塊
    // ============================================
    double targetSize = 1200.0;  // 変更前: 30.0
    std::cout << "\n*** ICE SIZE: " << targetSize << " (changed for blue color test) ***\n" << std::endl;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    // 床に接地（床の位置も調整）
    double floorY = -500.0;  // 変更前: -30.0
    double meshMinY = iceMesh->getMinY();
    iceMesh->translate(Eigen::Vector3d(0, floorY - meshMinY, 0));

    std::cout << "Scaled bounding box: " << std::endl;
    std::cout << "  Min: " << iceMesh->bboxMin.transpose() << std::endl;
    std::cout << "  Max: " << iceMesh->bboxMax.transpose() << std::endl;

    // 気泡を生成または読み込み
    std::vector<BubbleData> bubbles;

    if (!csvFilename.empty()) {
        bubbles = BubbleLoader::loadFromCSV(csvFilename);
        // CSVの気泡をメッシュのスケールに合わせる
        Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
        bubbles = BubbleLoader::scaleBubbles(bubbles, scale);
        bubbles = BubbleLoader::translateBubbles(bubbles, meshCenter);
        bubbles = BubbleLoader::filterBubblesInBox(bubbles, iceMesh->bboxMin, iceMesh->bboxMax, 0.5);
    } else {
        bubbles = generateBubblesInMesh(iceMesh->bboxMin, iceMesh->bboxMax);
    }

    // シーン構築（部屋のサイズも拡大）
    const auto room_r = 1e7;
    std::vector<Body> bodies;

    // 部屋の壁（拡散マテリアル）- サイズ拡大
    // 右壁
    bodies.emplace_back(
        Sphere(room_r, (room_r - 800) * Eigen::Vector3d::UnitX()),
        Material(codeToColor("#a0522d"), 0.8, 0.0)
    );
    // 左壁
    bodies.emplace_back(
        Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitX()),
        Material(codeToColor("#4682b4"), 0.8, 0.0)
    );
    // 床
    bodies.emplace_back(
        Sphere(room_r, (room_r - 500) * Eigen::Vector3d::UnitY()),
        Material(codeToColor("#b8b8b8"), 0.8, 0.0)
    );
    // 天井
    bodies.emplace_back(
        Sphere(room_r, -(room_r - 800) * Eigen::Vector3d::UnitY()),
        Material(codeToColor("#d8d8d8"), 0.8, 0.0)
    );
    // 奥の壁
    bodies.emplace_back(
        Sphere(room_r, (room_r - 1200) * Eigen::Vector3d::UnitZ()),
        Material(codeToColor("#2e8b57"), 0.8, 0.0)
    );

    // 氷メッシュ（Glassマテリアル）
    const double ice_ior = 1.31;
    bodies.emplace_back(
        iceMesh,
        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0)
    );

    // 気泡を追加（空気球体）
    const double air_ior = 1.0;
    for (const auto &bubble : bubbles) {
        bodies.emplace_back(
            Sphere(bubble.radius, bubble.center),
            Material(Color(1.0, 1.0, 1.0), air_ior, MaterialType::Glass, 0.0)
        );
    }

    // 光源（サイズと位置を調整）
    bodies.emplace_back(
        Sphere(200, Eigen::Vector3d(0, 700, 0)),  // 変更前: Sphere(12, (0, 44, 0))
        Material(Color(1, 1, 1), 1.0, 20)
    );
    bodies.emplace_back(
        Sphere(100, Eigen::Vector3d(-400, 550, 400)),  // 変更前: Sphere(6, (-25, 35, 25))
        Material(Color(0.9, 0.9, 1.0), 1.0, 8)
    );

    std::cout << "Total objects in scene: " << bodies.size() << std::endl;

    // カメラ設定（距離を調整）
    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 300, 1300);  // 変更前: (0, +20, 80)
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);

    // レンダラー設定
    const unsigned int maxDepth = 20;
    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), maxDepth);

    // パストレーシング
    std::cout << "Starting path tracing with NEE..." << std::endl;
    const unsigned int samples = 500;
    std::cout << "Samples per pixel: " << samples << std::endl;
    std::cout << "Resolution: " << camera.getFilm().resolution.x() << " x "
              << camera.getFilm().resolution.y() << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    const auto image = renderer.pathTracingRender(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Rendering completed in " << duration.count() << " seconds." << std::endl;

    image.save("ice_mesh_result_large.png");
    std::cout << "Saved: ice_mesh_result_large.png" << std::endl;
}

//==============================================================================
// 気泡なしのシンプルレンダリング（テスト用）
//==============================================================================
void iceRenderingSimple(const std::string &objFilename) {
    std::cout << "=== Ice Rendering (Simple - No Bubbles) ===" << std::endl;

    auto iceMesh = std::make_shared<Mesh>();
    bool meshLoaded = iceMesh->loadOBJ(objFilename);

    if (!meshLoaded) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    // ============================================
    // 【変更】氷サイズを大きく設定（青色検証用）
    // ============================================
    double targetSize = 500.0;  // 変更前: 30.0
    std::cout << "\n*** ICE SIZE: " << targetSize << " (changed for blue color test) ***\n" << std::endl;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    double floorY = -500.0;  // 変更前: -30.0
    iceMesh->translate(Eigen::Vector3d(0, floorY - iceMesh->getMinY(), 0));

    const auto room_r = 1e7;
    std::vector<Body> bodies;

    // 壁（サイズ拡大）
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

    // 氷
    const double ice_ior = 1.31;
    bodies.emplace_back(iceMesh,
                        Material(Color(0.98, 0.98, 1.0), ice_ior, MaterialType::Glass, 0.0));

    // 光源
    bodies.emplace_back(Sphere(200, Eigen::Vector3d(0, 700, 0)),
                        Material(Color(1, 1, 1), 1.0, 20));

    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 300, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 360, 4.0 / 3.0, 55, 35);

    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), 20);

    const unsigned int samples = 200;
    std::cout << "Samples per pixel: " << samples << std::endl;

    auto start = std::chrono::high_resolution_clock::now();
    const auto image = renderer.pathTracingRender(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);

    std::cout << "Rendering completed in " << duration.count() << " seconds." << std::endl;
    image.save("ice_simple_result_large.png");
    std::cout << "Saved: ice_simple_result_large.png" << std::endl;
}

//==============================================================================
// Spectral Rendering（波長依存レンダリング）- 青色検証用
//==============================================================================
void iceRenderingSpectral(const std::string &objFilename, const std::string &csvFilename = "") {
    std::cout << "=== Spectral Ice Rendering (Wavelength-dependent) ===" << std::endl;
    std::cout << "This mode uses wavelength-dependent IOR for realistic dispersion." << std::endl;
    std::cout << "\n*** NOTE: Current implementation has wavelength-dependent REFRACTION" << std::endl;
    std::cout << "*** but does NOT have wavelength-dependent ABSORPTION (Beer-Lambert)." << std::endl;
    std::cout << "*** Blue color from ice requires absorption, which is not implemented yet.\n" << std::endl;

    // ice.objを読み込む
    auto iceMesh = std::make_shared<Mesh>();
    bool meshLoaded = iceMesh->loadOBJ(objFilename);

    if (!meshLoaded) {
        std::cerr << "Error: Failed to load " << objFilename << std::endl;
        return;
    }

    std::cout << objFilename << " loaded successfully." << std::endl;

    // ============================================
    // 【変更】氷サイズを大幅に拡大（青色検証用）
    // 元の値: 30.0 → 新しい値: 500.0
    // これは約5メートルのスケール（単位がcmの場合）
    // 氷河の青色が見えるには通常10m以上必要
    // ============================================
    double targetSize = 800.0;  // 変更前: 30.0
    std::cout << "\n========================================" << std::endl;
    std::cout << "*** ICE SIZE: " << targetSize << " ***" << std::endl;
    std::cout << "*** (Original was 30.0, now ~17x larger) ***" << std::endl;
    std::cout << "========================================\n" << std::endl;

    Eigen::Vector3d originalSize = iceMesh->bboxMax - iceMesh->bboxMin;
    double scale = targetSize / originalSize.maxCoeff();
    iceMesh->scale(scale);

    // 床に接地（床の位置も調整）
    double floorY = -400.0;  // 変更前: -30.0
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

    // シーン構築（部屋のサイズも拡大）
    const auto room_r = 1e7;
    std::vector<Body> bodies;

    // 部屋の壁（サイズ拡大）
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
    const double ice_alpha = 0.05;  // マイクロファセット粗さ（0.05〜0.15推奨）

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

    // 光源（サイズと位置を調整）
    bodies.emplace_back(
        Sphere(200, Eigen::Vector3d(0, 700, 0)),
        Material(Color(1, 1, 1), 1.0, 20)
    );

    std::cout << "Total objects in scene: " << bodies.size() << std::endl;

    // カメラ設定（距離を調整）
    Eigen::Vector3d meshCenter = (iceMesh->bboxMin + iceMesh->bboxMax) / 2.0;
    const Eigen::Vector3d campos(0, meshCenter.y() + 300, 1300);
    const Eigen::Vector3d camdir = meshCenter - campos;
    const Camera camera(campos, camdir, 480, 4.0 / 3.0, 55, 35);

    // レンダラー設定
    const unsigned int maxDepth = 20;
    const Renderer renderer(bodies, camera, Color(0.02, 0.02, 0.05), maxDepth);

    // Spectralパストレーシング
    std::cout << "Starting Spectral path tracing..." << std::endl;
    const unsigned int samples = 600;
    std::cout << "Samples per pixel: " << samples << std::endl;
    std::cout << "Wavelengths: " << NUM_WAVELENGTHS << " (400-700nm)" << std::endl;
    std::cout << "Resolution: " << camera.getFilm().resolution.x() << " x "
              << camera.getFilm().resolution.y() << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    const auto image = renderer.spectralRender(samples)
                              .apply_reinhard_extended_tone_mapping()
                              .apply_gamma_correction();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Rendering completed in " << duration.count() << " seconds." << std::endl;

    image.save("ice_spectral_result_large.png");
    std::cout << "Saved: ice_spectral_result_large.png" << std::endl;
}

//==============================================================================
// Main
//==============================================================================
int main(int argc, char* argv[]) {
    std::cout << "============================================" << std::endl;
    std::cout << "Ice Rendering with OBJ Mesh" << std::endl;
    std::cout << "Based on \"Realistic Rendering of Ice and Crack Propagations\"" << std::endl;
    std::cout << "With Next Event Estimation (NEE)" << std::endl;
    std::cout << "+ Spectral Rendering Support" << std::endl;
    std::cout << "+ BVH Acceleration" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "\n*** LARGE ICE SIZE MODE (500 units) ***" << std::endl;
    std::cout << "*** For testing blue color hypothesis ***\n" << std::endl;

    std::string objFile = "ice.obj";
    std::string csvFile = "";

    if (argc > 1) {
        std::string mode = argv[1];

        if (mode == "simple") {
            if (argc > 2) objFile = argv[2];
            iceRenderingSimple(objFile);
        } else if (mode == "bubbles") {
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingWithMesh(objFile, csvFile);
        } else if (mode == "spectral") {
            // Spectral Rendering モード
            if (argc > 2) objFile = argv[2];
            if (argc > 3) csvFile = argv[3];
            iceRenderingSpectral(objFile, csvFile);
        } else if (mode == "help") {
            std::cout << "Usage: " << argv[0] << " [mode] [obj_file] [csv_file]" << std::endl;
            std::cout << "Modes:" << std::endl;
            std::cout << "  simple [obj]          - Ice without bubbles (test)" << std::endl;
            std::cout << "  bubbles [obj] [csv]   - Ice with bubbles (RGB rendering)" << std::endl;
            std::cout << "  spectral [obj] [csv]  - Ice with spectral rendering (wavelength-dependent IOR)" << std::endl;
            std::cout << "  (default)             - Ice with generated bubbles (RGB rendering)" << std::endl;
        } else {
            // モード指定なしの場合はOBJファイル名として扱う
            objFile = mode;
            iceRenderingWithMesh(objFile, csvFile);
        }
    } else {
        std::cout << "\nUsing default: ice.obj with generated bubbles" << std::endl;
        iceRenderingWithMesh(objFile, csvFile);
    }

    return 0;
}