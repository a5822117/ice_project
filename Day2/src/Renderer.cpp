//
// Renderer.cpp
// Path Tracing Renderer with NEE and BVH Acceleration
// Extended for ice/glass rendering
// + Beer-Lambert absorption for realistic blue ice
// + Microfacet model for realistic ice surface (Ghafari & Park 2017)
// + Hero Wavelength Sampling for ~7x faster spectral rendering
//

#include "Renderer.h"
#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <chrono>

//==============================================================================
// ユーティリティ関数
//==============================================================================

inline Color clampRadiance(const Color &color, double maxLuminance = 100.0) {
    double luminance = 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z();
    if (luminance > maxLuminance) {
        return color * (maxLuminance / luminance);
    }
    return color;
}

inline bool isValidColor(const Color &color) {
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(color[i]) || std::isinf(color[i]) || color[i] < 0.0) {
            return false;
        }
    }
    return true;
}

inline Color sanitizeColor(const Color &color) {
    Color result;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(color[i]) || std::isinf(color[i]) || color[i] < 0.0) {
            result[i] = 0.0;
        } else {
            result[i] = color[i];
        }
    }
    return result;
}

inline double sanitizeRadiance(double radiance, double maxRadiance = 50.0) {
    if (std::isnan(radiance) || std::isinf(radiance) || radiance < 0.0) {
        return 0.0;
    }
    return std::min(radiance, maxRadiance);
}

//==============================================================================
// Renderer 実装
//==============================================================================

Renderer::Renderer(const std::vector<Body> &bodies, Camera camera, Color bgColor, unsigned int maxDepth)
    : bodies(bodies), camera(std::move(camera)), bgColor(std::move(bgColor)),
      engine(0), dist(0, 1),
      maxDepth(maxDepth) {

    // 光源インデックスを構築
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isLight()) {
            lightIndices.push_back(static_cast<int>(i));
        }
    }

    if (!lightIndices.empty()) {
        std::cout << "Found " << lightIndices.size() << " light source(s) for NEE" << std::endl;
    }

    // SceneBVHを構築
    std::cout << "Building Scene BVH for " << bodies.size() << " objects..." << std::endl;
    auto bvhStart = std::chrono::high_resolution_clock::now();

    sceneBVH.build(this->bodies);

    auto bvhEnd = std::chrono::high_resolution_clock::now();
    auto bvhDuration = std::chrono::duration_cast<std::chrono::milliseconds>(bvhEnd - bvhStart);
    std::cout << "Scene BVH built in " << bvhDuration.count() << " ms" << std::endl;
}

void Renderer::rebuildBVH() {
    sceneBVH.build(bodies);
}

double Renderer::rand() const {
    return dist(engine);
}

bool Renderer::hitScene(const Ray &ray, RayHit &hit) const {
    if (useBVH && sceneBVH.root) {
        return sceneBVH.intersect(bodies, ray, hit);
    }

    // フォールバック：線形探索
    hit.t = DBL_MAX;
    hit.idx = -1;
    for (size_t i = 0; i < bodies.size(); ++i) {
        RayHit _hit;
        if (bodies[i].hit(ray, _hit) && _hit.t < hit.t) {
            hit.t = _hit.t;
            hit.idx = static_cast<int>(i);
            hit.point = _hit.point;
            hit.normal = _hit.normal;
        }
    }
    return hit.idx != -1;
}

Image Renderer::render() const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());
    for (int p_y = 0; p_y < image.height; p_y++) {
        for (int p_x = 0; p_x < image.width; p_x++) {
            const int p_idx = p_y * image.width + p_x;
            Ray ray;
            RayHit hit;
            camera.filmView(p_x, p_y, ray);
            image.pixels[p_idx] = hitScene(ray, hit) ? bodies[hit.idx].material.color : bgColor;
        }
    }
    return image;
}

Image Renderer::directIlluminationRender(const unsigned int &samples) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());
#pragma omp parallel for schedule(dynamic, 1)
    for (int p_y = 0; p_y < image.height; p_y++) {
        for (int p_x = 0; p_x < image.width; p_x++) {
            const int p_idx = p_y * image.width + p_x;
            Ray ray;
            RayHit hit;
            camera.filmView(p_x, p_y, ray);

            if (hitScene(ray, hit)) {
                if (bodies[hit.idx].isLight()) {
                    image.pixels[p_idx] = bodies[hit.idx].getEmission();
                } else {
                    Color reflectRadiance = Color::Zero();
                    for (unsigned int i = 0; i < samples; ++i) {
                        Eigen::Vector3d outDir;
                        double pdf;
                        cosineSample(hit.normal, outDir, pdf);
                        Ray _ray(hit.point + hit.normal * 1e-4, outDir);
                        RayHit _hit;
                        if (hitScene(_ray, _hit) && bodies[_hit.idx].isLight()) {
                            reflectRadiance += bodies[hit.idx].getKd().cwiseProduct(bodies[_hit.idx].getEmission());
                        }
                    }
                    image.pixels[p_idx] = reflectRadiance / static_cast<double>(samples);
                }
            } else {
                image.pixels[p_idx] = bgColor;
            }
        }
    }
    return image;
}

//==============================================================================
// Path Tracing with NEE (BVH Accelerated)
//==============================================================================

Image Renderer::pathTracingRender(const unsigned int &samplesPerPixel) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());

    int totalPixels = image.height;
    int progressInterval = std::max(1, totalPixels / 20);
    const double maxRadiance = 50.0;

    std::cout << "BVH Acceleration: " << (useBVH ? "ENABLED" : "DISABLED") << std::endl;

#pragma omp parallel for schedule(dynamic, 1)
    for (int p_y = 0; p_y < image.height; p_y++) {
        if (p_y % progressInterval == 0) {
#pragma omp critical
            {
                std::cout << "Progress: " << (p_y * 100 / totalPixels) << "%" << std::endl;
            }
        }

        for (int p_x = 0; p_x < image.width; p_x++) {
            const int p_idx = p_y * image.width + p_x;
            Color accumulatedColor = Color::Zero();

            for (unsigned int s = 0; s < samplesPerPixel; ++s) {
                Ray ray;
                camera.filmView(p_x, p_y, ray);

                Color sampleColor = tracePath(ray, 0, false, 1.0, true);
                sampleColor = sanitizeColor(sampleColor);
                sampleColor = clampRadiance(sampleColor, maxRadiance);

                accumulatedColor += sampleColor;
            }

            image.pixels[p_idx] = accumulatedColor / static_cast<double>(samplesPerPixel);
        }
    }

    std::cout << "Progress: 100%" << std::endl;
    return image;
}

//==============================================================================
// Spectral Rendering（従来版：全波長を毎回計算）
//==============================================================================

Image Renderer::spectralRender(const unsigned int &samplesPerPixel) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());

    int totalPixels = image.height;
    int progressInterval = std::max(1, totalPixels / 20);
    const double maxRadiance = 50.0;

    std::cout << "=== Spectral Rendering Mode (Original - All Wavelengths) ===" << std::endl;
    std::cout << "WARNING: This mode is slower. Consider using spectralRenderHero() instead." << std::endl;

#pragma omp parallel for schedule(dynamic, 1)
    for (int p_y = 0; p_y < image.height; p_y++) {
        if (p_y % progressInterval == 0) {
#pragma omp critical
            {
                std::cout << "Progress: " << (p_y * 100 / totalPixels) << "%" << std::endl;
            }
        }

        for (int p_x = 0; p_x < image.width; p_x++) {
            const int p_idx = p_y * image.width + p_x;

            std::array<double, NUM_WAVELENGTHS> spectralRadiance = {0};

            for (unsigned int s = 0; s < samplesPerPixel; ++s) {
                Ray ray;
                camera.filmView(p_x, p_y, ray);

                // 各波長で独立にパストレース（7回ループ = 遅い）
                for (int w = 0; w < NUM_WAVELENGTHS; ++w) {
                    double radiance = tracePathSpectral(ray, 0, w, false, 1.0, true);
                    radiance = sanitizeRadiance(radiance, maxRadiance);
                    spectralRadiance[w] += radiance;
                }
            }

            // 平均化
            for (int w = 0; w < NUM_WAVELENGTHS; ++w) {
                spectralRadiance[w] /= static_cast<double>(samplesPerPixel);
            }

            // スペクトル → XYZ → RGB 変換
            double X = 0, Y = 0, Z = 0;
            double normFactor = 0;

            for (int w = 0; w < NUM_WAVELENGTHS; ++w) {
                X += spectralRadiance[w] * CIE_X[w];
                Y += spectralRadiance[w] * CIE_Y[w];
                Z += spectralRadiance[w] * CIE_Z[w];
                normFactor += CIE_Y[w];
            }

            if (normFactor > 0) {
                X /= normFactor;
                Y /= normFactor;
                Z /= normFactor;
            }

            image.pixels[p_idx] = XYZtoRGB(X, Y, Z);
        }
    }

    std::cout << "Progress: 100%" << std::endl;
    return image;
}

//==============================================================================
// Hero Wavelength Sampling による高速スペクトルレンダリング
//
// 原理：
// - 各サンプルで1つの波長をランダムに選択（確率 1/N）
// - その波長でパストレーシングを実行
// - XYZ色空間に変換する際、選択確率で割る（= N倍する）
// - モンテカルロ積分として収束
//
// 効果：
// - 従来: サンプルあたり 7回のパストレース
// - Hero: サンプルあたり 1回のパストレース
// - 約7倍の高速化
//==============================================================================

Image Renderer::spectralRenderHero(const unsigned int &samplesPerPixel) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());

    int totalPixels = image.height;
    int progressInterval = std::max(1, totalPixels / 20);
    const double maxRadiance = 50.0;

    std::cout << "=== Hero Wavelength Spectral Rendering ===" << std::endl;
    std::cout << "BVH Acceleration: " << (useBVH ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Beer-Lambert Absorption: ENABLED" << std::endl;
    std::cout << "Samples per pixel: " << samplesPerPixel << std::endl;
    std::cout << "Wavelengths: " << NUM_WAVELENGTHS << " (400-700nm, hero sampling)" << std::endl;

    // 波長間隔（XYZへの変換用）
    constexpr double deltaLambda = 50.0;  // 50nm間隔

    // 選択確率（一様サンプリング）
    const double wavelengthPDF = getWavelengthPDF();

#pragma omp parallel for schedule(dynamic, 1)
    for (int p_y = 0; p_y < image.height; p_y++) {
        if (p_y % progressInterval == 0) {
#pragma omp critical
            {
                std::cout << "Progress: " << (p_y * 100 / totalPixels) << "%" << std::endl;
            }
        }

        for (int p_x = 0; p_x < image.width; p_x++) {
            const int p_idx = p_y * image.width + p_x;

            // XYZ色空間で蓄積
            double X_accum = 0.0, Y_accum = 0.0, Z_accum = 0.0;

            for (unsigned int s = 0; s < samplesPerPixel; ++s) {
                Ray ray;
                camera.filmView(p_x, p_y, ray);

                //==============================================================
                // Hero Wavelength: 1波長のみをランダムに選択
                //==============================================================
                int heroWavelength = sampleWavelengthIndex(rand());

                // その波長でパストレース
                double radiance = tracePathSpectral(ray, 0, heroWavelength, false, 1.0, true);
                radiance = sanitizeRadiance(radiance, maxRadiance);

                //==============================================================
                // XYZへの寄与を計算
                // モンテカルロ推定: E[f(λ)/p(λ)] で積分を近似
                // p(λ) = 1/N なので、結果を N 倍する
                //==============================================================
                double weight = deltaLambda / wavelengthPDF;  // = deltaLambda * N

                X_accum += radiance * CIE_X[heroWavelength] * weight;
                Y_accum += radiance * CIE_Y[heroWavelength] * weight;
                Z_accum += radiance * CIE_Z[heroWavelength] * weight;
            }

            // サンプル数で平均化
            X_accum /= static_cast<double>(samplesPerPixel);
            Y_accum /= static_cast<double>(samplesPerPixel);
            Z_accum /= static_cast<double>(samplesPerPixel);

            // 正規化（CIE Y関数の積分で割る）
            double normFactor = 0.0;
            for (int w = 0; w < NUM_WAVELENGTHS; ++w) {
                normFactor += CIE_Y[w] * deltaLambda;
            }

            if (normFactor > 0) {
                X_accum /= normFactor;
                Y_accum /= normFactor;
                Z_accum /= normFactor;
            }

            // XYZ → RGB 変換
            image.pixels[p_idx] = XYZtoRGB(X_accum, Y_accum, Z_accum);
        }
    }

    std::cout << "Progress: 100%" << std::endl;
    return image;
}

//==============================================================================
// Microfacet Normal Sampling（Beckmann分布）
//==============================================================================
Eigen::Vector3d Renderer::sampleBeckmannNormal(const Eigen::Vector3d &geometricNormal,
                                                double alpha) const {
    if (alpha < 1e-6) {
        return geometricNormal;
    }

    double r1 = rand();
    double r2 = rand();

    if (r1 >= 1.0 - 1e-10) {
        r1 = 1.0 - 1e-10;
    }

    double theta_h = std::atan(alpha * std::sqrt(-std::log(1.0 - r1)));
    double phi_h = 2.0 * EIGEN_PI * r2;

    if (theta_h <= 0.0 || theta_h >= EIGEN_PI / 2.0) {
        return geometricNormal;
    }

    double sin_theta = std::sin(theta_h);
    double cos_theta = std::cos(theta_h);

    Eigen::Vector3d h_local(
        std::cos(phi_h) * sin_theta,
        std::sin(phi_h) * sin_theta,
        cos_theta
    );

    Eigen::Vector3d u, v;
    computeLocalFrame(geometricNormal, u, v);

    Eigen::Vector3d h_world = h_local.x() * u + h_local.y() * v + h_local.z() * geometricNormal;

    double len = h_world.norm();
    if (len < 1e-8 || !std::isfinite(len)) {
        return geometricNormal;
    }
    h_world /= len;

    if (h_world.dot(geometricNormal) < 0.0) {
        h_world = -h_world;
    }

    return h_world;
}

//==============================================================================
// Spectral Path Tracing（単一波長）with BVH and Beer-Lambert Absorption
//==============================================================================

double Renderer::tracePathSpectral(const Ray &ray, unsigned int depth,
                                   int wavelengthIndex,
                                   bool insideObject, double currentIOR,
                                   bool prevSpecular) const {
    if (depth >= maxDepth) {
        return 0.0;
    }

    RayHit hit;
    if (!hitScene(ray, hit)) {
        return (bgColor.x() + bgColor.y() + bgColor.z()) / 3.0;
    }

    const Body &body = bodies[hit.idx];
    const Material &mat = body.material;

    if (mat.isEmissive()) {
        if (prevSpecular) {
            Color emission = body.getEmission();
            return (emission.x() + emission.y() + emission.z()) / 3.0;
        } else {
            return 0.0;
        }
    }

    double result = 0.0;

    if (mat.type == MaterialType::Glass) {
        Eigen::Vector3d incident = ray.dir.normalized();
        Eigen::Vector3d geometricNormal = hit.normal.normalized();

        Eigen::Vector3d normal;
        if (mat.isMicrofacet()) {
            normal = sampleBeckmannNormal(geometricNormal, mat.alpha);
        } else {
            normal = geometricNormal;
        }

        double n1, n2;
        double cosI = -incident.dot(normal);

        // 波長依存のIOR
        double spectralIOR;
        if (mat.isAir()) {
            spectralIOR = 1.0;
        } else {
            spectralIOR = getIceIOR(wavelengthIndex);
        }

        bool enteringMaterial = cosI > 0;
        if (cosI < 0) {
            normal = -normal;
            cosI = -cosI;
            n1 = spectralIOR;
            n2 = currentIOR;
        } else {
            n1 = currentIOR;
            n2 = spectralIOR;
        }

        cosI = std::min(1.0, std::max(0.0, cosI));

        Eigen::Vector3d sampledDir;
        bool isRefraction;
        sampleGlassBSDF(incident, normal, n1, n2, sampledDir, isRefraction);
        sampledDir.normalize();

        if (!std::isfinite(sampledDir.norm()) || sampledDir.norm() < 0.9) {
            sampledDir = reflect(incident, normal).normalized();
            isRefraction = false;
        }

        Eigen::Vector3d newOrigin;
        bool newInsideObject;
        double newIOR;

        if (isRefraction) {
            newOrigin = hit.point - normal * 1e-4;
            newInsideObject = !insideObject;
            newIOR = n2;
        } else {
            newOrigin = hit.point + normal * 1e-4;
            newInsideObject = insideObject;
            newIOR = currentIOR;
        }

        Ray newRay(newOrigin, sampledDir);
        double incomingRadiance = tracePathSpectral(newRay, depth + 1, wavelengthIndex,
                                                     newInsideObject, newIOR, true);

        //======================================================================
        // Beer-Lambert吸収の適用
        // 氷の内部を通過した光は波長依存の吸収を受ける
        //======================================================================
        if (insideObject && !mat.isAir() && hit.t > 0) {
            double travelDistance = hit.t;
            double absorptionCoeff = getIceAbsorption(wavelengthIndex);
            double transmittance = beerLambertTransmittance(absorptionCoeff, travelDistance);
            incomingRadiance *= transmittance;
        }

        result = incomingRadiance;

    } else {
        // Diffuse material
        double kd = mat.kd;
        double albedo = (mat.color.x() + mat.color.y() + mat.color.z()) / 3.0;

        // NEE for direct lighting
        if (!lightIndices.empty()) {
            Color directContrib = sampleDirectLight(hit.point, hit.normal, -ray.dir, mat);
            if (isValidColor(directContrib)) {
                result += (directContrib.x() + directContrib.y() + directContrib.z()) / 3.0;
            }
        }

        // Russian Roulette
        double continueProbability = std::min(0.95, std::max(kd * albedo, 0.1));

        if (rand() < continueProbability) {
            Eigen::Vector3d outDir;
            double pdf;
            cosineSample(hit.normal, outDir, pdf);
            Ray nextRay(hit.point + hit.normal * 1e-4, outDir);

            double indirectRadiance = tracePathSpectral(nextRay, depth + 1, wavelengthIndex,
                                                         insideObject, currentIOR, false);

            result += kd * albedo * indirectRadiance / continueProbability;
        }
    }

    return result;
}

Color Renderer::tracePath(const Ray &ray, unsigned int depth,
                          bool insideObject, double currentIOR,
                          bool prevSpecular) const {
    if (depth >= maxDepth) {
        return Color::Zero();
    }

    RayHit hit;
    if (!hitScene(ray, hit)) {
        return bgColor;
    }

    const Body &body = bodies[hit.idx];
    const Material &mat = body.material;

    if (mat.isEmissive()) {
        if (prevSpecular) {
            return body.getEmission();
        } else {
            return Color::Zero();
        }
    }

    Color result = Color::Zero();

    if (mat.type == MaterialType::Glass) {
        Eigen::Vector3d incident = ray.dir.normalized();
        Eigen::Vector3d geometricNormal = hit.normal.normalized();

        Eigen::Vector3d normal;
        if (mat.isMicrofacet()) {
            normal = sampleBeckmannNormal(geometricNormal, mat.alpha);
        } else {
            normal = geometricNormal;
        }

        double n1, n2;
        double cosI = -incident.dot(normal);

        if (cosI < 0) {
            normal = -normal;
            cosI = -cosI;
            n1 = mat.ior;
            n2 = currentIOR;
        } else {
            n1 = currentIOR;
            n2 = mat.ior;
        }

        cosI = std::min(1.0, std::max(0.0, cosI));

        Eigen::Vector3d sampledDir;
        bool isRefraction;
        sampleGlassBSDF(incident, normal, n1, n2, sampledDir, isRefraction);
        sampledDir.normalize();

        if (!std::isfinite(sampledDir.norm()) || sampledDir.norm() < 0.9) {
            sampledDir = reflect(incident, normal).normalized();
            isRefraction = false;
        }

        Eigen::Vector3d newOrigin;
        bool newInsideObject;
        double newIOR;

        if (isRefraction) {
            newOrigin = hit.point - normal * 1e-4;
            newInsideObject = !insideObject;
            newIOR = n2;
        } else {
            newOrigin = hit.point + normal * 1e-4;
            newInsideObject = insideObject;
            newIOR = currentIOR;
        }

        Ray newRay(newOrigin, sampledDir);
        Color incomingRadiance = tracePath(newRay, depth + 1, newInsideObject, newIOR, true);

        if (!isValidColor(incomingRadiance)) {
            return Color::Zero();
        }

        if (insideObject && isRefraction) {
            incomingRadiance = incomingRadiance.cwiseProduct(mat.color);
        }

        result = incomingRadiance;

    } else {
        if (!lightIndices.empty()) {
            Color directContrib = sampleDirectLight(hit.point, hit.normal, -ray.dir, mat);
            if (isValidColor(directContrib)) {
                result += directContrib;
            }
        }

        double continueProbability = std::min(0.95, std::max(mat.kd * mat.color.maxCoeff(), 0.1));

        if (rand() < continueProbability) {
            Eigen::Vector3d outDir;
            double pdf;
            cosineSample(hit.normal, outDir, pdf);
            Ray nextRay(hit.point + hit.normal * 1e-4, outDir);

            Color brdfWeight = body.getKd();
            Color indirectRadiance = tracePath(nextRay, depth + 1, insideObject, currentIOR, false);

            if (isValidColor(indirectRadiance)) {
                result += brdfWeight.cwiseProduct(indirectRadiance) / continueProbability;
            }
        }
    }

    return result;
}

//==============================================================================
// NEE - Direct Light Sampling (BVH optimized shadow rays)
//==============================================================================

Color Renderer::sampleDirectLight(const Eigen::Vector3d &hitPoint,
                                  const Eigen::Vector3d &normal,
                                  const Eigen::Vector3d &wo,
                                  const Material &material) const {
    if (lightIndices.empty()) {
        return Color::Zero();
    }

    Color directLight = Color::Zero();

    for (int lightIdx : lightIndices) {
        const Body &light = bodies[lightIdx];

        Eigen::Vector3d lightCenter;
        double lightRadius;
        if (!light.getSphereInfo(lightCenter, lightRadius)) {
            continue;
        }

        double lightPdf;
        Eigen::Vector3d lightPoint = sampleSphereUniform(lightCenter, lightRadius, lightPdf);

        Eigen::Vector3d toLight = lightPoint - hitPoint;
        double distanceSquared = toLight.squaredNorm();
        if (distanceSquared < 1e-6) continue;

        double distance = sqrt(distanceSquared);
        Eigen::Vector3d wi = toLight / distance;

        double cosTheta = normal.dot(wi);
        if (cosTheta <= 0) continue;

        Eigen::Vector3d lightNormal = (lightPoint - lightCenter).normalized();
        double cosThetaLight = -lightNormal.dot(wi);
        if (cosThetaLight <= 0) continue;

        Ray shadowRay(hitPoint + normal * 1e-4, wi);
        RayHit shadowHit;

        if (hitScene(shadowRay, shadowHit)) {
            if (shadowHit.idx != lightIdx || shadowHit.t > distance + 1e-3) {
                continue;
            }
        } else {
            continue;
        }

        double G = (cosTheta * cosThetaLight) / distanceSquared;
        G = std::min(G, 1000.0);

        Color brdf = material.kd * material.color / EIGEN_PI;
        Color Le = light.getEmission();

        if (lightPdf < 1e-10) continue;

        Color contrib = Le.cwiseProduct(brdf) * G / lightPdf;
        if (isValidColor(contrib)) {
            directLight += contrib;
        }
    }

    return directLight;
}

Eigen::Vector3d Renderer::sampleSphereUniform(const Eigen::Vector3d &center,
                                              double radius, double &pdf) const {
    double z = 1.0 - 2.0 * rand();
    double r = sqrt(std::max(0.0, 1.0 - z * z));
    double phi = 2.0 * EIGEN_PI * rand();

    Eigen::Vector3d localPoint(r * cos(phi), r * sin(phi), z);
    pdf = 1.0 / (4.0 * EIGEN_PI * radius * radius);

    return center + radius * localPoint;
}

//==============================================================================
// Sampling utilities
//==============================================================================

void Renderer::cosineSample(const Eigen::Vector3d &normal,
                            Eigen::Vector3d &outDir, double &pdf) const {
    double phi = 2.0 * EIGEN_PI * rand();
    double cosTheta = sqrt(rand());
    double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    Eigen::Vector3d u, v;
    computeLocalFrame(normal, u, v);

    outDir = sinTheta * cos(phi) * u + cosTheta * normal + sinTheta * sin(phi) * v;
    outDir.normalize();
    pdf = cosTheta / EIGEN_PI;
}

void Renderer::computeLocalFrame(const Eigen::Vector3d &w, Eigen::Vector3d &u, Eigen::Vector3d &v) {
    if (fabs(w.x()) > 1e-3)
        u = Eigen::Vector3d::UnitY().cross(w).normalized();
    else
        u = Eigen::Vector3d::UnitX().cross(w).normalized();
    v = w.cross(u).normalized();
}

//==============================================================================
// Glass BSDF Sampling
//==============================================================================

double Renderer::sampleGlassBSDF(const Eigen::Vector3d &wo, const Eigen::Vector3d &normal,
                                 double n1, double n2, Eigen::Vector3d &wi,
                                 bool &isRefraction) const {
    double cosI = -wo.dot(normal);
    cosI = std::min(1.0, std::abs(cosI));

    Eigen::Vector3d refractedDir;
    bool canRefract = refract(wo, normal, n1, n2, refractedDir);

    double fresnelR;
    if (canRefract) {
        fresnelR = fresnelSchlick(cosI, n1, n2);
        fresnelR = std::min(1.0, std::max(0.0, fresnelR));
    } else {
        fresnelR = 1.0;
    }

    Eigen::Vector3d reflectedDir = reflect(wo, normal);

    double r = rand();
    if (r < fresnelR || !canRefract) {
        wi = reflectedDir.normalized();
        isRefraction = false;
        return 1.0;
    } else {
        wi = refractedDir.normalized();
        isRefraction = true;
        return 1.0;
    }
}