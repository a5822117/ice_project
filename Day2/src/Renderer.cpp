//
// Path Tracing Renderer with NEE
// Extended for ice/glass rendering
//

#include "Renderer.h"
#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>

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

//==============================================================================
// Renderer 実装
//==============================================================================

Renderer::Renderer(const std::vector<Body> &bodies, Camera camera, Color bgColor, unsigned int maxDepth)
    : bodies(bodies), camera(std::move(camera)), bgColor(std::move(bgColor)),
      engine(0), dist(0, 1), maxDepth(maxDepth) {

    // 光源インデックスを構築
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].isLight()) {
            lightIndices.push_back(static_cast<int>(i));
        }
    }

    if (!lightIndices.empty()) {
        std::cout << "Found " << lightIndices.size() << " light source(s) for NEE" << std::endl;
    }
}

double Renderer::rand() const {
    return dist(engine);
}

bool Renderer::hitScene(const Ray &ray, RayHit &hit) const {
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
// Path Tracing with NEE
//==============================================================================

Image Renderer::pathTracingRender(const unsigned int &samplesPerPixel) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());

    int totalPixels = image.height;
    int progressInterval = std::max(1, totalPixels / 20);
    const double maxRadiance = 50.0;

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
// Spectral Rendering（波長依存レンダリング）
//==============================================================================

Image Renderer::spectralRender(const unsigned int &samplesPerPixel) const {
    Image image(camera.getFilm().resolution.x(), camera.getFilm().resolution.y());

    int totalPixels = image.height;
    int progressInterval = std::max(1, totalPixels / 20);
    const double maxRadiance = 50.0;

    std::cout << "=== Spectral Rendering Mode ===" << std::endl;
    std::cout << "Wavelengths: ";
    for (int i = 0; i < NUM_WAVELENGTHS; ++i) {
        std::cout << WAVELENGTHS[i] << "nm ";
    }
    std::cout << std::endl;

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

            // 各波長でのスペクトル値を蓄積
            std::array<double, NUM_WAVELENGTHS> spectralRadiance = {0};

            for (unsigned int s = 0; s < samplesPerPixel; ++s) {
                Ray ray;
                camera.filmView(p_x, p_y, ray);

                // 各波長で独立にパストレース
                for (int w = 0; w < NUM_WAVELENGTHS; ++w) {
                    double radiance = tracePathSpectral(ray, 0, w, false, 1.0, true);

                    // 値のサニタイズ
                    if (std::isnan(radiance) || std::isinf(radiance) || radiance < 0) {
                        radiance = 0.0;
                    }
                    radiance = std::min(radiance, maxRadiance);

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
                normFactor += CIE_Y[w];  // Y成分で正規化
            }

            // 正規化
            if (normFactor > 0) {
                X /= normFactor;
                Y /= normFactor;
                Z /= normFactor;
            }

            // XYZ → RGB変換
            image.pixels[p_idx] = XYZtoRGB(X, Y, Z);
        }
    }

    std::cout << "Progress: 100%" << std::endl;
    return image;
}

//==============================================================================
// Spectral Path Tracing（単一波長）
// 【修正】気泡（空気、IOR=1.0）は波長依存しない
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
        // 背景色（グレースケール化）
        return (bgColor.x() + bgColor.y() + bgColor.z()) / 3.0;
    }

    const Body &body = bodies[hit.idx];
    const Material &mat = body.material;

    // 光源にヒット
    if (mat.isEmissive()) {
        if (prevSpecular) {
            // 光源の輝度（グレースケール）
            Color emission = body.getEmission();
            return (emission.x() + emission.y() + emission.z()) / 3.0;
        } else {
            return 0.0;
        }
    }

    double result = 0.0;

    if (mat.type == MaterialType::Glass) {
        //======================================================================
        // Glass/Ice material - 波長依存の屈折率を使用
        // 【重要な修正】気泡（空気、IOR≈1.0）は波長依存しない
        //======================================================================
        Eigen::Vector3d incident = ray.dir.normalized();
        Eigen::Vector3d normal = hit.normal.normalized();
        double n1, n2;

        double cosI = -incident.dot(normal);

        // 波長依存の屈折率を取得
        // 気泡（空気）の場合はIOR=1.0を維持、氷の場合のみ波長依存IORを使用
        double spectralIOR;
        if (std::abs(mat.ior - 1.0) < 0.05) {
            // 空気（気泡）: 波長に依存しないIOR = 1.0
            spectralIOR = 1.0;
        } else {
            // 氷: 波長依存の屈折率を使用（Warren 1984）
            spectralIOR = getIceIOR(wavelengthIndex);
        }

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

        // 媒質内での減衰（マテリアルカラーをグレースケール化）
        if (insideObject && isRefraction) {
            double attenuation = (mat.color.x() + mat.color.y() + mat.color.z()) / 3.0;
            incomingRadiance *= attenuation;
        }

        result = incomingRadiance;

    } else {
        //======================================================================
        // Diffuse material
        //======================================================================

        // マテリアルの反射率（グレースケール）
        double kd = mat.kd;
        double albedo = (mat.color.x() + mat.color.y() + mat.color.z()) / 3.0;

        // NEE: 光源直接サンプリング
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

    // 光源にヒット
    if (mat.isEmissive()) {
        if (prevSpecular) {
            return body.getEmission();
        } else {
            return Color::Zero();  // NEEで既にカウント済み
        }
    }

    Color result = Color::Zero();

    if (mat.type == MaterialType::Glass) {
        //======================================================================
        // Glass/Ice material
        //======================================================================
        Eigen::Vector3d incident = ray.dir.normalized();
        Eigen::Vector3d normal = hit.normal.normalized();
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

        // 媒質内での減衰
        if (insideObject && isRefraction) {
            incomingRadiance = incomingRadiance.cwiseProduct(mat.color);
        }

        result = incomingRadiance;

    } else {
        //======================================================================
        // Diffuse material
        //======================================================================

        // NEE: 光源直接サンプリング
        if (!lightIndices.empty()) {
            Color directContrib = sampleDirectLight(hit.point, hit.normal, -ray.dir, mat);
            if (isValidColor(directContrib)) {
                result += directContrib;
            }
        }

        // Russian Roulette
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
// NEE - Direct Light Sampling
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
            continue;  // 球体光源のみサポート
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

        // シャドウレイテスト
        Ray shadowRay(hitPoint + normal * 1e-4, wi);
        RayHit shadowHit;

        if (hitScene(shadowRay, shadowHit)) {
            if (shadowHit.idx != lightIdx || shadowHit.t > distance + 1e-3) {
                continue;  // 遮蔽
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
        fresnelR = 1.0;  // 全反射
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