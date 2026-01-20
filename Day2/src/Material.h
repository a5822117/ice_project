//
// Material.h
// Extended for ice rendering with glass/ice materials
// + Beer-Lambert absorption for realistic blue ice color
// + Microfacet model for realistic ice surface (Ghafari & Park 2017)
// + Hero Wavelength Sampling support
//

#ifndef DAY_3_MATERIAL_H
#define DAY_3_MATERIAL_H

#include <utility>
#include <cmath>
#include <algorithm>
#include <array>
#include "Image.h"

//==============================================================================
// Spectral Rendering 用の定数とユーティリティ
//==============================================================================

// サンプリング波長（nm）
constexpr int NUM_WAVELENGTHS = 7;
constexpr std::array<double, NUM_WAVELENGTHS> WAVELENGTHS = {400, 450, 500, 550, 600, 650, 700};

// 氷の屈折率（波長依存）- Warren 1984 の実測データに基づく
constexpr std::array<double, NUM_WAVELENGTHS> ICE_IOR = {
    1.3170,  // 400nm
    1.3140,  // 450nm
    1.3115,  // 500nm
    1.3092,  // 550nm
    1.3073,  // 600nm
    1.3058,  // 650nm
    1.3046   // 700nm
};

//==============================================================================
// 氷の吸収係数（Beer-Lambert則用）
// Warren & Brandt (2008) "Optical constants of ice" に基づく
// 単位: m^-1
//==============================================================================
constexpr std::array<double, NUM_WAVELENGTHS> ICE_ABSORPTION_COEFF = {
    0.0170,   // 400nm
    0.0106,   // 450nm - 最も低い吸収
    0.0114,   // 500nm
    0.0264,   // 550nm
    0.0619,   // 600nm
    0.2570,   // 650nm
    0.4100    // 700nm - 最も高い吸収
};

// 吸収のスケールファクター
constexpr double ABSORPTION_SCALE = 0.01;

//==============================================================================
// Hero Wavelength Sampling 用：XYZ等色関数
// CIE 1931 標準観測者（簡易版、7波長）
//==============================================================================
constexpr std::array<double, NUM_WAVELENGTHS> CIE_X = {
    0.01431, 0.33620, 0.00490, 0.43400, 1.06200, 0.28310, 0.01100
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Y = {
    0.00040, 0.03800, 0.32300, 0.99500, 0.63100, 0.10700, 0.00400
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Z = {
    0.06790, 1.77211, 0.27200, 0.00880, 0.00080, 0.00000, 0.00000
};

//==============================================================================
// Hero Wavelength Sampling ユーティリティ関数
//==============================================================================

/// 一様乱数から波長インデックスをサンプリング
/// @param u 一様乱数 [0, 1)
/// @return 波長インデックス (0 to NUM_WAVELENGTHS-1)
inline int sampleWavelengthIndex(double u) {
    int idx = static_cast<int>(u * NUM_WAVELENGTHS);
    return std::min(idx, NUM_WAVELENGTHS - 1);
}

/// 波長サンプリングのPDFを取得（一様サンプリング）
inline double getWavelengthPDF() {
    return 1.0 / NUM_WAVELENGTHS;
}

/// 波長インデックスから氷のIORを取得
inline double getIceIOR(int wavelengthIndex) {
    if (wavelengthIndex < 0 || wavelengthIndex >= NUM_WAVELENGTHS) {
        return 1.31;
    }
    return ICE_IOR[wavelengthIndex];
}

/// 波長インデックスから氷の吸収係数を取得
inline double getIceAbsorption(int wavelengthIndex) {
    if (wavelengthIndex < 0 || wavelengthIndex >= NUM_WAVELENGTHS) {
        return 0.05;
    }
    return ICE_ABSORPTION_COEFF[wavelengthIndex] * ABSORPTION_SCALE;
}

/// Beer-Lambert則による透過率を計算
inline double beerLambertTransmittance(double absorptionCoeff, double distance) {
    if (distance <= 0.0 || absorptionCoeff <= 0.0) {
        return 1.0;
    }
    return std::exp(-absorptionCoeff * distance);
}

/// 波長（nm）から氷のIORを補間計算
inline double getIceIORByWavelength(double wavelength_nm) {
    if (wavelength_nm <= WAVELENGTHS[0]) return ICE_IOR[0];
    if (wavelength_nm >= WAVELENGTHS[NUM_WAVELENGTHS-1]) return ICE_IOR[NUM_WAVELENGTHS-1];

    for (int i = 0; i < NUM_WAVELENGTHS - 1; ++i) {
        if (wavelength_nm >= WAVELENGTHS[i] && wavelength_nm <= WAVELENGTHS[i+1]) {
            double t = (wavelength_nm - WAVELENGTHS[i]) / (WAVELENGTHS[i+1] - WAVELENGTHS[i]);
            return ICE_IOR[i] * (1.0 - t) + ICE_IOR[i+1] * t;
        }
    }
    return 1.31;
}

//==============================================================================
// XYZ → RGB 変換
//==============================================================================

/// XYZ色空間からsRGB色空間への変換
inline Color XYZtoRGB(double X, double Y, double Z) {
    double R =  3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z;
    double G = -0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z;
    double B =  0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z;

    R = std::max(0.0, R);
    G = std::max(0.0, G);
    B = std::max(0.0, B);

    return Color(R, G, B);
}

/// マテリアルの種類を表す列挙型
enum class MaterialType {
    Diffuse,
    Glass
};

struct Material {
    Color color;
    double kd;
    double emission;
    MaterialType type;
    double ior;
    double alpha;  // マイクロファセット粗さ

public:
    /// 拡散マテリアルのコンストラクタ
    Material(Color color, const double &kd, const double &emission = 0.0)
        : color(std::move(color)), kd(kd), emission(emission),
          type(MaterialType::Diffuse), ior(1.0), alpha(0.0) {}

    /// ガラス/氷マテリアルのコンストラクタ
    Material(Color color, double ior, MaterialType type, const double &emission = 0.0,
             double alpha = 0.0)
        : color(std::move(color)), kd(0.0), emission(emission),
          type(type), ior(ior), alpha(alpha) {}

    /// デフォルトコンストラクタ
    Material() : color(Color::Ones()), kd(0.8), emission(0.0),
                 type(MaterialType::Diffuse), ior(1.0), alpha(0.0) {}

    bool isGlass() const {
        return type == MaterialType::Glass;
    }

    bool isEmissive() const {
        return emission > 0.0;
    }

    bool isMicrofacet() const {
        return alpha > 1e-6;
    }

    bool isIce() const {
        return type == MaterialType::Glass && std::abs(ior - 1.31) < 0.05;
    }

    bool isAir() const {
        return type == MaterialType::Glass && std::abs(ior - 1.0) < 0.05;
    }
};

//==============================================================================
// 光学計算用のユーティリティ関数
//==============================================================================

/// フレネル反射率を計算（Schlickの近似式）
inline double fresnelSchlick(double cosTheta, double n1, double n2) {
    cosTheta = std::min(1.0, std::max(0.0, std::abs(cosTheta)));
    if (n1 <= 0.0) n1 = 1.0;
    if (n2 <= 0.0) n2 = 1.0;

    double ratio = (n1 - n2) / (n1 + n2);
    double R0 = ratio * ratio;
    double oneMinusCos = 1.0 - cosTheta;
    double oneMinusCos5 = oneMinusCos * oneMinusCos * oneMinusCos * oneMinusCos * oneMinusCos;
    double R = R0 + (1.0 - R0) * oneMinusCos5;
    return std::min(1.0, std::max(0.0, R));
}

/// 屈折方向を計算（スネルの法則）
inline bool refract(const Eigen::Vector3d &incident, const Eigen::Vector3d &normal,
                    double n1, double n2, Eigen::Vector3d &refracted) {
    if (n2 <= 0.0 || n1 <= 0.0) return false;

    double eta = n1 / n2;
    double cosI = -incident.dot(normal);
    cosI = std::min(1.0, std::max(-1.0, cosI));
    if (cosI < 0) cosI = std::abs(cosI);

    double sin2T = eta * eta * (1.0 - cosI * cosI);
    if (sin2T > 1.0 - 1e-8) return false;

    double cosT = std::sqrt(std::max(0.0, 1.0 - sin2T));
    refracted = eta * incident + (eta * cosI - cosT) * normal;

    double len = refracted.norm();
    if (len < 1e-8 || !std::isfinite(len)) return false;
    refracted /= len;
    return true;
}

/// 反射方向を計算
inline Eigen::Vector3d reflect(const Eigen::Vector3d &incident, const Eigen::Vector3d &normal) {
    Eigen::Vector3d reflected = incident - 2.0 * incident.dot(normal) * normal;
    double len = reflected.norm();
    if (len > 1e-8) reflected /= len;
    return reflected;
}

#endif //DAY_3_MATERIAL_H