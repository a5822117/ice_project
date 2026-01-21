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

// 波長間隔（nm）- Hero Wavelength Samplingで使用
constexpr double WAVELENGTH_MIN = 400.0;
constexpr double WAVELENGTH_MAX = 700.0;
constexpr double WAVELENGTH_RANGE = WAVELENGTH_MAX - WAVELENGTH_MIN;

// 氷の屈折率（波長依存）- Warren 1984 の実測データに基づく
// 短波長（青）ほど屈折率が高い → 分散効果
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
// 単位: m^-1 (1メートルあたりの吸収)
//==============================================================================
constexpr std::array<double, NUM_WAVELENGTHS> ICE_ABSORPTION_COEFF = {
    0.0170,   // 400nm - 青紫: 非常に低い吸収
    0.0106,   // 450nm - 青: 最も低い吸収（氷が青く見える主因）
    0.0114,   // 500nm - 青緑: 低い吸収
    0.0264,   // 550nm - 緑: やや吸収
    0.0619,   // 600nm - 橙: 吸収増加
    0.2570,   // 650nm - 赤橙: 高い吸収
    0.4100    // 700nm - 赤: 非常に高い吸収（青の約40倍）
};

// 吸収のスケールファクター - シーンの単位系に合わせて調整
constexpr double ABSORPTION_SCALE = 0.003;

//==============================================================================
// Hero Wavelength Sampling 用ユーティリティ
//==============================================================================

/// 連続波長から氷のIORを線形補間で取得
inline double getIceIORContinuous(double wavelength_nm) {
    if (wavelength_nm <= WAVELENGTH_MIN) return ICE_IOR[0];
    if (wavelength_nm >= WAVELENGTH_MAX) return ICE_IOR[NUM_WAVELENGTHS - 1];

    // 線形補間
    double t = (wavelength_nm - WAVELENGTH_MIN) / WAVELENGTH_RANGE;
    int idx = static_cast<int>(t * (NUM_WAVELENGTHS - 1));
    idx = std::min(idx, NUM_WAVELENGTHS - 2);

    double local_t = (wavelength_nm - WAVELENGTHS[idx]) / (WAVELENGTHS[idx + 1] - WAVELENGTHS[idx]);
    local_t = std::max(0.0, std::min(1.0, local_t));

    return ICE_IOR[idx] * (1.0 - local_t) + ICE_IOR[idx + 1] * local_t;
}

/// 連続波長から氷の吸収係数を線形補間で取得
inline double getIceAbsorptionContinuous(double wavelength_nm) {
    if (wavelength_nm <= WAVELENGTH_MIN) return ICE_ABSORPTION_COEFF[0] * ABSORPTION_SCALE;
    if (wavelength_nm >= WAVELENGTH_MAX) return ICE_ABSORPTION_COEFF[NUM_WAVELENGTHS - 1] * ABSORPTION_SCALE;

    double t = (wavelength_nm - WAVELENGTH_MIN) / WAVELENGTH_RANGE;
    int idx = static_cast<int>(t * (NUM_WAVELENGTHS - 1));
    idx = std::min(idx, NUM_WAVELENGTHS - 2);

    double local_t = (wavelength_nm - WAVELENGTHS[idx]) / (WAVELENGTHS[idx + 1] - WAVELENGTHS[idx]);
    local_t = std::max(0.0, std::min(1.0, local_t));

    double coeff = ICE_ABSORPTION_COEFF[idx] * (1.0 - local_t) + ICE_ABSORPTION_COEFF[idx + 1] * local_t;
    return coeff * ABSORPTION_SCALE;
}

/// 波長インデックスから氷のIORを取得（離散版 - 後方互換性用）
inline double getIceIOR(int wavelengthIndex) {
    if (wavelengthIndex < 0 || wavelengthIndex >= NUM_WAVELENGTHS) {
        return 1.31;
    }
    return ICE_IOR[wavelengthIndex];
}

/// 波長インデックスから氷の吸収係数を取得（離散版 - 後方互換性用）
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

//==============================================================================
// CIE 1931 XYZ等色関数（連続波長対応版）
//==============================================================================

// 離散版（7波長）
constexpr std::array<double, NUM_WAVELENGTHS> CIE_X = {
    0.01431, 0.33620, 0.00490, 0.43400, 1.06200, 0.28310, 0.01100
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Y = {
    0.00040, 0.03800, 0.32300, 0.99500, 0.63100, 0.10700, 0.00400
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Z = {
    0.06790, 1.77211, 0.27200, 0.00880, 0.00080, 0.00000, 0.00000
};

/// 連続波長からCIE XYZ応答を線形補間で取得
inline void getCIEXYZContinuous(double wavelength_nm, double& x, double& y, double& z) {
    if (wavelength_nm <= WAVELENGTH_MIN) {
        x = CIE_X[0]; y = CIE_Y[0]; z = CIE_Z[0];
        return;
    }
    if (wavelength_nm >= WAVELENGTH_MAX) {
        x = CIE_X[NUM_WAVELENGTHS - 1];
        y = CIE_Y[NUM_WAVELENGTHS - 1];
        z = CIE_Z[NUM_WAVELENGTHS - 1];
        return;
    }

    double t = (wavelength_nm - WAVELENGTH_MIN) / WAVELENGTH_RANGE;
    int idx = static_cast<int>(t * (NUM_WAVELENGTHS - 1));
    idx = std::min(idx, NUM_WAVELENGTHS - 2);

    double local_t = (wavelength_nm - WAVELENGTHS[idx]) / (WAVELENGTHS[idx + 1] - WAVELENGTHS[idx]);
    local_t = std::max(0.0, std::min(1.0, local_t));

    x = CIE_X[idx] * (1.0 - local_t) + CIE_X[idx + 1] * local_t;
    y = CIE_Y[idx] * (1.0 - local_t) + CIE_Y[idx + 1] * local_t;
    z = CIE_Z[idx] * (1.0 - local_t) + CIE_Z[idx + 1] * local_t;
}

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

    bool isGlass() const { return type == MaterialType::Glass; }
    bool isEmissive() const { return emission > 0.0; }
    bool isMicrofacet() const { return alpha > 1e-6; }
    bool isIce() const { return type == MaterialType::Glass && std::abs(ior - 1.31) < 0.05; }
    bool isAir() const { return type == MaterialType::Glass && std::abs(ior - 1.0) < 0.05; }
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
    return std::min(1.0, std::max(0.0, R0 + (1.0 - R0) * oneMinusCos5));
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