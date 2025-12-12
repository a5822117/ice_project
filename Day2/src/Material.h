//
// Material.h
// Extended for ice rendering with glass/ice materials
// + Beer-Lambert absorption for realistic blue ice color
// + Microfacet model for realistic ice surface (Ghafari & Park 2017)
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
//
// 単位: m^-1 (1メートルあたりの吸収)
//
// 重要な物理:
// - 青色光（400-500nm）: 吸収係数が非常に小さい → よく透過する
// - 赤色光（600-700nm）: 吸収係数が大きい → 吸収されやすい
// - この差により、厚い氷を通過した光は青く見える
//
// Beer-Lambert則: I(d) = I₀ × exp(-α × d)
// ここで α は吸収係数、d は光路長
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

// 吸収のスケールファクター
// シーンの単位系に合わせて調整
// 値が大きいほど吸収が強くなる（青みが増す）
// デフォルト: 1.0 = 単位がメートルの場合
// シーンがセンチメートル単位の場合: 0.01
// シーンの単位が任意の場合、視覚的に調整
constexpr double ABSORPTION_SCALE = 0.01;  // シーン単位に合わせて調整

/// 波長インデックスから氷のIORを取得
inline double getIceIOR(int wavelengthIndex) {
    if (wavelengthIndex < 0 || wavelengthIndex >= NUM_WAVELENGTHS) {
        return 1.31;  // デフォルト値
    }
    return ICE_IOR[wavelengthIndex];
}

/// 波長インデックスから氷の吸収係数を取得
inline double getIceAbsorption(int wavelengthIndex) {
    if (wavelengthIndex < 0 || wavelengthIndex >= NUM_WAVELENGTHS) {
        return 0.05;  // デフォルト値
    }
    return ICE_ABSORPTION_COEFF[wavelengthIndex] * ABSORPTION_SCALE;
}

/// Beer-Lambert則による透過率を計算
/// @param absorptionCoeff 吸収係数 (m^-1 or scene units^-1)
/// @param distance 光路長 (m or scene units)
/// @return 透過率 (0.0 - 1.0)
inline double beerLambertTransmittance(double absorptionCoeff, double distance) {
    if (distance <= 0.0 || absorptionCoeff <= 0.0) {
        return 1.0;  // 吸収なし
    }
    return std::exp(-absorptionCoeff * distance);
}

/// 波長（nm）から氷のIORを補間計算
inline double getIceIORByWavelength(double wavelength_nm) {
    // 線形補間
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

// CIE 1931 XYZ等色関数（簡易版、正規化済み）
// 各波長でのX, Y, Z応答
constexpr std::array<double, NUM_WAVELENGTHS> CIE_X = {
    0.01431, 0.33620, 0.00490, 0.43400, 1.06200, 0.28310, 0.01100
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Y = {
    0.00040, 0.03800, 0.32300, 0.99500, 0.63100, 0.10700, 0.00400
};
constexpr std::array<double, NUM_WAVELENGTHS> CIE_Z = {
    0.06790, 1.77211, 0.27200, 0.00880, 0.00080, 0.00000, 0.00000
};

/// XYZ色空間からsRGB色空間への変換
inline Color XYZtoRGB(double X, double Y, double Z) {
    // sRGB D65変換行列
    double R =  3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z;
    double G = -0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z;
    double B =  0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z;

    // 負の値をクランプ
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
    double ior;  // 屈折率（Index of Refraction）

    //==========================================================================
    // Microfacet パラメータ（Ghafari & Park 2017）
    //
    // alpha: Beckmann分布の粗さパラメータ
    // - alpha = 0.0: 完全に滑らかな表面（通常のガラス）
    // - alpha = 0.05〜0.15: 氷の表面（推奨値）
    // - alpha = 0.3以上: 非常に粗い表面（曇りガラス）
    //
    // 氷は完全に滑らかなガラスとは異なり、表面に微細な凹凸があるため、
    // 光が散乱して曇ったように見えます。このalphaパラメータにより、
    // その効果を再現します。
    //==========================================================================
    double alpha;  // マイクロファセット粗さ（0.0 = 完全鏡面）

public:
    /// 拡散マテリアルのコンストラクタ
    Material(Color color, const double &kd, const double &emission = 0.0)
        : color(std::move(color)), kd(kd), emission(emission),
          type(MaterialType::Diffuse), ior(1.0), alpha(0.0) {}

    /// ガラス/氷マテリアルのコンストラクタ（マイクロファセット対応）
    /// @param alpha マイクロファセット粗さ（0.0〜1.0、氷の場合0.05〜0.15推奨）
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

    /// マイクロファセットモデルを使用するかどうか
    bool isMicrofacet() const {
        return alpha > 1e-6;
    }

    /// このマテリアルが氷かどうか判定（IOR ≈ 1.31）
    bool isIce() const {
        return type == MaterialType::Glass && std::abs(ior - 1.31) < 0.05;
    }

    /// このマテリアルが空気（気泡）かどうか判定（IOR ≈ 1.0）
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
    if (sin2T > 1.0 - 1e-8) return false;  // 全反射

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