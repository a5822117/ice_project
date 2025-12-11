//
// Created by kango on 2023/01/02.
//

#ifndef CUDATEST_CAMERA_CUH
#define CUDATEST_CAMERA_CUH


#include "Ray.h"
#include "Film.h"

class Camera{
private:
    double focalLength;
    Eigen::Vector3d org, right, up;
    Film film;

public:
    /**
     * @param position カメラの位置
     * @param dir カメラの向き
     * @param resolutionHeight カメラの解像度（高さ）
     * @param aspectRatio アスペクト比
     * @param verticalFoV 視野角（高さ）
     * @param focalLength カメラとフィルムの距離
     */
    Camera(Eigen::Vector3d position, const Eigen::Vector3d &dir, const int &resolutionHeight, double aspectRatio, double verticalFoV, double focalLength=1.0);
    void filmView(const unsigned int &p_x, const unsigned int &p_y, Ray &out_ray) const;

    const Film &getFilm() const;

    Eigen::Vector3d dir;
};


#endif //CUDATEST_CAMERA_CUH