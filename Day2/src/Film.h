//
// Created by kango on 2023/01/02.
//

#ifndef CUDATEST_FILM_H
#define CUDATEST_FILM_H

#include "Eigen/Dense"
#include "Image.h"

struct Film {
    /// 空間上におけるフィルムのサイズ
    Eigen::Vector2d filmSize;
    /// 解像度
    Eigen::Vector2i resolution;

    Film() = default;

    /**
     *
     * @param resolutionHeight
     * @param aspectRatio
     * @param filmHeight
     */
    Film(const int &resolutionHeight, const double &aspectRatio, const double &filmHeight)
    : resolution(Eigen::Vector2i{resolutionHeight * aspectRatio, resolutionHeight}),
    filmSize(Eigen::Vector2d{filmHeight * aspectRatio, filmHeight}) {}

    Eigen::Vector2d pixelLocalPosition(const unsigned int &x, const unsigned int &y) const {
        return Eigen::Vector2d{(x + 0.5) / resolution.x(), (y + 0.5) / resolution.y()};
    }
};


#endif //CUDATEST_FILM_H