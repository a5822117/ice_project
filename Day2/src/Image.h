//
// Created by kango on 2023/03/14.
//

#ifndef DAY_1_IMAGE_H
#define DAY_1_IMAGE_H

#include "Color.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class Image {
public:
    int width;
    int height;

    Color *pixels;

    Image() = default;

    Image(const int &width, const int &height);

    ~Image();

    void save(const std::string &fname) const;

    Image apply_gamma_correction() const;

    double getMaxLuminance() const;

    void reinhard_tone_mapping() const;

    Image apply_reinhard_extended_tone_mapping() const;

    Image apply_gaussian_blur(const unsigned int &k_width, const unsigned int &k_height, const double &sigma=1.0) const;

    Image apply_bilateral_filter(const unsigned int &d, const unsigned int &sigma_color,
                                 const unsigned int &sigma_space) const;

    cv::Mat toCvMat() const;

    cv::Mat toCvMat8UC3() const;

    Image resize(const double &scaleRatio) const;

    void show(const std::string &name="display") const;

    static Image cvMatToImage(const cv::Mat &mat);

    static Image cvMat8U3CToImage(const cv::Mat &mat);

    static Image loadImage(const std::string &filename);
};


#endif //DAY_1_IMAGE_H