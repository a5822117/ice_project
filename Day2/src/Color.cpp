//
// Created by kango on 2022/12/11.
//

#include "Color.h"
#include <cstdlib>
#include <cstdint>

Color codeToColor(const std::string &colorCode) {
    const auto rgb = static_cast<uint32_t>(strtol(&colorCode[1], nullptr, 16));

    const auto red = (rgb >> 16) & 0xFF;
    const auto green = (rgb >> 8) & 0xFF;
    const auto blue = (rgb >> 0) & 0xFF;

    return Color{static_cast<double>(red) / 255.0,
                 static_cast<double>(green) / 255.0,
                 static_cast<double>(blue) / 255.0};
}

double getLuminance(const Color &c) {
    return c.dot(Eigen::Vector3d(0.2126, 0.7152, 0.0722));
}

Color changeLuminance(const Color &c, const double &l_out) {
    return c * (l_out / getLuminance(c));
}