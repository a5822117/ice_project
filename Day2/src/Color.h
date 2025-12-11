//
// Created by kango on 2022/09/21.
//

#ifndef REFACTORINGIVR_COLOR_H
#define REFACTORINGIVR_COLOR_H

#include <Eigen/Dense>
#include <string>

using Color = Eigen::Vector3d;

double getLuminance(const Color &c);

Color changeLuminance(const Color &c, const double &l_out);

Color codeToColor(const std::string &colorCode);

#endif //REFACTORINGIVR_COLOR_H