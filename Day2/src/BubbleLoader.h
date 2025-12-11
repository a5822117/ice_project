//
// BubbleLoader.h
// 気泡データの読み込みとフィルタリング
//

#ifndef BUBBLE_LOADER_H
#define BUBBLE_LOADER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

struct BubbleData {
    Eigen::Vector3d center;
    double radius;

    BubbleData(const Eigen::Vector3d &c, double r) : center(c), radius(r) {}
};

class BubbleLoader {
public:
    /// CSVファイルから気泡データを読み込む
    static std::vector<BubbleData> loadFromCSV(const std::string &filename) {
        std::vector<BubbleData> bubbles;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open bubble data file: " << filename << std::endl;
            return bubbles;
        }

        std::string line;
        int lineNum = 0;

        while (std::getline(file, line)) {
            lineNum++;

            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (const std::exception &e) {
                    std::cerr << "Warning: Could not parse value at line " << lineNum << std::endl;
                }
            }

            if (values.size() >= 4) {
                Eigen::Vector3d center(values[0], values[1], values[2]);
                double radius = values[3];
                bubbles.emplace_back(center, radius);
            }
        }

        file.close();
        std::cout << "Loaded " << bubbles.size() << " bubbles from " << filename << std::endl;

        return bubbles;
    }

    /// バウンディングボックス内の気泡のみをフィルタリング
    static std::vector<BubbleData> filterBubblesInBox(
            const std::vector<BubbleData> &bubbles,
            const Eigen::Vector3d &bboxMin,
            const Eigen::Vector3d &bboxMax,
            double margin = 0.0) {

        std::vector<BubbleData> filtered;

        for (const auto &bubble : bubbles) {
            bool inside = true;
            for (int i = 0; i < 3; ++i) {
                if (bubble.center[i] - bubble.radius - margin < bboxMin[i] ||
                    bubble.center[i] + bubble.radius + margin > bboxMax[i]) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                filtered.push_back(bubble);
            }
        }

        std::cout << "Filtered: " << filtered.size() << " / " << bubbles.size()
                  << " bubbles are inside the bounding box" << std::endl;

        return filtered;
    }

    /// 気泡の位置を変換
    static std::vector<BubbleData> translateBubbles(
            const std::vector<BubbleData> &bubbles,
            const Eigen::Vector3d &offset) {

        std::vector<BubbleData> translated;
        translated.reserve(bubbles.size());

        for (const auto &bubble : bubbles) {
            translated.emplace_back(bubble.center + offset, bubble.radius);
        }

        return translated;
    }

    /// 気泡のスケールを変換
    static std::vector<BubbleData> scaleBubbles(
            const std::vector<BubbleData> &bubbles,
            double scale) {

        std::vector<BubbleData> scaled;
        scaled.reserve(bubbles.size());

        for (const auto &bubble : bubbles) {
            scaled.emplace_back(bubble.center * scale, bubble.radius * scale);
        }

        return scaled;
    }

    /// バウンディングボックスの中心を基準にスケール
    static std::vector<BubbleData> scaleBubblesFromCenter(
            const std::vector<BubbleData> &bubbles,
            const Eigen::Vector3d &center,
            double scale) {

        std::vector<BubbleData> scaled;
        scaled.reserve(bubbles.size());

        for (const auto &bubble : bubbles) {
            Eigen::Vector3d newCenter = center + (bubble.center - center) * scale;
            scaled.emplace_back(newCenter, bubble.radius * scale);
        }

        return scaled;
    }
};

#endif // BUBBLE_LOADER_H
