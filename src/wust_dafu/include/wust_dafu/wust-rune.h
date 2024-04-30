// RuneDetection.h
#ifndef RUNE_DETECTION_H
#define RUNE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

enum class LightType {
    INVALID_LIGHT = -1,
    BIG_LIGHT = 0,
    SMALL_LIGHT = 1,
    HUGE_LIGHT = 2,
    CIRCULAR_LIGHT = 3
};

enum class RuneType {
    INVALID = -1,
    DISACTIVED = 0, // 未激活扇叶
    ACTIVED = 1,    // 已激活扇叶
    R_SIGN = 2,     // R标
    TargetRing = 3  // 靶环
};

enum class DectorState {
    NO_ACTION = 0,
    COLLECTING = 1,
    PREDICTING = 2,
    TRACKING = 3
};

class Light {
public:
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    LightType type;

    Light(const std::vector<cv::Point>& contour, const cv::RotatedRect& rect)
        : contour(contour), rect(rect), type(LightType::INVALID_LIGHT) {}
};

class Rune {
public:
    cv::RotatedRect rect;
    RuneType type;

    Rune(const cv::RotatedRect& rect)
        : rect(rect), type(RuneType::INVALID) {}
};

#endif // RUNE_DETECTION_H
