#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <chrono>
#include <iostream>
#include"wust_dafu/wust-runedetector.h"

using namespace cv;
using namespace std;
class Line {
public:
    float A, B, C;

    // 基本构造函数
    Line(float A = 0, float B = 0, float C = 0) : A(A), B(B), C(C) {}

    // 接受 std::tuple 作为参数的构造函数
    Line(const tuple<float, float, float>& params) {
        tie(A, B, C) = params;
    }

    // 两点求解直线方程
    void SolveLine(const Point2f& point1, const Point2f& point2) {
        A = point1.y - point2.y;
        B = point2.x - point1.x;
        C = -A * point1.x - B * point1.y;
    }

    // 根据法线经过的一点求解法线方程
    tuple<float, float, float> SolveNormalLineParameters(const Point2f& point) {
        float newA = -this->B;
        float newB = this->A;
        float newC = -newA * point.x - newB * point.y;
        return make_tuple(newA, newB, newC);
    }

    // 根据给定x值求解y值
    float SolvePointByX(float x) const {
        if (B != 0) {
            return (-C - A * x) / B;
        } else {
            return 0; // 或者返回一个错误码/抛出异常
        }
    }

    // 判断两点是否在直线同侧
    bool DetermineIfSameSide(const Point2f& point1, const Point2f& point2) const {
        float value1 = A * point1.x + B * point1.y + C;
        float value2 = A * point2.x + B * point2.y + C;
        return value1 * value2 >= 0;
    }
};
    // 拟合函数的定义
    // double BigRuneFunc(double t, double a, double omega, double c, double d) {
    //     return (-a / omega) * cos(omega * (t + c)) + (2.090 - a) * (t + c) + d;
    // }

    // double SmallRuneFunc(double t, double a, double b, double c) {
    //     return a * (t + b) + c;
    // }

    // double RuneSpeedFunc(double t, double a, double omega, double c) {
    //     return a * sin(omega * (t + c)) + (2.09 - a);
    // }

    // double CalcDeltaRunePos(double t0, double t1, double a, double omega, double c) {
    //     double p0 = (-a / omega) * cos(omega * (t0 + c)) + (2.090 - a) * t0;
    //     double p1 = (-a / omega) * cos(omega * (t1 + c)) + (2.090 - a) * t1;
    //     return p1 - p0;
    // }
    void RuneDetector::Reset() {
        result_img_ = cv::Mat();
        binary_img_ = cv::Mat();
        small_lights_.clear();
        big_lights_.clear();
        huge_lights_.clear();
        circular_lights_.clear();
        runes_.clear();
        
        center_ = cv::Point2f(0, 0); // 能量机关旋转中心
        target_center_ = cv::Point2f(0, 0); // 待打击的位置
        target_angles_.clear();
        key_angles_.clear();
        predict_angles_.clear();
        predict_diffs_.clear();
        rune_speeds_.clear();
        rune_speeds_smoothed_.clear();
        time_stamps_.clear();
        start_time_ = std::chrono::steady_clock::now();
        // tracker = Tracker(); 如果Tracker需要重置，调用tracker.Reset();
        state = DectorState::NO_ACTION;
        lose_target_count = 0;
        rune_params = {1.0, 1.9, 1.0, 1.0};
        rune_speed_params = {1.0, 1.9, 1.0};
    }
// double RuneDetector::CalcTargetAngle(const cv::Point2f& pt) {
// //     float dy = pt.y - center_.y;
// //     float dx = pt.x - center_.x;
// //     double angle = std::atan2(dy, dx);
// //     if (angle < 0) {
// //         angle += 2 * M_PI;
// //     }
//     // return angle;
// }
// void RuneDetector::FitTraceCurve() {

// }

// void RuneDetector::CollectData() {
//     if (target_angles_.size() >= 50) {
//         state = DectorState::PREDICTING;
//         // FitTraceCurve();
//     }

//     cv::Point2f target_center(0, 0); 
//     for (auto& rune : runes_) {
//         if (rune.type == RuneType::DISACTIVED) {
//             target_center = GetTargetCenter(rune.rect);
//             target_center_ = target_center;
//             break;
//         }
//     }
//     double target_angle = CalcTargetAngle(target_center);
//     target_angles_.push_back(target_angle);

//     if (target_angles_.size() >= 2) {
//         double rune_speed = 50 * (target_angles_.back() - target_angles_.at(target_angles_.size() - 2));
//         if (std::abs(rune_speed) > 2) {
//             rune_speed = rune_speeds_.back();
//         }
//         rune_speeds_.push_back(rune_speed);
//         rune_speeds_smoothed_.push_back(rune_speed);
//         // 更新时间戳
//         auto now = std::chrono::steady_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
//         time_stamps_.push_back(duration);
//     }
//     Filter();
// }
// void RuneDetector::Filter() {
   
// }
// void RuneDetector::Predict(float dt) {
//     CollectData();

//     double a = rune_speed_params[0];
//     double omega = rune_speed_params[1];
//     double b = 2.09 - a;
//     double c = rune_speed_params[2];
//     double t0 = time_stamps_.back();
//     double t1 = t0 + dt;
//     double dp = CalcDeltaRunePos(t0, t1, a, omega, c);

//     double rotation_radius = cv::norm(target_center_ - center_);
//     if (rotation_radius < 0.1) {
//         return;
//     }

//     double predict_pos_angle = target_angles_.back() + dp;
//     if (predict_pos_angle > 2 * CV_PI) {
//         predict_pos_angle -= 2 * CV_PI;
//     }
//     predict_angles_.push_back(predict_pos_angle);

//     double x = center_.x + rotation_radius * cos(predict_pos_angle);
//     double y = center_.y + rotation_radius * sin(predict_pos_angle);
//     cv::circle(result_img_, cv::Point(int(x), int(y)), 5, cv::Scalar(0, 0, 255), -1);
//     cv::putText(result_img_, "predict pos", cv::Point(int(x), int(y) - 20), 
//                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
// }

//处理图像
void RuneDetector::PreprocessImage(const cv::Mat& rgb_img) {
    result_img_ = rgb_img.clone();
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);//灰度图
    cv::threshold(gray_img, binary_img_, binary_thresh, 255, cv::THRESH_BINARY);//二值化
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary_img_, binary_img_, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);//腐蚀找到各种灯条分类轮廓
}

void RuneDetector::FindLights() {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    small_lights_.clear();
    big_lights_.clear();
    huge_lights_.clear();
    circular_lights_.clear();

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 200) {
            continue;
        }

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double height = std::max(rect.size.width, rect.size.height);
        double width = std::min(rect.size.width, rect.size.height);
        
        if (height / width > 3) {
            continue;
        }

        double aspect_ratio = height / width;
        Light light(contour, rect);
        //灯条分类
        if (aspect_ratio < 1.1) {
            light.type = LightType::CIRCULAR_LIGHT;
            circular_lights_.push_back(light);
        } else if (std::abs(aspect_ratio - 1.25) < 0.1) {
            light.type = LightType::BIG_LIGHT;
            big_lights_.push_back(light);
        } else if (std::abs(aspect_ratio - 2.7) < 0.2) {
            light.type = LightType::SMALL_LIGHT;
            small_lights_.push_back(light);
        } else if (std::abs(aspect_ratio - 1.85) < 0.25) {
            double area = rect.size.width * rect.size.height;
            if (area < 20000) {
                continue;
            }
            light.type = LightType::HUGE_LIGHT;
            huge_lights_.push_back(light);
        }
    }
}

void RuneDetector::DistinguishingLights() {
    runes_.clear();
    lose_target_count += 1;

    Light* disactived_huge_light = nullptr;
    bool disactived_is_found = false;
    for (auto& light : huge_lights_) {
        double areaRatio = cv::contourArea(light.contour) / (light.rect.size.width * light.rect.size.height);
        Rune rune(light.rect);
        if (areaRatio > 0.6) {
            rune.type = RuneType::ACTIVED;
        } else if (!disactived_is_found) {
            rune.type = RuneType::DISACTIVED;
            disactived_huge_light = &light;
            disactived_is_found = true;
            lose_target_count = 0;
        }
        runes_.push_back(rune);
    }

    if (!disactived_huge_light) return;

    // 获取待激活扇叶外接矩形四个顶点的坐标
    cv::Point2f box[4];
    disactived_huge_light->rect.points(box);

    std::vector<cv::Point2f> line1_points = {box[0], box[1]};
    std::vector<cv::Point2f> line2_points = {box[2], box[3]};
    if (cv::norm(box[0] - box[1]) >= cv::norm(box[1] - box[2])) {
        line1_points = {box[1], box[2]};
        line2_points = {box[0], box[3]};
    }

    Line line1;
    line1.SolveLine(line1_points[0], line1_points[1]);

    for (auto& light : circular_lights_) {
        auto center = light.rect.center;
        auto normal_line_params = line1.SolveNormalLineParameters(center);
        Line line1_normal_line(std::get<0>(normal_line_params), std::get<1>(normal_line_params), std::get<2>(normal_line_params));

        bool line2_point_in_same_side = line1_normal_line.DetermineIfSameSide(line2_points[0], line2_points[1]);
        
        Rune rune(light.rect);
        if (!line2_point_in_same_side) {
            rune.type = RuneType::R_SIGN;
            runes_.push_back(rune);
            center_ = center;
            if (state == DectorState::NO_ACTION) {
                state = DectorState::COLLECTING;
            }
        } else {
            rune.type = RuneType::TargetRing;
            runes_.push_back(rune);
        }
    }
    // DrawRunes();
}
cv::Point2f RuneDetector::GetTargetCenter(const cv::RotatedRect& rect) {
    cv::Point2f vertices[4];
    rect.points(vertices);
    std::vector<cv::Point2f> box(vertices, vertices + 4);

    cv::Point2f dst_pts[3];
    if (cv::norm(box[0] - box[1]) < cv::norm(box[1] - box[2])) {
        dst_pts[0] = cv::Point2f(0, 0);
        dst_pts[1] = cv::Point2f(0, 75);
        dst_pts[2] = cv::Point2f(150, 75);
    } else {
        dst_pts[0] = cv::Point2f(0, 75);
        dst_pts[1] = cv::Point2f(150, 75);
        dst_pts[2] = cv::Point2f(150, 0);
    }

    cv::Mat M = cv::getAffineTransform(box.data(), dst_pts);
    cv::Mat transformed;
    cv::warpAffine(binary_img_, transformed, M, cv::Size(150, 75));

    cv::Mat closed, opened;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(transformed, closed, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(closed, opened, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(opened, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& contour : contours) {
        cv::RotatedRect minRect = cv::minAreaRect(contour);
        cv::Point2f pt = minRect.center;

        std::vector<cv::Point2f> pts = {pt};
        cv::Mat M_inv = cv::Mat::zeros(2, 3, CV_32F); // Create an empty 2x3 matrix
        cv::invertAffineTransform(M, M_inv);
        cv::transform(pts, pts, M_inv); // Use cv::transform for affine transformations
        cv::Point2f pos = pts[0];

        cv::circle(result_img_, pos, 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(result_img_, "target pos", pos - cv::Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        return pos;
    }

    return cv::Point2f(0,0);
}





void RuneDetector::DrawLights() {
    int dy = -15;
    
    // 处理大灯
    for (const auto& light : huge_lights_) {
        // 定义一个包含四个顶点的vector
        std::vector<cv::Point2f> boxPoints(4);
        
        // 使用cv::boxPoints获取RotatedRect的四个顶点
        cv::boxPoints(light.rect, boxPoints);

        // 绘制轮廓
        cv::polylines(result_img_, std::vector<std::vector<cv::Point>>{std::vector<cv::Point>(boxPoints.begin(), boxPoints.end())}, true, cv::Scalar(255, 255, 0), 2);
        
        // 计算面积并绘制
        double area = cv::contourArea(boxPoints);
        cv::putText(result_img_, "area=" + std::to_string(area),
                    cv::Point(light.rect.center.x, light.rect.center.y + dy),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }
    // // Circular lights
    // for (const auto& light : circular_lights_) {
    //     cv::ellipse(result_img_, light.rect, cv::Scalar(0, 255, 0), 2);
    //     cv::putText(result_img_, "circular light",
    //                 cv::Point(light.rect.center.x, light.rect.center.y + dy),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    // }

    // // Big lights
    // for (const auto& light : big_lights_) {
    //     std::vector<cv::Point2f> box;
    //     cv::boxPoints(light.rect, box);
    //     cv::drawContours(result_img_, std::vector<std::vector<cv::Point>>{{box.begin(), box.end()}}, 0, cv::Scalar(255, 0, 0), 2);
    //     cv::putText(result_img_, "big light",
    //                 cv::Point(light.rect.center.x, light.rect.center.y + dy),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    // }

    // // Small lights
    // for (const auto& light : small_lights_) {
    //     std::vector<cv::Point2f> box;
    //     cv::boxPoints(light.rect, box);
    //     cv::drawContours(result_img_, std::vector<std::vector<cv::Point>>{{box.begin(), box.end()}}, 0, cv::Scalar(0, 0, 255), 2);
    //     cv::putText(result_img_, "small light",
    //                 cv::Point(light.rect.center.x, light.rect.center.y + dy),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    // }
}
cv::Point2f RuneDetector::DrawRunes() {
    int dy = 10;
    cv::Point2f pos;
    for (const auto& rune : runes_) {
        // 创建一个Mat对象来接收cv::boxPoints的输出
        cv::Mat boxMat;
        cv::boxPoints(rune.rect, boxMat); 

        // 将Mat转换为vector<Point2f>
        std::vector<cv::Point2f> boxPoints(4);
        for (int i = 0; i < 4; ++i) {
            boxPoints[i] = boxMat.at<cv::Point2f>(i, 0);
        }

        // 将Point2f转换为Point，因为drawContours函数期望的是Point类型
        std::vector<cv::Point> points;
        for (const auto& pt : boxPoints) {
            points.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }

        // 绘制矩形
        std::vector<std::vector<cv::Point>> contours{points};
        cv::drawContours(result_img_, contours, -1, cv::Scalar(255, 255, 0), 2);

        // 根据rune的类型绘制对应的文字
        
        std::string text;
        switch (rune.type) {
            case RuneType::DISACTIVED: text = "disactived" ;pos=GetTargetCenter(rune.rect);break;
            case RuneType::ACTIVED:    text = "actived"; break;
            case RuneType::R_SIGN:     text = "R sign"; dy = -20; break; // 调整文字位置
            case RuneType::TargetRing: text = "target ring"; break;
            default: text = "invalid"; break;
        }
        
        cv::putText(result_img_, text,
                    cv::Point(rune.rect.center.x, rune.rect.center.y + dy),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                    return pos;
    }
    
}

