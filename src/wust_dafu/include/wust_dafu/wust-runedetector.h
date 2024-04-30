// RuneDetector.h
#ifndef RUNE_DETECTOR_H
#define RUNE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include"wust-rune.h"
#include"wust-tracker.h"

class RuneDetector {
public:
     RuneDetector(int thresh = 100) : binary_thresh(thresh) {
        Reset();  // 在构造函数中调用Reset方法
    }
    void Reset();
    // 曲线拟合参数 TODO
    double CalcTargetAngle(const cv::Point2f& pt);
    void FitTraceCurve();
    void CollectData();
    void Filter();
    void Predict(float dt);
    // 图像处理
    void PreprocessImage(const cv::Mat& rgb_img); //初始化处理图像
    void FindLights(); //找到灯条分类
    void DistinguishingLights();// 区分灯条
    cv::Point2f GetTargetCenter(const cv::RotatedRect& rect);//找到目标点中心
    void DrawLights();//绘制灯条
    cv::Point2f DrawRunes();//绘制rune轮廓

     int binary_thresh;
    cv::Mat result_img_;//处理结果图像
    cv::Mat binary_img_;//二值化图像
    std::vector<Light> small_lights_;//小灯条
    std::vector<Light> big_lights_;//大灯条
    std::vector<Light> huge_lights_;//巨大灯条
    std::vector<Light> circular_lights_;//圆形灯条
    std::vector<Rune> runes_; 
    cv::Point2f center_;  // 能量机关旋转中心
    cv::Point2f target_center_;  // 待打击的位置
    std::vector<double> target_angles_;  // 待打击扇叶的角位置
    std::vector<double> key_angles_;  // 连续的角位置
    std::vector<double> predict_angles_;  // 预测的待打击扇叶的角位置
    std::vector<double> predict_diffs_;  // 预测的角位置与实际角位置的差值
    // TODO 曲线拟合参数
    std::vector<double> rune_speeds_;
    std::vector<double> rune_speeds_smoothed_;
    std::vector<double> time_stamps_;
    std::chrono::steady_clock::time_point start_time_;
    Tracker tracker; 
    DectorState state;
    int lose_target_count;
    std::vector<double> rune_params;
    std::vector<double> rune_speed_params;
   
};

#endif // RUNE_DETECTOR_H
