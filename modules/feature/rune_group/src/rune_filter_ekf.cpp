#include "vc/feature/rune_filter_ekf.h"
#include "vc/feature/rune_filter_ekf_param.h"
#include <iostream>

using namespace cv;

RuneFilterEKF_CV::RuneFilterEKF_CV(RuneFilterDataType type):
    _data_type(type)
{
    x_ = cv::Mat::zeros(12, 1, CV_64F);

    // 初始协方差
    P_ = cv::Mat::eye(12, 12, CV_64F) * rune_ekf_param.INIT_POS_VAR;
    for (int i = 6; i < 12; i++)
        P_.at<double>(i, i) = rune_ekf_param.INIT_VEL_VAR;

    // 过程噪声 Q
    Q_ = cv::Mat::eye(12, 12, CV_64F);
    Q_(cv::Rect(0, 0, 3, 3)) *= rune_ekf_param.Q_POS;       // pos
    Q_(cv::Rect(3, 3, 3, 3)) *= rune_ekf_param.Q_ANGLE;     // angle
    Q_(cv::Rect(6, 6, 3, 3)) *= rune_ekf_param.Q_VEL;       // vel
    Q_(cv::Rect(9, 9, 3, 3)) *= rune_ekf_param.Q_ANGLE_VEL; // angle vel

    // 观测噪声 R
    R_ = cv::Mat::eye(6, 6, CV_64F);
    R_(cv::Rect(0, 0, 3, 3)) *= rune_ekf_param.R_POS;   // pos noise
    R_(cv::Rect(3, 3, 3, 3)) *= rune_ekf_param.R_ANGLE; // angle noise

    // 观测矩阵 H (6x12)
    H_ = cv::Mat::zeros(6, 12, CV_64F);
    for (int i = 0; i < 6; i++)
        H_.at<double>(i, i) = 1.0;
}

void RuneFilterEKF_CV::predict(double dt)
{
    // 状态转移矩阵 F (12x12)
    Mat F = Mat::eye(12, 12, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        F.at<double>(i, 6 + i) = dt;     // pos += vel*dt
        F.at<double>(3 + i, 9 + i) = dt; // angle += ang_vel*dt
    }

    // 状态预测
    x_ = F * x_;

    // 协方差预测
    P_ = F * P_ * F.t() + Q_;
}

void RuneFilterEKF_CV::update(const Mat &z)
{
    // 预测观测
    Mat z_pred = H_ * x_;

    // 创新
    Mat y = z - z_pred; // 6x1

    // 创新协方差
    Mat S = H_ * P_ * H_.t() + R_;

    // 卡尔曼增益 K = P * H^T * S^-1
    Mat K = P_ * H_.t() * S.inv(DECOMP_SVD);

    // 状态更新
    x_ = x_ + K * y;

    // 协方差更新
    Mat I = Mat::eye(12, 12, CV_64F);
    P_ = (I - K * H_) * P_;
}

RuneFilterStrategy::FilterOutput RuneFilterEKF_CV::filter(FilterInput input)
{
    // 初始化
    if (!initialized_)
    {
        for (int i = 0; i < 6; i++)
            x_.at<double>(i, 0) = input.raw_pos(i);

        last_tick_ = input.tick;
        initialized_ = true;
        __latest_value = input.raw_pos;
        return {__latest_value};
    }

    // 时间间隔
    dt_ = (input.tick - last_tick_) / cv::getTickFrequency();
    if (dt_ <= rune_ekf_param.MIN_DT)
        dt_ = rune_ekf_param.MIN_DT;

    last_tick_ = input.tick;

    // 预测
    predict(dt_);

    // 如果有观测则更新
    if (input.is_observation)
    {
        Mat z(6, 1, CV_64F);
        for (int i = 0; i < 6; i++)
            z.at<double>(i, 0) = input.raw_pos(i);
        update(z);
    }

    // 更新父类成员
    __filter_count++;
    for (int i = 0; i < 6; i++)
        __latest_value(i) = x_.at<double>(i, 0);

    return {__latest_value};
}

cv::Matx61d RuneFilterEKF_CV::getPredict()
{
    cv::Matx61d predict_value;
    for (int i = 0; i < 6; i++)
        predict_value(i) = x_.at<double>(i, 0);
    return predict_value;
}

bool RuneFilterEKF_CV::isValid()
{
    return initialized_;
}
