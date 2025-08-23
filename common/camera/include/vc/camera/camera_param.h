/**
 * @file camera_para.hpp
 * @author SCUT RobotLab Vision Group
 * @brief CameraParam module header file
 * @version 1.0
 * @date 2023-10-03
 *
 * @copyright Copyright 2023 (c), SCUT RobotLab Vision Group
 *
 */

#pragma once

#include <opencv2/core/types.hpp>

//! CameraParam 参数模块
struct CameraParam
{
    //! 相机曝光
    int exposure = 800;
    //! 相机Gamma值
    int gamma = 100;
    //! 相机亮度
    int brightness = 15;
    //! 相机对比度
    int contrast = 100;
    //! 相机饱和度
    int saturation = 100;
    //! 相机锐度
    int sharpness = 100;
    //! 相机全通道增益
    int gain = 64;
    //! 相机蓝色增益
    int b_gain = 100;
    //! 相机绿色增益
    int g_gain = 100;
    //! 相机红色增益
    int r_gain = 100;
    //! 相机grab模式
    int grab_mode = 1;
    //! 相机retrieve模式
    int retrieve_mode = 1;
    //! 相机自动曝光模式
    int auto_exposure = 0;
    //! 相机自动白平衡模式
    int auto_wb = 0;
    //! 相机内参
    cv::Matx33f cameraMatrix = {1250, 0, 640, 0, 1250, 512, 0, 0, 1};
    //! 畸变参数
    cv::Matx<float, 5, 1> distCoeff = cv::Matx<float, 5, 1>(0, 0, 0, 0, 0);
    //! 相机坐标系到转轴坐标系的欧拉角
    cv::Matx<float, 3, 1> camera2joint_euler_angle = cv::Matx<float, 3, 1>(0, 0, 0);
    //! 相机坐标系到转轴坐标系的旋转矩阵
    cv::Matx<float, 3, 3> cam2joint_rmat = cv::Matx<float, 3, 3>(1, 0, 0, 0, 1, 0, 0, 0, 1);
    //! 相机坐标系到转轴坐标系的平移向量
    cv::Matx<float, 3, 1> cam2joint_tvec = cv::Matx<float, 3, 1>(0, 0, 0);

    //! 相机LUT查表
    std::vector<int> lut_vec;
    //! 相机序列号
    std::string serial_number;
    //! 相机宽度和高度
    int image_width = 1440;
    //! 相机宽度和高度
    int image_height = 1080;

    //! ---------------------图传参数--------------------
    //! 图传宽度和高度
    int trans_width = 720;
    //! 图传宽度和高度
    int trans_height = 480;
    // ！ 操作手击打点坐标
    int hit_point_x = 360;
    //! 操作手击打点坐标
    int hit_point_y = 240;
    //! 图传内参
    cv::Matx33f transMatrix = {1000, 0, 360, 0, 1000, 240, 0, 0, 1};
    //! 转轴坐标系到图传坐标系的欧拉角
    cv::Matx<float, 3, 1> trans2joint_euler_angle = cv::Matx<float, 3, 1>(0, 0, 0);
    //! 转轴坐标系到图传坐标系的旋转矩阵
    cv::Matx<float, 3, 3> trans2joint_rmat = cv::Matx<float, 3, 3>(1, 0, 0, 0, 1, 0, 0, 0, 1);
    //! 转轴坐标系到图传坐标系的平移向量
    cv::Matx<float, 3, 1> trans2joint_tvec = cv::Matx<float, 3, 1>(0, 0, 0);

    YML_INIT(
        CameraParam,
        YML_ADD_PARAM(exposure);
        YML_ADD_PARAM(gamma);
        YML_ADD_PARAM(brightness);
        YML_ADD_PARAM(contrast);
        YML_ADD_PARAM(saturation);
        YML_ADD_PARAM(sharpness);
        YML_ADD_PARAM(gain);
        YML_ADD_PARAM(b_gain);
        YML_ADD_PARAM(g_gain);
        YML_ADD_PARAM(r_gain);
        YML_ADD_PARAM(grab_mode);
        YML_ADD_PARAM(retrieve_mode);
        YML_ADD_PARAM(auto_exposure);
        YML_ADD_PARAM(auto_wb);
        YML_ADD_PARAM(cameraMatrix);
        YML_ADD_PARAM(distCoeff);
        YML_ADD_PARAM(camera2joint_euler_angle);
        YML_ADD_PARAM(cam2joint_rmat);
        YML_ADD_PARAM(cam2joint_tvec);
        YML_ADD_PARAM(lut_vec);
        YML_ADD_PARAM(serial_number);
        YML_ADD_PARAM(image_width);
        YML_ADD_PARAM(image_height);
    );

};

//! CameraParam 参数模块
inline CameraParam camera_param;
