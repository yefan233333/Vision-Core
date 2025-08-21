// #include "vc/detector/rune_detector.h"


// using namespace std;
// using namespace cv;

// /**
//  * @brief 更新特征组
//  *
//  * @param[in] matched_features 匹配好的特征
//  * @param[out] features 特征组
//  */
// static inline void setFeatures(const std::vector<std::tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> &matched_features, std::vector<FeatureNode_ptr> &features)
// {
//     // 更新特征组
//     for (const auto &[target, center, fan] : matched_features)
//     {
//         if (target)
//             features.emplace_back(target);
//         if (center)
//             features.emplace_back(center);
//         if (fan)
//             features.emplace_back(fan);
//     }
// }

// /**
//  * @brief 设置序列组的基础属性
//  */
// inline static void setBaseProperties(std::vector<FeatureNode_ptr> &groups, const GyroData &gyro_data, int64 record_time)
// {
//     for (auto &group : groups)
//     {
//         auto rune_group = dynamic_pointer_cast<RuneGroup>(group);
//         rune_group->setGyroData(gyro_data);
//         rune_group->setTick(record_time);
//     }
// }


// void RuneDetector::detect(std::vector<group_ptr> &groups, cv::Mat &src, PixChannel color,
//                           const GyroData &gyro_data, int64 record_time)
// {
//     TIMER_START(rune_detect);
//     if (groups.size() > 1)
//         SRVL_Error(SRVL_StsBadArg, "Size of the argument \"groups\" is greater than 1");
//     __src = src;
//     __tick = record_time;
//     __gyro_data = gyro_data;
//     __current_combos.clear();
//     __current_features.clear();
//     // 初始化存储信息
//     if (groups.empty())
//         groups.emplace_back(RuneGroup::make_group());

//     setBaseProperties(groups, __gyro_data, __tick);

//     auto rune_group = dynamic_pointer_cast<RuneGroup>(groups.front());
//     // 二值化处理图像
//     PixChannel ch_minus = color == RED ? BLUE : RED;
//     int thesh = color == RED ? rune_detector_param.GRAY_THRESHOLD_RED : rune_detector_param.GRAY_THRESHOLD_BLUE;
//     // thesh = 100;
//     // thesh = 100;

//     TIMER_START(binary);
//     Mat bin = binary(__src, color, ch_minus, thesh);
//     TIMER_STOP(binary);

//     if (rune_debug_tools_param.ENABLE_BINARIZATION_DEBUG)
//     {
//         namedWindow("bin", WINDOW_NORMAL);
//         resizeWindow("bin", Size(640, 512));
//         imshow("bin", bin);
//     }

//     // Mat bin = binary(__src,color,ch_minus,60);
//     // Mat bin = binary(__src, color, ch_minus, 95);

//     // Mat bin = binary(__src, color, ch_minus, 160);
//     // Mat bin = binary(__src, color, ch_minus, 180);
//     // Mat bin = binary(__src, color, ch_minus, 200);

// #ifdef rune_detect_debug

// #endif

// #if rune_detect_debug
//     // 更新调试图像
//     // contour_visualizer->update(__src);
//     TIMER_START(update_contour_viz);
//     rune_contour_viz->update(__src);
//     TIMER_STOP(update_contour_viz);

// #endif

// #ifdef rune_fan_debug_0

//     if (rune_debug_tools_param.ENABLE_RUNE_DEBUG_CLASS)
//     {
//         TIMER_START(update_rune_debug);
//         RuneDebug::get().getImg() = __src.clone();
//         RuneDebug::get().addFrame();
//         TIMER_STOP(update_rune_debug);
//     }

//     // namedWindow("bin", WINDOW_NORMAL);
//     // resizeWindow("bin", Size(640, 512));
//     // imshow("bin", bin);

// #endif

//     vector<tuple<RuneT_ptr, RuneC_ptr, RuneF_ptr>> matched_features{}; // 匹配好的特征

//     // 更新函数
//     auto updateRuneGroup = [&]() -> bool
//     {
//         // 尝试获取神符中心的估计位置

//         auto center_estimation_tuple = [&]() -> std::tuple<bool, cv::Point2f>
//         {
//             if (!rune_group->isSetCenterEstimationInfo())
//             {
//                 return std::make_tuple(false, cv::Point2f(0, 0));
//             }
//             auto center_estimation_info = rune_group->getCenterEstimationInfo();
//             if (!center_estimation_info.is_valid)
//             {
//                 return std::make_tuple(false, cv::Point2f(0, 0));
//             }
//             // 判断时间是否相差过久
//             auto time_diff = (__tick - center_estimation_info.tick) / cv::getTickFrequency();
//             if (time_diff > rune_detector_param.ESTIMATE_CENTER_VALID_TIME)
//             {
//                 return std::make_tuple(false, cv::Point2f(0, 0));
//             }
//             // 进行坐标系变换，获取神符中心的估计位置
//             auto center_to_gyro = center_estimation_info.pos_to_gyro;
//             auto cam_to_gyro = rune_group->calcCamToGyroPnpData(gyro_data);
//             auto center_to_cam = ResultPnP<float>(Vec3f(0, 0, 0), center_to_gyro) + cam_to_gyro.inv();
//             // 重投影转化为图像坐标系
//             vector<Point2f> center_estimation_points;

//             vector<Point3f> center_estimation_points_3d;
//             center_estimation_points_3d.emplace_back(Point3f(0, 0, 0));
//             cv::projectPoints(center_estimation_points_3d, center_to_cam.rvec(), center_to_cam.tvec(),
//                               camera_param.cameraMatrix, camera_param.distCoeff,
//                               center_estimation_points);
//             Point2f result_point = center_estimation_points[0];
//             // #ifdef rune_fan_debug_0
//             //             // 绘制估计的神符中心位置
//             //             RuneDebug::get().drawCircle(result_point, 200,Scalar(200,100,0),5);

//             // #endif
//             // 检查该点是否有效
//             bool is_valid = [&]() -> bool
//             {
//                 if (std::isnan(result_point.x) || std::isnan(result_point.y))
//                 {
//                     return false;
//                 }
//                 // row 边界
//                 constexpr int limit_ratio = 3.0;
//                 int left_limit = -limit_ratio * __src.cols;
//                 int right_limit = (limit_ratio + 1) * __src.cols;
//                 int top_limit = -limit_ratio * __src.rows;
//                 int bottom_limit = (limit_ratio + 1) * __src.rows;

//                 // 检查边界
//                 if (result_point.x < left_limit || result_point.x > right_limit ||
//                     result_point.y < top_limit || result_point.y > bottom_limit)
//                 {
//                     return false;
//                 }
//                 return true;
//             }();
//             if (!is_valid)
//             {
//                 return std::make_tuple(false, cv::Point2f(0, 0));
//             }
//             // 返回结果
//             return std::make_tuple(true, center_estimation_points[0]);
//         }();

//         // 尝试查找所有的神符特征
//         TIMER_START(find_features);
//         if (!findFeatures(bin, center_estimation_tuple, __current_features, matched_features))
//             return false;
//         TIMER_STOP(find_features);
//         // cout <<"神符特征查找成功" << endl;
//         // 尝试获取PNP解算数据
//         TIMER_START(get_pnp_data);
//         ResultPnP<float> runeGroup_to_cam;
//         if (!getPnpData(runeGroup_to_cam, rune_group, matched_features))
//             return false;
//         TIMER_STOP(get_pnp_data);

//         TIMER_START(update_rune_group);
//         // 更新序列组
//         if (!rune_group->update(runeGroup_to_cam, __gyro_data, __tick))
//             return false;
//         TIMER_STOP(update_rune_group);

//         TIMER_START(get_runes);
//         // 尝试获取所有的神符组合体
//         if (!getRunes(__current_combos, rune_group, matched_features, rune_group->getCamPnpData()))
//             return false;
//         TIMER_STOP(get_runes);

//         // 更新神符中心估计信息

//         return true;
//     };

//     // 掉帧状态下的更新函数
//     auto updateRuneGroupVanish = [&]() -> bool
//     {
//         // 掉帧状态处理
//         if (!rune_group->visibilityProcess(false))
//             return false;

//         TIMER_START(get_cam_pnp_data);

//         ResultPnP<float> runeGroup_to_cam;
//         if (!rune_group->getCamPnpDataFromPast(runeGroup_to_cam))
//             return false;
//         TIMER_STOP(get_cam_pnp_data);

//         TIMER_START(update_rune_group);
//         // 更新序列组
//         if (!rune_group->update(runeGroup_to_cam, __gyro_data, __tick))
//             return false;
//         TIMER_STOP(update_rune_group);

//         TIMER_START(get_runes);
//         // 尝试获取所有的神符组合体
//         if (!getRunes(__current_combos, rune_group, rune_group->getLastFrameFeatures(), rune_group->getCamPnpData()))
//             return false;
//         TIMER_STOP(get_runes);

//         TIMER_START(set_features);
//         // 更新特征组
//         setFeatures(rune_group->getLastFrameFeatures(), __current_features);
//         TIMER_STOP(set_features);

//         return true;
//     };

//     // 尝试更新神符序列组
//     // 是否为掉帧更新
//     bool is_vanish_update = false;
//     if (!updateRuneGroup())
//     {
//         is_vanish_update = true;
//         // 若更新失败，尝试掉帧状态下的更新
//         if (!updateRuneGroupVanish())
//         {
//             is_vanish_update = true;
//             // 若掉帧状态下的更新失败，重新构建神符序列组
//             groups = {RuneGroup::make_group()};
//             setBaseProperties(groups, __gyro_data, __tick);
//             return;
//         }
//     }
//     if (!is_vanish_update)
//     {
//         // 若更新成功，清空掉帧数量
//         rune_group->visibilityProcess(true);
//     }

//     // cout << "更新神符序列组成功" << endl;

//     if (__current_combos.empty())
//     {
//         SRVL_Error(SRVL_StsBadArg, "组合体为空");
//     }

//     TIMER_START(match);
//     // 匹配
//     auto &rune_trackers = rune_group->getTrackers();
//     if (!match(__current_combos, rune_trackers, is_vanish_update))
//     {
//         rune_trackers.clear();
//         match(__current_combos, rune_trackers, is_vanish_update);
//     }
//     TIMER_STOP(match);

// #if rune_detect_debug
//     // contour_visualizer->refresh();
//     TIMER_START(refresh_contour_viz);
//     rune_contour_viz->refresh();
//     TIMER_STOP(refresh_contour_viz);
// #endif

//     TIMER_START(sync);
//     rune_group->sync(__gyro_data, __tick);
//     TIMER_STOP(sync);

//     TIMER_STOP(rune_detect);
// }
