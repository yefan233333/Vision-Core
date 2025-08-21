#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_combo.h"
#include "vc/camera/camera_param.h"

#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan.h"

#include "vc/feature/rune_target_inactive.h"
#include "vc/feature/rune_target_active.h"

#include "vc/feature/rune_fan_inactive.h"
#include "vc/feature/rune_fan_active.h"

#include "vc/feature/rune_target_param.h"
#include "vc/feature/rune_center_param.h"
#include "vc/feature/rune_fan_param.h"

using namespace std;
using namespace cv;

inline Point2f getCenter(const FeatureNode_cptr feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getCenter();
}

/**
 * @brief 设置五片空神符的PNP数据
 *
 * @param pnp_datas 五片空神符的PNP数据
 * @param camera_pnp_data 相机坐标系下的神符中心PNP数据
 * @return 是否设置成功
 */
inline bool setTempRunePnpData(vector<PoseNode>&pnp_datas, const PoseNode &camera_pnp_data)
{
    if (pnp_datas.size() != 5)
    {
        VC_THROW_ERROR("The pnp_datas size is not equal to 5");
    }
    for (size_t i = 0; i < 5; i++) // 设置五片空神符的PNP数据
    {
        float deivation_angle = static_cast<float>(i) * 72;
        // 旋转矩阵，绕Z轴旋转
        PoseNode rune_pnp_data = camera_pnp_data;
        rune_pnp_data.rotate_z(deivation_angle);
        // 神符靶心相对于旋转中心的平移
        Vec3f target_to_rotCenter_tvec(-1 * rune_center_param.TRANSLATION(0), -1 * rune_center_param.TRANSLATION(1), 0);
        Vec3f rune_tvec = rune_pnp_data.tvec() + rune_pnp_data.rmat() * target_to_rotCenter_tvec;
        pnp_datas[i] = PoseNode(rune_pnp_data.rmat(), rune_tvec);
    }
    return true;
}

/**
 * @brief 设置五片空神符的类型
 *
 * @param[out] types 五片空神符的类型
 * @param[in] pnp_datas 五片空神符的PNP数据
 * @param[in] features_visible 特征组
 * @param[in] group 神符组
 */
inline bool setTempRuneType(vector<RuneType> &types, const vector<PoseNode> &pnp_datas, const vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> &features_visible, const FeatureNode_ptr &group)
{
    // 获取重投影后的点
    auto getProjectPoint = [&](const cv::Matx31f &src, const PoseNode &pnp_data) -> cv::Point2f
    {
        vector<Point2f> project_points;
        projectPoints(src, pnp_data.rvec(), pnp_data.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, project_points);
        return project_points[0];   
    };

    if (types.size() != 5 || pnp_datas.size() != 5)
    {
        VC_THROW_ERROR("The types size is not equal to 5");
    }

    // 通过最新获取到的神符进行更新
    unordered_set<size_t> pending_update_idxs = {0, 1, 2, 3, 4}; // 待更新类型的编号

    if (features_visible.size() > 5)
    {
        VC_THROW_ERROR("The features_visible size is greater than 5");
    }

    for (auto &feature_group : features_visible) // 利用当前帧的可见特征更新类型
    {
        auto [target, center, fan] = feature_group;
        RuneType type{}; // 当前特征组的神符类型

        if (fan != nullptr)
        {
            type = RuneFan::cast(fan)->getActiveFlag() ? RuneType::STRUCK : RuneType::PENDING_STRUCK;
        }
        else if (target != nullptr)
        {
            type = RuneTarget::cast(target)->getActiveFlag() ? RuneType::STRUCK : RuneType::PENDING_STRUCK;
        }
        else
        {
            VC_THROW_ERROR("The target and fan is nullptr");
        }

        // 获取最近的空白神符
        int min_distance_idx = -1;
        float min_distance = numeric_limits<float>::max();
        for (const auto &num : pending_update_idxs)
        {
            Point2f p1 = (target) ? getCenter(target) : getCenter(fan);
            Point2f p2 = (target) ? getProjectPoint(rune_target_param.TRANSLATION, pnp_datas[num])
                                  : getProjectPoint(RuneFan::cast(fan)->getActiveFlag() ? rune_fan_param.ACTIVE_TRANSLATION : rune_fan_param.INACTIVE_TRANSLATION, pnp_datas[num]);

            float distance = getDist(p1, p2);
            if (distance < min_distance)
            {
                min_distance = distance;
                min_distance_idx = num;
            }
        }
        if (min_distance_idx < 0 || min_distance_idx >= 5)
        {
            break;
        }
        types[min_distance_idx] = type;
        pending_update_idxs.erase(min_distance_idx);
    }

    // 对于那些当前帧没有找到对应特征组的神符，全部设置为未击打
    for (auto &idx : pending_update_idxs)
    {
        types[idx] = RuneType::UNSTRUCK;
    }

    return true;
}

/**
 * @brief 获取所有的神符特征
 *
 * @param[in] rune_pnp_datas 神符的PNP数据
 * @param[in] types 神符的类型
 * @param[out] rune_features 所有的神符特征(靶心、中心、扇叶)
 */
inline bool setAllfeatures(const vector<PoseNode> &rune_pnp_datas, const vector<RuneType> &types, vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> &rune_features)
{
    if (rune_pnp_datas.size() != 5 || types.size() != 5)
    {
        VC_THROW_ERROR("The pnp_datas size is not equal to 5");
    }
    rune_features.clear();
    rune_features.resize(rune_pnp_datas.size());
    for (size_t i = 0; i < rune_pnp_datas.size(); i++)
    {
        //------------------ 获取靶心特征 -------------------
        FeatureNode_ptr target = nullptr;
        PoseNode target_to_rune(rune_target_param.ROTATION, rune_target_param.TRANSLATION);
        const PoseNode &rune_to_cam = rune_pnp_datas[i];
        if (types[i] == RuneType::STRUCK)
        {
            PoseNode target_to_cam = target_to_rune + rune_to_cam;
            target = RuneTargetActive::make_feature(target_to_cam);
        }
        else if (types[i] == RuneType::UNSTRUCK || types[i] == RuneType::PENDING_STRUCK)
        {
            PoseNode target_to_cam = target_to_rune + rune_to_cam;
            target = RuneTargetInactive::make_feature(target_to_cam);
        }
        else
        {
            VC_ERROR_INFO("The type is not STRUCK or PENDING_STRUCK or PENDING_UNSTRUCK");
        }
        //------------------ 获取神符中心特征 -------------------
        PoseNode center_to_rune(rune_center_param.ROTATION, rune_center_param.TRANSLATION);
        PoseNode center_to_cam = center_to_rune + rune_to_cam;
        FeatureNode_ptr center = RuneCenter::make_feature(center_to_cam);
        //------------------ 获取扇叶特征 -------------------
        FeatureNode_ptr fan = nullptr;
        if (types[i] == RuneType::STRUCK)
        {
            PoseNode fan_to_rune(rune_fan_param.ACTIVE_ROTATION, rune_fan_param.ACTIVE_TRANSLATION);
            PoseNode fan_to_cam = fan_to_rune + rune_to_cam;
            fan = RuneFan::make_feature(fan_to_cam, true);
        }
        else if (types[i] == RuneType::PENDING_STRUCK || types[i] == RuneType::UNSTRUCK)
        {
            PoseNode fan_to_rune(rune_fan_param.INACTIVE_ROTATION, rune_fan_param.INACTIVE_TRANSLATION);
            PoseNode fan_to_cam = fan_to_rune + rune_to_cam;
            fan = RuneFan::make_feature(fan_to_cam, false);
        }
        else
        {
            VC_ERROR_INFO("The type is not STRUCK or PENDING_STRUCK or PENDING_UNSTRUCK");
        }
        rune_features[i] = make_tuple(target, center, fan);
    }

    // 异常检测所有特征是否都构造成功
    for (auto &feature : rune_features)
    {
        auto [target, center, fan] = feature;
        if (target == nullptr || center == nullptr || fan == nullptr)
        {
            return false;
        }
    }

    return true;
}

bool RuneDetector::getRunes(std::vector<FeatureNode_ptr> &current_combos,
                            const FeatureNode_ptr &group,
                            const std::vector<std::tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> &features_visible,
                            const PoseNode &camera_pnp_data) const
{
    if (group == nullptr)
    {
        return false;
    }
    if (features_visible.empty()) // 没有找到特征组,返回
    {
        return false;
    }
    vector<tuple<FeatureNode_ptr, FeatureNode_ptr, FeatureNode_ptr>> pnp_runes(5); // 使用PNP解算得到的神符
    vector<PoseNode> pnp_rune_datas(5);                  // 五片PNP神符的数据
    vector<RuneType> pnp_rune_types(5);                          // 五片空神符的类型

    if (setTempRunePnpData(pnp_rune_datas, camera_pnp_data) == false)
        return false; // 设置五片PNP神符的数据

    if (setTempRuneType(pnp_rune_types, pnp_rune_datas, features_visible, group) == false)
        return false; // 设置五片空神符的类型

    if (setAllfeatures(pnp_rune_datas, pnp_rune_types, pnp_runes) == false)
        return false; // 设置五片空神符的特征

    if (pnp_rune_types.size() != 5 || pnp_rune_datas.size() != 5 || pnp_runes.size() != 5)
    {
        VC_THROW_ERROR("The rune_types size is not equal to 5");
    }

    // 构造神符组合体
    current_combos.clear();
    for (size_t i = 0; i < 5; i++)
    {
        auto [target, center, fan] = pnp_runes[i];
        if (target == nullptr || center == nullptr || fan == nullptr)
        {
            continue;
        }
        auto rune = RuneCombo::make_feature(pnp_rune_datas[i], pnp_rune_types[i], getGyroData(), getTick());
        if (rune)
        {
            current_combos.push_back(dynamic_pointer_cast<RuneCombo>(rune));
        }
    }

    return true;
}