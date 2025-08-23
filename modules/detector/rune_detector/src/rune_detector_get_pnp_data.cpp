#include "vc/detector/rune_detector.h"
#include "vc/detector/rune_detector_param.h"
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_center_param.h"
#include "vc/feature/rune_group.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

/**
 * @brief 获取参考PnP数据
 */
inline void getReferencePnpData()
{
}

inline Point2f getCenter(const FeatureNode_cptr &feature)
{
    if (!feature)
        return Point2f(0.f, 0.f);
    return feature->getImageCache().getCenter();
}

/**
 * @brief 获取神符偏移角度
 *
 * @param runes 神符组
 * @return 神符偏移角度 , 长度 = 神符组长度
 *
 * @note 单个神符的偏移角本身没有意义，只用于表示不同神符之间的理想相对角度 (即 72 的倍数)
 */
inline vector<float> getRuneDeviation(const std::vector<RuneFeatureComboConst> &runes)
{
    // 角度归一化函数   将角度限定在 0 ~ 360 之间
    auto normalizeAngle = [](float angle) -> float
    {
        while (angle > 360)
            angle -= 360;
        while (angle < 0)
            angle += 360;
        return angle;
    };

    auto temp_runes = runes;
    // 像素坐标系的角度
    map<tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>, float> angle_map;

    for (const auto &feature_group : temp_runes)
    {
        auto [target, center, fan] = feature_group;
        if ((target == nullptr && fan == nullptr) || center == nullptr)
        {
        }
        float angle;
        if (target != nullptr)
        {
            Point2f direction = getCenter(target) - getCenter(center);
            angle = rad2deg(atan2(direction.y, direction.x));
        }
        else
        {
            Point2f direction = getCenter(fan) - getCenter(center);
            angle = rad2deg(atan2(direction.y, direction.x));
        }
        // 将角度限定在 0 ~ 360 之间
        angle_map[feature_group] = normalizeAngle(angle);
    }

    // 如果获取到未激活神符，则将未激活神符的角度设置为 0 ,并更新其它神符的相对角度
    bool has_inactive_rune = false;
    float inactive_angle = 0;
    for (auto &[target, center, fan] : temp_runes)
    {
        if ((target && !RuneTarget::cast(target)->getActiveFlag()) || (fan && !RuneFan::cast(fan)->getActiveFlag()))
        {
            has_inactive_rune = true;
            inactive_angle = angle_map[{target, center, fan}];
            break;
        }
        else if (!target && !fan) // 靶心和扇叶都为空时报错
        {
            VC_THROW_ERROR("The target and fan is nullptr");
        }
    }
    if (has_inactive_rune) // 如果存在未激活神符
        for (auto &features : temp_runes)
            angle_map[features] -= inactive_angle; // 更新相对角度

    // 角度归一化处理,将角度限定在 0 ~ 360 之间
    for (auto &[features, angle] : angle_map)
        angle = normalizeAngle(angle);

    // 按角度对神符进行排序.
    sort(temp_runes.begin(), temp_runes.end(),
         [&](const RuneFeatureComboConst &rune_1,
             const RuneFeatureComboConst &rune_2)
         {
             return angle_map[rune_1] < angle_map[rune_2];
         });

    vector<float> reference_angles{};
    for (size_t i = 0; i < 5; i++)
        reference_angles.push_back(72 * static_cast<int>(i));

    // 为每个神符设置对应的3D角度
    unordered_map<float, float> rotate_deltas{}; // 记录各个 旋转角度 对应的误差量
    for (int rotate_angle = -180; rotate_angle < 180; rotate_angle++)
    {
        float delta = 0;
        for (auto &feature_group : temp_runes)
        {
            float angle = angle_map[feature_group] + rotate_angle;
            // 获取最接近的角度，并计算对应的误差量
            float min_delta = 1e5;
            for (auto &reference_angle : reference_angles)
            {
                min_delta = min(min_delta, abs(angle - reference_angle));
            }
            delta += min_delta * min_delta;
        }
        rotate_deltas[rotate_angle] = delta;
    }

    // 获取误差最小的旋转角度
    auto [rotate_angle, min_delta] = *min_element(rotate_deltas.begin(), rotate_deltas.end(), [](const auto &lhs, const auto &rhs)
                                                  { return lhs.second < rhs.second; });

    // 更新标准角度
    for (auto &angle : reference_angles)
    {
        angle -= rotate_angle;
    }

    // 为各个特征组设置其对应神符的编号 （0~4）
    map<tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>, int> rune_idx{};
    unordered_set<int> pending_num{0, 1, 2, 3, 4}; // 待匹配的编号
    for (auto &feature_group : temp_runes)
    {
        if(pending_num.empty())
            break;
        float angle = angle_map[feature_group];
        int min_delta_idx = -1;
        float min_delta = 1e5;
        for (size_t i = 0; i < 5; i++)
        {
            if (pending_num.find(static_cast<int>(i)) == pending_num.end())
                continue;
            float delta = abs(angle - reference_angles[i]);
            if (delta < min_delta)
            {
                min_delta = delta;
                min_delta_idx = static_cast<int>(i);
            }
        }
        cout << "pending_num.szie() = " << pending_num.size() << endl;
        if (min_delta_idx == -1)
        {
            VC_THROW_ERROR("The min_delta_idx is -1");
        }
        // 设置编号
        rune_idx[feature_group] = min_delta_idx;
        // 从待匹配的编号中移除
        pending_num.erase(min_delta_idx);
    }

    if(rune_idx.size() > 5)
        VC_THROW_ERROR("The rune_idx size is greater than 5");

    // 将 最小编号设置为 0 ，其余编号依次递增
    int min_idx = 1e5;
    for (auto &idx : rune_idx)
        min_idx = min(min_idx, idx.second);
    for (auto &[features, idx] : rune_idx)
        idx -= min_idx;
    vector<float> rune_deviation{};
    for (auto &feature_group : runes)
    {
        if (rune_idx.find(feature_group) == rune_idx.end())
        {
            continue;
        }
        rune_deviation.push_back(rune_idx[feature_group] * 72);
    }
    return rune_deviation;
}

/**
 * @brief 计算绕Z轴旋转后的点集
 *
 * @param[in] raw_points 原始点集
 * @param[in] angle 旋转角度
 * @param[out] rotated_points 旋转后的点集
 */
inline void rotatePoints(const vector<Point3f> &raw_points, float angle, vector<Point3f> &rotated_points)
{
    rotated_points.clear();
    // 计算旋转矩阵
    Matx33f rotate_mat = {cos(deg2rad(angle)), -sin(deg2rad(angle)), 0,
                          sin(deg2rad(angle)), cos(deg2rad(angle)), 0,
                          0, 0, 1};

    // 计算旋转后的点集
    for (const auto &point : raw_points)
    {
        Matx31f point_3d{point.x, point.y, point.z};
        Matx31f rotated_point_3d = rotate_mat * point_3d;
        rotated_points.emplace_back(rotated_point_3d(0), rotated_point_3d(1), rotated_point_3d(2));
    }

    // 对获取的3D点进行旋转
    // vector<Point3f> rotated_points_3d(rune_points_3d.size());
    // float deviation_angle = rune_deviation_angle[feature_group];
    // // 顺时针旋转
    // Matx33f rotate_mat = {cos(deg2rad(deviation_angle)), -sin(deg2rad(deviation_angle)), 0,
    //                       sin(deg2rad(deviation_angle)), cos(deg2rad(deviation_angle)), 0,
    //                       0, 0, 1};
    // for (size_t i = 0; i < rune_points_3d.size(); i++)
    // {
    //     Matx31f point_3d{rune_points_3d[i].x, rune_points_3d[i].y, rune_points_3d[i].z};
    //     Matx31f rotated_point_3d = rotate_mat * point_3d;
    //     rotated_points_3d[i] = Point3f(rotated_point_3d(0), rotated_point_3d(1), rotated_point_3d(2));
    // }
}

/**
 * @brief 获取PnP解算的点集
 *
 * @param matched_features 匹配好的特征
 * @param group 神符组
 * @return  返回PnP解算的点集 (2D点、3D点)
 */
inline bool getPnpPoints(const FeatureNode_ptr &group,
                         const std::vector<RuneFeatureComboConst> &runes,
                         std::vector<cv::Point2f> &points_2d,
                         std::vector<cv::Point3f> &points_3d,
                         std::vector<float> &point_weights)
{
    if (runes.empty())
    {
        VC_THROW_ERROR("The matched features is empty");
    }

    vector<float> deviation_angles = getRuneDeviation(runes);                                       // 获取神符偏移角度
    map<tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>, float> rune_deviation_angle{}; // 特征组对应的偏移角度
    for (size_t i = 0; i < runes.size(); i++)
    {
        rune_deviation_angle[runes[i]] = deviation_angles[i];
    }

    // 设置神符组的2D点
    vector<Point2f> group_points_2d{};
    // 设置神符组的3D点
    vector<Point3f> group_points_3d{};
    // 设置各个点在解算时的权重
    vector<float> group_points_weight{};
    bool rune_center_flag = false; // 是否已经存放了神符中心的角点
    // 设置PNP点集
    for (auto &feature_group : runes)
    {
        auto [target_feature, center_feature, fan_feature] = feature_group;
        auto target = RuneTarget::cast(target_feature);
        auto center = RuneCenter::cast(center_feature);
        auto fan = RuneFan::cast(fan_feature);
        if ((target == nullptr && fan == nullptr) || center == nullptr)
        {
            VC_THROW_ERROR("The target and fan is nullptr");
        }
        vector<Point2f> rune_points_2d{}; // 相对于图像坐标系
        vector<Point3f> rune_points_3d{}; // 靶心坐标系下的神符各个角点
        vector<float> rune_points_weight{};
        if (target != nullptr)
        {
            auto [points_2d, points_3d, points_weight] = target->getRelativePnpPoints();

            if (target->getActiveFlag())
            {
                for (int i = 0; i < 5; i++)
                {
                    rune_points_2d.insert(rune_points_2d.end(), points_2d.begin(), points_2d.end());
                    rune_points_3d.insert(rune_points_3d.end(), points_3d.begin(), points_3d.end());
                    rune_points_weight.insert(rune_points_weight.end(), points_weight.begin(), points_weight.end());
                }
            }
            else
            {
                rune_points_2d.insert(rune_points_2d.end(), points_2d.begin(), points_2d.end());
                rune_points_3d.insert(rune_points_3d.end(), points_3d.begin(), points_3d.end());
                rune_points_weight.insert(rune_points_weight.end(), points_weight.begin(), points_weight.end());
            }
        }
        if (center != nullptr && rune_center_flag == false)
        {
            rune_center_flag = true;
            auto [points_2d, points_3d, points_weight] = center->getRelativePnpPoints();

            for (int i = 0; i < 5; i++)
            {
                rune_points_2d.insert(rune_points_2d.end(), points_2d.begin(), points_2d.end());
                rune_points_3d.insert(rune_points_3d.end(), points_3d.begin(), points_3d.end());
                rune_points_weight.insert(rune_points_weight.end(), points_weight.begin(), points_weight.end());
            }
        }
        if (fan != nullptr)
        {
            auto [points_2d, points_3d, points_weight] = fan->getRelativePnpPoints();

            for (int i = 0; i < 2; i++)
            {
                rune_points_2d.insert(rune_points_2d.end(), points_2d.begin(), points_2d.end());
                rune_points_3d.insert(rune_points_3d.end(), points_3d.begin(), points_3d.end());
                rune_points_weight.insert(rune_points_weight.end(), points_weight.begin(), points_weight.end());
            }
        }

        // 对获取的3D点进行旋转
        vector<Point3f> rotated_points_3d(rune_points_3d.size());
        // 获取当前特征组的偏移角度
        float deviation_angle = rune_deviation_angle[feature_group];
        // 靶心坐标系相对于旋转中心坐标系的偏移量
        Point3f rotCenter_to_target(rune_center_param.TRANSLATION(0), rune_center_param.TRANSLATION(1), 0);
        // 顺时针旋转
        Matx33f rotate_mat = {cos(deg2rad(deviation_angle)), -sin(deg2rad(deviation_angle)), 0,
                              sin(deg2rad(deviation_angle)), cos(deg2rad(deviation_angle)), 0,
                              0, 0, 1};
        for (size_t i = 0; i < rune_points_3d.size(); i++)
        {
            Matx31f point_3d{rune_points_3d[i].x, rune_points_3d[i].y, rune_points_3d[i].z}; // 靶心坐标系下的点
            point_3d(0) -= rotCenter_to_target.x;                                            // 将点坐标转换到旋转中心坐标系下
            point_3d(1) -= rotCenter_to_target.y;                                            // 将点坐标转换到旋转中心坐标系下
            Matx31f rotated_point_3d = rotate_mat * point_3d;                                // 进行旋转
            rotated_points_3d[i] = Point3f(rotated_point_3d(0), rotated_point_3d(1), rotated_point_3d(2));
        }
        // 设置点集
        group_points_2d.insert(group_points_2d.end(), rune_points_2d.begin(), rune_points_2d.end());
        group_points_3d.insert(group_points_3d.end(), rotated_points_3d.begin(), rotated_points_3d.end());
        group_points_weight.insert(group_points_weight.end(), rune_points_weight.begin(), rune_points_weight.end());
    }

    if (!(group_points_2d.size() == group_points_3d.size() && group_points_2d.size() == group_points_weight.size()))
    {
        VC_THROW_ERROR("The 2D points and 3D points size is not equal");
    }

    points_2d = move(group_points_2d);
    points_3d = move(group_points_3d);
    point_weights = move(group_points_weight);

    return true;
}

/**
 * @brief 修正PNP解算的数据
 *
 * @param group 神符组
 * @param src 源PNP数据
 * @param dst 目标PNP数据
 */
inline bool correctPnpData(const FeatureNode_ptr &group, const PoseNode &src, PoseNode &dst)
{
    dst = src;
    return true;
}

/**
 * @brief PNP异常检查
 *
 * @param pnp_data
 */
inline bool checkPnpData(const PoseNode &pnp_data)
{
    // 检查旋转向量的大小
    if (norm(pnp_data.rvec()) > 1e5)
    {
        return false;
    }
    // 检查平移向量的大小
    if (norm(pnp_data.tvec()) > 1e5)
    {
        return false;
    }
    return true;
}

/**
 * @brief 输出有效点的数量
 *
 * @note 筛去重合点后剩余的点的数量（只检测point2f）
 */
inline int getValidPointCount(const vector<Point2f> &points_2d, const vector<Point3f> &points_3d, const vector<float> &point_weights)
{
    if (points_2d.size() != points_3d.size())
    {
        VC_THROW_ERROR("The 2D points and 3D points size is not equal");
    }
    vector<Point2f> vaild_points_2d{};
    for (size_t i = 0; i < points_2d.size(); i++)
    {
        Point2f point_2d = points_2d[i];
        // 检查点是否已经存在
        bool is_exist = false;
        for (const auto &point : vaild_points_2d)
        {
            if (norm(point - point_2d) < 1e-5)
            {
                is_exist = true;
                break;
            }
        }
        // 如果点不存在，则添加到有效点集中
        if (!is_exist)
        {
            vaild_points_2d.push_back(point_2d);
        }
    }
    return vaild_points_2d.size();
}

/**
 * @brief 判断能否进行PnP解算
 */
inline bool canSolvePnP(const FeatureNode_ptr &group, const vector<Point2f> &points_2d, const vector<Point3f> &points_3d, const vector<float> &point_weights)
{
    if (group == nullptr)
    {
        VC_THROW_ERROR("The group is nullptr");
    }
    if (points_2d.empty() || points_3d.empty() || point_weights.empty())
    {
        return false;
    }
    if (points_2d.size() != points_3d.size() || points_2d.size() != point_weights.size())
    {
        VC_THROW_ERROR("The 2D points and 3D points size is not equal");
    }
    if (points_2d.size() < 4)
    {
        return false;
    }

    // 检查点的数量
    int valid_point_count = getValidPointCount(points_2d, points_3d, point_weights);
    if (valid_point_count < 4)
        return false;

    return true;
}

bool RuneDetector::getCameraPnpData(const FeatureNode_ptr &group, const vector<Point2f> &points_2d, const vector<Point3f> &points_3d, const vector<float> &point_weights, PoseNode &pnp_data) const
{
    auto rune_group = std::dynamic_pointer_cast<RuneGroup>(group);

    if (group == nullptr)
    {
        VC_ERROR_INFO("The group is nullptr");
    }
    if (points_2d.empty() || points_3d.empty() || point_weights.empty())
    {
        return false;
    }
    if (points_2d.size() != points_3d.size() || points_2d.size() != point_weights.size())
    {
        VC_ERROR_INFO("The 2D points and 3D points size is not equal");
    }

    if (!canSolvePnP(group, points_2d, points_3d, point_weights))
    {
        return false;
    }
    // 设置PnP解算的数据
    Matx31f rvec, tvec;
    Matx31f rvec_temp, tvec_temp;
    solvePnP(points_3d, points_2d, camera_param.cameraMatrix, camera_param.distCoeff, rvec_temp, tvec_temp, false, SOLVEPNP_SQPNP);
    tvec = tvec_temp;
    rvec = rvec_temp;

    PoseNode original_pnp_data;
    original_pnp_data.tvec(tvec);
    original_pnp_data.rvec(rvec);

    PoseNode corrected_pnp_data;
    if (correctPnpData(group, original_pnp_data, corrected_pnp_data) == false)
    {
        return false;
    }
    pnp_data = corrected_pnp_data;
    return true;
}

bool RuneDetector::getPnpData(PoseNode &camera_pnp_data,
                              const FeatureNode_ptr &group,
                              const std::vector<RuneFeatureComboConst> &matched_features) const
{
    if (group == nullptr)
    {
        VC_ERROR_INFO("The group is nullptr");
    }
    if (matched_features.empty())
    {
        return false;
    }
    // 获取参考PnP数据
    vector<Point2f> points_2d{};
    vector<Point3f> points_3d{};
    vector<float> point_weights{};
    if (getPnpPoints(group, matched_features, points_2d, points_3d, point_weights) == false) // 获取PNP解算的点集
    {
        return false;
    }

    if (points_2d.size() < 4)
    {
        return false;
    }

    if (getCameraPnpData(group, points_2d, points_3d, point_weights, camera_pnp_data) == false) // 获取神符在相机坐标系下的PNP数据
    {
        return false;
    }

    return true;
}