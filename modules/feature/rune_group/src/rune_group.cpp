#include "vc/feature/rune_group.h"
#include "vc/feature/rune_group_param.h"
#include "vc/feature/rune_group_filter.h"
#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_target.h"
#include "vc/feature/rune_center.h"
#include "vc/feature/rune_fan.h"
#include "vc/feature/rune_filter_fusion.h"
#include "vc/camera/camera_param.h"

using namespace std;
using namespace cv;

/**
 * @brief 异常值检测
 */
inline bool errorValueProcess(const Matx61f &pos)
{
    // 检测是否为无效值
    if (isnan(pos(0)) || isnan(pos(1)) || isnan(pos(2)) || isnan(pos(3)) || isnan(pos(4)) || isnan(pos(5)))
    {
        return false;
    }

    // 检测是否为无穷大
    if (isinf(pos(0)) || isinf(pos(1)) || isinf(pos(2)) || isinf(pos(3)) || isinf(pos(4)) || isinf(pos(5)))
    {
        return false;
    }

    // 检测是否为过大值
    if (abs(pos(0)) > 1e5 || abs(pos(1)) > 1e5 || abs(pos(2)) > 1e5 || abs(pos(3)) > 1e5 || abs(pos(4)) > 1e5 || abs(pos(5)) > 1e5)
    {
        return false;
    }

    return true;
}

/**
 * @brief 检验两帧之间的位姿是否差距过大
 *
 * @param pos1 第一帧位姿
 * @param pos2 第二帧位姿
 *
 * @return true 差距正常
 */
inline bool checkPoseDiff(const Matx61f &pos1, const Matx61f &pos2)
{
    float diff_x = pos1(0) - pos2(0);
    float diff_y = pos1(1) - pos2(1);
    float diff_z = pos1(2) - pos2(2);
    float diff_yaw = pos1(3) - pos2(3);
    float diff_pitch = pos1(4) - pos2(4);
    float diff_roll = pos1(5) - pos2(5);

    if (abs(diff_x) > rune_group_param.MAX_X_DEVIATION)
    {
        return false;
    }

    if (abs(diff_y) > rune_group_param.MAX_Y_DEVIATION)
    {
        return false;
    }

    if (abs(diff_z) > rune_group_param.MAX_Z_DEVIATION)
    {
        return false;
    }

    if (abs(diff_yaw) > rune_group_param.MAX_YAW_DEVIATION)
    {
        return false;
    }

    if (abs(diff_pitch) > rune_group_param.MAX_PITCH_DEVIATION)
    {
        return false;
    }

    if (abs(diff_roll) > rune_group_param.MAX_ROLL_DEVIATION)
    {
        return false;
    }

    return true;
}

/**
 * @brief 检验位姿数据是否无效
 *
 * @param[in] pose 位姿数据
 *
 * @return true 检测通过
 */
inline bool checkPoseValid(const PoseNode &pose)
{
    auto tvec = pose.tvec();
    auto rvec = pose.rvec();
    if (isnan(tvec(0)) || isnan(tvec(1)) || isnan(tvec(2)) ||
        isnan(rvec(0)) || isnan(rvec(1)) || isnan(rvec(2)))
    {
        return false;
    }
    if (isinf(tvec(0)) || isinf(tvec(1)) || isinf(tvec(2)) ||
        isinf(rvec(0)) || isinf(rvec(1)) || isinf(rvec(2)))
    {
        return false;
    }
    return true;
}

/**
 * @brief 通过陀螺仪坐标系下的目标位置信息，计算由坐标系原点指向目标的 yaw ptich 角
 *
 * @param[in] target_pos 陀螺仪坐标系下的目标位置
 *
 * @return std::pair [yaw, pitch]
 *
 * @note yaw 由左向右增大；pitch 由上向下增大
 */
inline Point2d calculateTargetAngle(const Point3d &target_pos)
{
    // 1. 计算 yaw 和 pitch
    float yaw = atan2(target_pos.x, target_pos.z);
    float s = sqrt(target_pos.x * target_pos.x + target_pos.z * target_pos.z);
    float pitch = atan2(target_pos.y, s);

    // 2. 转换为角度
    yaw = rad2deg(yaw);
    pitch = rad2deg(pitch);

    return {yaw, pitch};
}

/**
 * @brief 检验位姿是否过于极端
 *
 * @param[in] rune_to_cam 神符在相机坐标系下的位姿数据
 * @param[in] rune_to_gyro 神符在陀螺仪坐标系下的位姿数据
 * @param[in] gyro_data 陀螺仪数据
 *
 * @return true 检测通过
 */
bool RuneGroup::checkExtremePose(const PoseNode &rune_to_cam, const PoseNode &rune_to_gyro, const GyroData &gyro_data)
{
    if (!rune_group_param.ENABLE_EXTREME_VALUE_FILTER)
    {
        return true;
    }

    if (!checkPoseValid(rune_to_cam) || !checkPoseValid(rune_to_gyro))
    {
        return false;
    }

    //----------------------【相机坐标系判断】---------------------
    // 1. 判断距离
    float dis_cam = cv::norm(rune_to_cam.tvec());
    if (dis_cam > rune_group_param.MAX_DISTANCE || dis_cam < rune_group_param.MIN_DISTANCE)
    {
        return false;
    }
    // 2. 判断位姿朝向是否触发万向锁
    bool is_gimbal_lock = false;
    Matx61f pos_cam = DataConverter::cvtPos(rune_to_cam.tvec(), rune_to_cam.rvec(), is_gimbal_lock);
    if (is_gimbal_lock)
    {
        return false;
    }
    // 3. 判断yaw角朝向是否极端
    float yaw_cam = pos_cam(3);
    if (yaw_cam > rune_group_param.MAX_YAW_DEVIATION_ANGLE || yaw_cam < -rune_group_param.MAX_YAW_DEVIATION_ANGLE)
    {
        return false;
    }
    // 4. 判断神符的目标转角是否过于极端
    auto [rotate_yaw, rotate_pitch] = calculateTargetAngle(rune_to_cam.tvec());
    if (rotate_yaw > rune_group_param.MAX_ROTATE_YAW || rotate_yaw < -rune_group_param.MAX_ROTATE_YAW)
    {
        return false;
    }
    if (rotate_pitch > rune_group_param.MAX_ROTATE_PITCH || rotate_pitch < -rune_group_param.MAX_ROTATE_PITCH)
    {
        return false;
    }

    //------------------------【陀螺仪坐标系判断】---------------------
    // 1. 判断位姿朝向是否触发了万向锁
    Matx61f pos_gyro = DataConverter::cvtPos(rune_to_gyro.tvec(), rune_to_gyro.rvec(), is_gimbal_lock);
    if (is_gimbal_lock)
    {
        return false;
    }
    // 2. 判断神符的pitch角朝向是否过于极端
    float pitch_gyro = pos_gyro(4);
    if (pitch_gyro > rune_group_param.MAX_PITCH_DEVIATION_ANGLE || pitch_gyro < -rune_group_param.MAX_PITCH_DEVIATION_ANGLE)
    {
        return false;
    }

    return true;
}

bool RuneGroup::update(const PoseNode &rune_to_cam_raw, const GyroData &gyro_data, int64_t tick)
{

    // 清除上一帧的数据
    clearPreviousData();
    // 滤波器判空
    if (_filter == nullptr)
    {
        _filter = RuneFilterFusion::make_filter();
    }
    // 是否在同一帧内发生了重复更新
    bool is_same_frame = false;
    if (!isSetLastUpdateTick())
    {
        is_same_frame = false;
    }
    else
    {
        is_same_frame = (tick == getLastUpdateTick());
    }
    setLastUpdateTick(tick); // 更新最后一次更新时间戳

    // 矫正观测位姿数据 的 roll 角
    PoseNode corrected_rune_to_cam_raw{};
    correctRoll(rune_to_cam_raw, corrected_rune_to_cam_raw);

    // 计算相机坐标系到转轴坐标系的PNP数据
    // auto cam_to_gyro = calcCamToGyroPnpData(gyro_data);
    auto cam_to_gyro = this->isSetCamToGyro() ? getCamToGyro() : calcCamToGyroPnpData(gyro_data);
    PoseNode rune_to_gyro_raw = corrected_rune_to_cam_raw + cam_to_gyro;

    // 是否处于丢帧状态
    bool is_vanish = false;
    if (this->getVanishNum() <= 0)
    {
        is_vanish = false;
        this->setLastValidPoseGyro(rune_to_gyro_raw); // 更新最后一次有效观测位姿位置
    }
    else
    {
        is_vanish = true;
    }

    bool _is_gimbal_lock = false;
    // 转化为滤波数据
    auto &data_converters = this->__data_converters;
    if (data_converters.find("4") == data_converters.end())
    {
        data_converters["4"] = DataConverter::make_converter();
    }
    Matx61f raw_pos = data_converters["4"]->toFilterForm(rune_to_gyro_raw.tvec(), rune_to_gyro_raw.rvec(), _is_gimbal_lock);
    // Matx61f raw_pos = cvtFilterPos(rune_to_gyro_raw.tvec(), rune_to_gyro_raw.rvec(), _is_gimbal_lock); // 转化为滤波数据

    // 检验两帧之间的位姿是否差距过大
    if (_filter->isValid())
    {
        auto predict_pnp_data = _filter->getPredict();
        if (!checkPoseDiff(predict_pnp_data, raw_pos))
        {
            return false;
        }
    }

    // 极端位姿检验
    if (!checkExtremePose(rune_to_cam_raw, rune_to_gyro_raw, gyro_data))
    {
        return false;
    }

    // 异常值检测
    if (!errorValueProcess(raw_pos))
    {
        return false;
    }
    // 进行滤波处理
    RuneFilterStrategy::FilterInput filter_input;
    filter_input.raw_pos = raw_pos;
    filter_input.tick = tick;
    filter_input.cam_to_gyro = cam_to_gyro;
    filter_input.is_observation = !is_vanish; // 是否为有效观测数据
    // 执行滤波
    auto filter_output = _filter->filter(filter_input);
    // 获取滤波后的位置
    Matx61f filter_pos = filter_output.filtered_pos;
    // 转化为平移向量和旋转向量
    auto [tvec, rvec] = DataConverter::toTvecAndRvec(filter_pos);
    // 更新PNP数据
    PoseNode rune_to_gyro(rvec, tvec);
    this->updatePnpData(rune_to_gyro, gyro_data); // 更新PNP数据
    if (!is_vanish)
    {
        this->updateCenterEstimation(); // 更新神符中心估计信息
    }
    // // 更新激活信息数据
    // this->updateActivationInfo();
    return true;
}

bool RuneGroup::getCamPnpDataFromFilter(PoseNode &runeGroup_to_cam) const
{
    if (_filter == nullptr)
    {
        return false;
    }

    // 判断滤波器是否有效
    if (!_filter->isValid())
    {
        return false;
    }

    // 获取滤波后的数据
    Matx61f filter_pos = _filter->getPredict();
    // 转化为平移向量和旋转向量
    auto [tvec, rvec] = DataConverter::toTvecAndRvec(filter_pos);
    // auto [tvec, rvec] = cvtTvecRvec(filter_pos);
    // 更新PNP数据
    PoseNode rune_to_gyro(rvec, tvec);
    // 获取陀螺仪坐标系到相机坐标系的PNP数据
    runeGroup_to_cam = rune_to_gyro + calcCamToGyroPnpData(getPoseCache().getGyroData()).inv();
    return true;
}

bool RuneGroup::getCamPnpDataFromPast(PoseNode &runeGroup_to_cam) const
{
    // 1. 获取预测器的预测结果
    PoseNode pos_to_cam_from_filter{};
    if (!getCamPnpDataFromFilter(pos_to_cam_from_filter))
    {
        return false;
    }

    float roll_from_predict = 0;
    // 2. 获取角度变化函数
    if (!isSetPredictFunc() || !getPredictFunc())
    {
        runeGroup_to_cam = pos_to_cam_from_filter;
        return true;
    }
    else
    {
        // 3. 计算不同帧之间的平均tick_delta
        auto &tick_deque = getHistoryTicks();
        if (tick_deque.size() < 2)
        {
            runeGroup_to_cam = pos_to_cam_from_filter;
            return true;
        }
        // int64_t tick_delta = (tick_deque.front() - tick_deque.back()) / (tick_deque.size() - 1);
        int64_t predict_tick = tick_deque.front();
        // int64_t predict_tick = tick_deque.front();
        // int64_t predict_tick = tick_deque.front() - tick_delta;
        // 4. 计算预测的角度
        float predict_angle = getPredictFunc()(predict_tick);
        if (std::isnan(predict_angle) || std::isinf(predict_angle))
        {
            runeGroup_to_cam = pos_to_cam_from_filter;
            return true;
        }
        roll_from_predict = predict_angle;
    }

    // 5. 将滤波器数据转化为陀螺仪坐标系下
    auto cam_to_gyro = calcCamToGyroPnpData(getPoseCache().getGyroData());
    auto pos_to_gyro_from_filter = pos_to_cam_from_filter + cam_to_gyro;

    // 6. 将滤波器数据转化为 xyz 和 yaw pitch roll
    auto tvec = pos_to_gyro_from_filter.tvec();
    auto rvec = pos_to_gyro_from_filter.rvec();
    bool is_gimbal_lock = false;
    auto pos_ = DataConverter::cvtPos(tvec, rvec, is_gimbal_lock);
    // auto pos_ = RuneGroup::cvtPos(tvec, rvec, is_gimbal_lock);

    // 7. 使用预测的 roll 角替换滤波器数据中的 roll 角
    auto pos_to_gyro_from_predict = pos_;
    pos_to_gyro_from_predict(5) = roll_from_predict;

    // 8. 将预测的位姿数据转化为陀螺仪坐标系下
    auto [tvec_to_gyro_from_predict, rvec_to_gyro_from_predict] = DataConverter::toTvecAndRvec(pos_to_gyro_from_predict);
    // auto [tvec_to_gyro_from_predict, rvec_to_gyro_from_predict] = cvtTvecRvec(pos_to_gyro_from_predict);

    // 7. 更新PNP数据
    PoseNode predict_pos_to_gyro(rvec_to_gyro_from_predict, tvec_to_gyro_from_predict);

    // 8. 转化为相机坐标系下
    auto predict_pos_to_cam = predict_pos_to_gyro + cam_to_gyro.inv();

    // 9. 更新PNP数据
    runeGroup_to_cam = predict_pos_to_cam;

    return true;
}

std::vector<RuneFeatureComboConst> RuneGroup::getLastFrameFeatures() const
{
    std::vector<RuneFeatureComboConst> features{};
    auto &_trackers = this->getTrackers();
    for (auto &tracker : _trackers)
    {
        // 获取最新帧
        auto combo = TrackingFeatureNode::cast(tracker)->getHistoryNodes().front();
        if (combo == nullptr)
            continue;
        auto rune = RuneCombo::cast(combo);
        if (rune->getRuneType() == RuneType::UNKNOWN || rune->getRuneType() == RuneType::UNSTRUCK)
            continue;

        // 获取特征
        auto RuneT_ptr = RuneTarget::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_TARGET));
        auto RuneC_ptr = RuneCenter::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_CENTER));
        auto RuneF_ptr = RuneFan::cast(rune->getChildFeatures().at(ChildFeatureType::RUNE_FAN));
        features.emplace_back(std::make_tuple(RuneT_ptr, RuneC_ptr, RuneF_ptr));
    }
    return features;
}

bool RuneGroup::visibilityProcess(bool is_observation)
{
    auto &_vanish_num = this->getVanishNum();
    if (!is_observation)
    {
        if (_vanish_num > rune_group_param.MAX_VANISH_NUMBER)
        {
            return false;
        }
        _vanish_num++;
    }
    else
    {
        _vanish_num = 0;
        return true;
    }
    return true;
}

void RuneGroup::clearPreviousData()
{
    // 清除上一帧的PNP解算数据，重置标志位
    clearCamToGyro();
}

/**
 * @brief 判断滤波器中的数据是否有效
 */
inline bool isFilterDataValid(const Matx61f &pos)
{
    // 检测是否为无效值
    if (isnan(pos(0)) || isnan(pos(1)) || isnan(pos(2)) || isnan(pos(3)) || isnan(pos(4)) || isnan(pos(5)))
    {
        return false;
    }

    // 检测是否为无穷大
    if (isinf(pos(0)) || isinf(pos(1)) || isinf(pos(2)) || isinf(pos(3)) || isinf(pos(4)) || isinf(pos(5)))
    {
        return false;
    }

    // 检测是否为过大值
    if (abs(pos(0)) > 1e5 || abs(pos(1)) > 1e5 || abs(pos(2)) > 1e5 || abs(pos(3)) > 1e5 || abs(pos(4)) > 1e5 || abs(pos(5)) > 1e5)
    {
        return false;
    }

    return true;
}

bool RuneGroup::getCurrentRotateAngle(float &angle) const
{
    // 获取滤波器的最新数据
    Matx61f pos = _filter->getLatestValue();

    float roll_angle = 0;
    if (isFilterDataValid(pos))
    {
        roll_angle = pos(5);
    }
    else
    {
        auto &raw_datas = getRawDatas();
        if (raw_datas.size() >= 2)
        {
            roll_angle = (raw_datas[0] - raw_datas[1]) + raw_datas[0];
        }
        else if (raw_datas.size() == 1)
        {
            roll_angle = raw_datas.front();
        }
        else
        {
            roll_angle = 0;
        }
    }
    angle = roll_angle;
    return true;
}

void RuneGroup::sync(const GyroData &, int64_t)
{
    // 从滤波器中获取roll角作为当前的角度
    float roll;
    getCurrentRotateAngle(roll);
    auto &raw_datas = getRawDatas();

    raw_datas.push_front(roll);
    if (raw_datas.size() > rune_group_param.RAW_DATAS_SIZE)
        raw_datas.pop_back();

    // 更新时间戳队列
    auto &tick_deque = getHistoryTicks();
    tick_deque.push_front(getTick());
    if (tick_deque.size() > rune_group_param.RAW_DATAS_SIZE)
        tick_deque.pop_back();
}

/**
 * @brief 通过神符类型数据判断是否出现识别异常，并返回出现异常的帧数下标
 * @param[in] trackers
 * @param[in] range
 * @return 返回出现异常的帧数下标集合
 *
 * @note 用于判断的范围为前 N 帧数据，N 由 range 参数指定
 */
inline unordered_set<size_t> getAbnormalFrames(const vector<TrackingFeatureNode_cptr> &trackers, size_t range)
{
    unordered_set<size_t> abnormal_frames{};
    // 获取有效的下标范围
    vector<size_t> lens(trackers.size(), 0);
    for (size_t i = 0; i < trackers.size(); ++i)
        lens[i] = TrackingFeatureNode::cast(trackers[i])->getHistoryNodes().size();
    auto min_len = *min_element(lens.begin(), lens.end());
    range = min(range, min_len);

    // 获取所有神符组合体的类型信息
    deque<vector<RuneType>> rune_types_array(range, vector<RuneType>(trackers.size(), RuneType::UNKNOWN));
    for (size_t frame_id = 0; frame_id < range; ++frame_id)
    {
        for (size_t tracker_id = 0; tracker_id < trackers.size(); ++tracker_id)
        {
            const auto &combo = RuneCombo::cast(TrackingFeatureNode::cast(trackers[tracker_id])->getHistoryNodes()[frame_id]);
            if (combo == nullptr || combo->getRuneType() == RuneType::UNKNOWN)
            {
                rune_types_array[frame_id][tracker_id] = RuneType::UNKNOWN;
            }
            else
            {
                rune_types_array[frame_id][tracker_id] = combo->getRuneType();
            }
        }
    }
    // 如果某一帧帧出现了多个 PENDING_STRUCK 类型的神符、或者没有 PENDING_STRUCK，则跳过该帧
    for (size_t frame_id = 0; frame_id < range; ++frame_id)
    {
        int pending_struck_count = 0;
        for (const auto &rune_type : rune_types_array[frame_id])
        {
            if (rune_type == RuneType::PENDING_STRUCK)
            {
                pending_struck_count++;
            }
        }
        if (pending_struck_count > 1 || pending_struck_count == 0)
        {
            abnormal_frames.insert(frame_id);
        }
    }

    // 如果前后相邻的两帧，新旧帧都存在 STRUCK 类型的神符，但旧帧 STRUCK 类型的神符数量大于新帧，则跳过该帧（因为已击打神符不可能越打越少）
    for (size_t frame_id = 0; frame_id < range - 1; ++frame_id)
    {
        if (abnormal_frames.count(frame_id) || abnormal_frames.count(frame_id + 1))
        {
            continue; // 跳过已标记的帧
        }

        int struck_count_old = 0;
        int struck_count_new = 0;
        for (const auto &rune_type : rune_types_array[frame_id + 1])
        {
            if (rune_type == RuneType::STRUCK)
            {
                struck_count_old++;
            }
        }
        for (const auto &rune_type : rune_types_array[frame_id])
        {
            if (rune_type == RuneType::STRUCK)
            {
                struck_count_new++;
            }
        }
        if (struck_count_old > 0 && struck_count_new > 0 && struck_count_new < struck_count_old)
        {
            abnormal_frames.insert(frame_id);
            abnormal_frames.insert(frame_id + 1);
        }
    }

    return abnormal_frames;
}

bool RuneGroup::isTargetCenterChanged() const
{
    vector<TrackingFeatureNode_cptr> trackers{};
    for (const auto &tracker : this->getTrackers())
    {
        trackers.push_back(TrackingFeatureNode::cast(tracker));
    }
    if (trackers.empty() || trackers.size() != 5)
    {
        return false;
    }

    // 保证每个神符组合体都至少有三个以上的历史数据
    for (const auto &tracker : trackers)
    {
        if (tracker->getHistoryNodes().size() < 3)
        {
            return false;
        }
    }

    // 通过历史前 N 帧数据作为判断依据
    int history_size = 10;
    vector<int> combo_len_array{};
    for (const auto &tracker : trackers)
    {
        combo_len_array.push_back(static_cast<int>(tracker->getHistoryNodes().size()));
    }
    auto min_combo_len = *min_element(combo_len_array.begin(), combo_len_array.end());
    history_size = min(history_size, min_combo_len);

    // 获取需要跳过的帧数下标
    auto abnormal_frames = getAbnormalFrames(trackers, history_size);

    // 获取各个追踪器在有效帧数中，被标记为 PENDING_STRUCK 的次数
    vector<int> pending_struck_count(trackers.size(), 0);
    for (size_t frame_id = 0; frame_id < history_size; ++frame_id)
    {
        if (abnormal_frames.count(frame_id))
        {
            continue; // 跳过已标记的帧
        }
        for (size_t tracker_id = 0; tracker_id < trackers.size(); ++tracker_id)
        {
            const auto &combo = RuneCombo::cast(trackers[tracker_id]->getHistoryNodes()[frame_id]);
            if (combo == nullptr || combo->getRuneType() != RuneType::PENDING_STRUCK)
            {
                continue;
            }
            pending_struck_count[tracker_id]++;
        }
    }
    // 统计出现了 threshold_count 以上次数的 PENDING_STRUCK 的追踪器的个数
    constexpr int threshold_count = 2; // 设定阈值 X
    int count = 0;
    for (const auto &c : pending_struck_count)
    {
        if (c > threshold_count)
        {
            count++;
        }
    }

    if (count > 1)
    {
        return true;
    }

    return false;
}

/**
 * @brief 获取众数
 *
 * @param nums 输入的整数数组
 * @return 众数
 */
int findMode(const vector<int> &nums)
{
    unordered_map<int, int> count_map;
    int mode = nums.empty() ? -1 : nums[0];
    int max_count = 0;
    for (int num : nums)
    {
        count_map[num]++;
        if (count_map[num] > max_count)
        {
            max_count = count_map[num];
            mode = num;
        }
    }
    return mode;
}

struct CenterEstimationOutput
{
    //! 是否有效
    bool is_valid = false;
    //! 神符中心陀螺仪坐标系位置
    cv::Point3f pos_to_gyro;
    //! 获取该估计位置的时间戳
    int64_t tick;
};

bool RuneGroup::updateCenterEstimation()
{
    // 判空
    if (this->getTrackers().empty())
        return false;
    const auto &tracker = TrackingFeatureNode::cast(this->getTrackers().front());
    if (tracker->getHistoryNodes().empty())
        return false;
    const auto &combo = tracker->getHistoryNodes().front();
    const auto &center = RuneCenter::cast(combo->getChildFeatures().at(ChildFeatureType::RUNE_CENTER));
    // const auto &center_to_cam = center->getCamPnpData();
    const auto &center_to_cam = center->getPoseCache().getPoseNodes()[CoordFrame::CAMERA];
    const auto &cam_to_gyro = this->getCamToGyro();
    auto center_to_gyro = center_to_cam + cam_to_gyro;
    auto current_tick = this->getTick();
    CenterEstimationInfo center_estimation_info;
    center_estimation_info.is_valid = true;
    center_estimation_info.pos_to_gyro = center_to_gyro.tvec();
    center_estimation_info.tick = current_tick;
    this->setCenterEstimationInfo(center_estimation_info);

    return true;
}

std::vector<FeatureNode_ptr> RuneGroup::getTrackers()
{
    std::vector<FeatureNode_ptr> trackers;
    for (const auto &[id, tracker] : this->getChildFeatures())
        trackers.emplace_back(tracker);
    return trackers;
}

const std::vector<FeatureNode_cptr> RuneGroup::getTrackers() const
{
    std::vector<FeatureNode_cptr> trackers;
    for (const auto &[id, tracker] : this->getChildFeatures())
        trackers.emplace_back(tracker);
    return trackers;
}

template <class T>
std::string get_shared_id(const std::shared_ptr<T> &sp)
{
    if (!sp)
        return {};

    // 用控制块地址生成唯一 ID
    auto cb_addr = std::shared_ptr<void>(sp).get(); // 同一控制块不同别名指针也一致
    uintptr_t addr = reinterpret_cast<uintptr_t>(cb_addr);

    std::ostringstream oss;
    oss << "obj-" << std::hex << std::setw(sizeof(uintptr_t) * 2)
        << std::setfill('0') << addr;
    return oss.str();
}

void RuneGroup::setTrackers(const std::vector<FeatureNode_ptr> &trackers)
{
    this->getChildFeatures().clear();
    for (const auto &tracker : trackers)
    {
        if (tracker == nullptr)
            continue;
        auto id = get_shared_id(tracker);
        if (id.empty())
            continue;
        this->getChildFeatures()[id] = tracker;
    }
}

inline void drawCube(cv::Mat &img, const PoseNode &pnp_data, float x_len, float y_len, float z_len, cv::Scalar color)
{
    float half_x = x_len / 2;
    float half_y = y_len / 2;
    float half_z = z_len / 2;
    // 获取立方体的 3D 点
    vector<Point3f> cube_points_3d = {
        // z < 0 的正方形
        Point3f(-half_x, -half_y, -half_z),
        Point3f(half_x, -half_y, -half_z),
        Point3f(half_x, half_y, -half_z),
        Point3f(-half_x, half_y, -half_z),
        // z > 0 的正方形
        Point3f(-half_x, -half_y, half_z),
        Point3f(half_x, -half_y, half_z),
        Point3f(half_x, half_y, half_z),
        Point3f(-half_x, half_y, half_z)};

    // 获取立方体的 2D 点
    vector<Point2f> cube_points_2d{};
    projectPoints(cube_points_3d, pnp_data.rvec(), pnp_data.tvec(), camera_param.cameraMatrix, camera_param.distCoeff, cube_points_2d);

    // 线宽
    constexpr int line_width = 1;
    // 绘制立方体
    for (size_t i = 0; i < 4; i++)
    {
        auto p1 = cube_points_2d[i], p2 = cube_points_2d[(i + 1) % 4];
        line(img, p1, p2, color, line_width);

        p1 = cube_points_2d[i + 4], p2 = cube_points_2d[(i + 1) % 4 + 4];
        line(img, p1, p2, color, line_width);

        p1 = cube_points_2d[i], p2 = cube_points_2d[i + 4];
        line(img, p1, p2, color, line_width);
    }
}
void RuneGroup::drawFeature(cv::Mat &image, const DrawConfig_cptr &config) const
{
    if (image.empty())
        return;
    auto& pose_info = getPoseCache();
    do
    {
        //! 安全检查
        if (pose_info.getPoseNodes().find(CoordFrame::CAMERA) == pose_info.getPoseNodes().end())
            break;
        auto& rune_to_camera = pose_info.getPoseNodes().at(CoordFrame::CAMERA);
        drawCube(image, rune_to_camera, 2000, 2000, 500, cv::Scalar(0, 255, 0));

        
    }while(0);
}