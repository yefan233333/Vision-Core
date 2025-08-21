#pragma once

#include "vc/feature/tracking_feature_node.h"

class DataConverter; //!< 前向声明数据转换器
class RuneFilterStrategy; //!< 前向声明神符滤波器

//! 神符序列组
class RuneGroup : public TrackingFeatureNode
{

    using Ptr = std::shared_ptr<RuneGroup>;

    DEFINE_PROPERTY_WITH_INIT(RawDatas, public, protected, (std::deque<double>), {});    //!< 原始角度数据
    DEFINE_PROPERTY_WITH_INIT(AngleSpeeds, public, protected, (std::deque<double>), {}); //!< 角速度数据（顺负逆正）
    DEFINE_PROPERTY(PredictFunc, public, public, (std::function<double(int64_t)>));      //!< 角度变化函数
    DEFINE_PROPERTY(RecommendedTicks, public, public, (std::vector<int64_t>));           //!< 推荐击打的时间戳 (由小到大)
    DEFINE_PROPERTY(CamToGyro, public, protected, (PoseNode));                           //!< 相机坐标系到陀螺仪坐标系的变换位姿数据
    DEFINE_PROPERTY(GyroData, public, public, (GyroData));                               //!< 陀螺仪数据
    DEFINE_PROPERTY(LastUpdateTick, public, protected, (int64_t));

    std::shared_ptr<RuneFilterStrategy> _filter = nullptr; //!< 滤波策略,用于对陀螺仪坐标系下的PNP数据进行滤波

public:
    RuneGroup() = default;

    //! 构建 RuneGroup
    static inline std::shared_ptr<RuneGroup> make_group() { return std::make_shared<RuneGroup>(); }

    /**
     * @brief 动态类型转换
     *
     * @param[in] p_group group_ptr 抽象指针
     * @return 派生对象指针
     */
    static inline std::shared_ptr<RuneGroup> cast(FeatureNode_ptr p_group)
    {
        return std::dynamic_pointer_cast<RuneGroup>(p_group);
    }

    /**
     * @brief 更新神符序列组
     *
     * @param[in] rune_to_cam_raw 神符序列组在相机坐标系下的PNP数据（原始数据）
     *  @param[in] gyro_data 陀螺仪数据
     * @param[in] tick 时间戳
     */
    bool update(const PoseNode &rune_to_cam_raw, const GyroData &gyro_data, int64_t tick);

    /**
     * @brief 利用滤波器获取最新帧的相机坐标系PNP数据
     *
     * @return ResultPnP<float> 相机坐标系PNP数据
     */
    bool getCamPnpDataFromFilter(PoseNode &runeGroup_to_cam) const;

    /**
     * @brief 利用过去的数据获取最新帧的相机坐标系PNP数据
     *
     * @param[out] runeGroup_to_cam 相机坐标系PNP数据
     *
     * @note 依赖 预测方程和 getCamPnpDataFromFilter 函数
     */
    bool getCamPnpDataFromPast(PoseNode &runeGroup_to_cam) const;

    /**
     * @brief 获取上一帧的特征信息
     */
    std::vector<std::tuple<FeatureNode_cptr, FeatureNode_cptr, FeatureNode_cptr>> getLastFrameFeatures() const;

    /**
     * @brief 可见性处理
     *
     * @param[in] is_observation 是否为有效观测数据
     *
     * @return 是否处理失败（处理失败舍弃该序列组）
     */
    bool visibilityProcess(bool is_observation);

    /**
     * @brief 获取当前神符的转角
     *
     * @param[out] angle 当前神符的转角
     *
     * @return 是否获取成功
     */
    bool getCurrentRotateAngle(float &angle) const;

    /**
     * @brief 神符序列组同步
     *
     * @param[in] gyro_data 最新陀螺仪数据
     * @param[in] tick 最新时间戳sync
     *
     * @note
     * - 计算整个序列组当前帧的原始数据 `raw_data`，并同步至 `datas` 中,
     * - 将神符序列组的角度信息同步给所有的追踪器
     */
    void sync(const GyroData &gyro_data, int64_t tick);

    /**
     * @brief 神符激活信息结构体
     */
    struct ActivationInfo
    {
        //! 当前已激活的靶心个数
        int active_target_count = 0;
        //! 是存在前一次靶心切换
        bool has_last_switch = false;
        //! 前一次靶心切换的时间戳
        int64_t last_switch_tick = 0;
    };

    /**
     * @brief 获取神符最新的激活信息
     *
     * @return 神符激活信息
     */
    ActivationInfo getActivationInfo() const;

    /**
     * @brief 获取相机坐标系到陀螺仪坐标系的PNP数据
     *
     * @param[in] gyro_data 陀螺仪数据
     */
    static PoseNode calcCamToGyroPnpData(const GyroData &gyro_data);

private:
    //! 转化器映射
    std::unordered_map<std::string, std::shared_ptr<DataConverter>> __data_converters;

    /**
     * @brief 更新PNP数据
     *
     * @param[in] rune_to_gyro 陀螺仪坐标系下的PNP数据
     * @param[in] gyro_data 陀螺仪数据
     *
     * @note 通过 `rune_to_gyro` 和 `gyro_data` 更新 `_gyro_pnp_data`、`_joint_pnp_data` 和 `_cam_pnp_data`
     */
    void updatePnpData(const PoseNode &rune_to_gyro, const GyroData &gyro_data);

    /**
     * @brief 重置部分属性，清除上一帧的数据
     */
    void clearPreviousData();

    /**
     * @brief 修正观测到的神符的roll角
     *
     * @param[in] raw_cam_pnp_data 原始相机坐标系PNP数据
     * @param[out] corrected_cam_pnp_data 修正后的相机坐标系PNP数据
     *
     * @return 是否进行了修正
     */
    bool correctRoll(const PoseNode &raw_cam_pnp_data, PoseNode &corrected_cam_pnp_data);

    PoseNode _correntRoll_last_cam_pnp_data{}; //!< 上一帧的相机坐标系PNP数据
    bool _correntRoll_last_cam_pnp_data_flag = false;  //!< 上一帧的相机坐标系PNP数据是否有效

    /**
     * @brief 检验位姿是否过于极端
     *
     * @param[in] rune_to_cam 神符在相机坐标系下的位姿数据
     * @param[in] rune_to_gyro 神符在陀螺仪坐标系下的位姿数据
     * @param[in] gyro_data 陀螺仪数据
     *
     * @return true 检测通过
     */
    static bool checkExtremePose(const PoseNode &rune_to_cam, const PoseNode &rune_to_gyro, const GyroData &gyro_data);

    /**
     * @brief 判断当前帧是否发生了靶心切换
     */
    bool isTargetCenterChanged() const;

    /**
     * @brief 更新神符激活信息
     */
    void updateActivationInfo();

    /**
     * @brief 更新神符中心的估计信息
     *
     * @param[in] rune_to_cam 神符在相机坐标系下的位姿数据
     * @param[in] tick 时间戳
     */
    bool updateCenterEstimation();
};

using RuneGroup_ptr = std::shared_ptr<RuneGroup>;

/**
 * @brief 数据转化器
 */
class DataConverter
{
public:
    DataConverter() = default;

    /**
     * @brief 获取转化器
     */
    static std::shared_ptr<DataConverter> make_converter();

    /**
     * @brief 将可滤波形式的数据转化为平移向量和旋转向量
     *
     * @param[in] filter_pos [x y z | yaw pitch roll]
     *
     * @return std::tuple<cv::Matx31f, cv::Matx31f> 平移向量和旋转向量
     */
    static std::tuple<cv::Matx31f, cv::Matx31f> toTvecAndRvec(const cv::Matx61f &filter_pos)
    {
        // 将欧拉角转化为旋转矩阵 YXZ

        // 计算旋转矩阵
        // const float &pitch = deg2rad(filter_pos(3));
        // const float &yaw = deg2rad(filter_pos(4));
        // const float &roll = deg2rad(filter_pos(5));

        const float &yaw = deg2rad(filter_pos(3));
        const float &pitch = deg2rad(filter_pos(4));
        const float &roll = deg2rad(filter_pos(5));

        auto rvec = euler2Rvec(yaw, pitch, roll);
        // 计算平移向量
        cv::Matx31f tvec{filter_pos(0), filter_pos(1), filter_pos(2)};
        return std::make_tuple(tvec, rvec);
    }

    /**
     * @brief 将平移向量和旋转向量转化为可滤波形式
     *
     * @param[in] tvec 平移向量
     * @param[in] rvec 旋转向量
     * @param[out] is_gimbal_lock 是否万向节锁死
     * @param[in] is_reset 是否重置连续性处理
     *
     *
     * @return cv::Matx61f [x y z | yaw pitch roll]
     */
    cv::Matx61f toFilterForm(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock, bool is_reset = false)
    {

        float &last_yaw = _last_yaw;
        float &last_roll = _last_roll;

        // 将旋转向量转化为欧拉角, 顺序为 YXZ
        auto [yaw, pitch, roll] = rvec2Euler(rvec);

        // 连续性处理
        if (is_reset)
        {
            last_yaw = yaw;
            last_roll = roll;
        }
        else
        {
            // 连续性处理
            while (yaw - last_yaw > CV_PI)
            {
                yaw -= 2 * CV_PI;
            }
            while (yaw - last_yaw < -CV_PI)
            {
                yaw += 2 * CV_PI;
            }
            last_yaw = yaw;

            // 连续性处理
            while (roll - last_roll > CV_PI)
            {
                roll -= 2 * CV_PI;
            }
            while (roll - last_roll < -CV_PI)
            {
                roll += 2 * CV_PI;
            }
            last_roll = roll;
        }

        // 弧度转角度
        pitch = rad2deg(pitch);
        yaw = rad2deg(yaw);
        roll = rad2deg(roll);

        // 万向锁死判断
        if (pitch > 85 || pitch < -85)
        {
            is_gimbal_lock = true;
        }
        else
        {
            is_gimbal_lock = false;
        }

        // return cv::Matx61f{tvec(0), tvec(1), tvec(2), pitch, yaw, roll};
        return cv::Matx61f{tvec(0), tvec(1), tvec(2), yaw, pitch, roll};
    }
    /**
     * @brief 将欧拉角转换为旋转向量
     *
     * @param[in] yaw 欧拉角 yaw
     * @param[in] pitch 欧拉角 pitch
     * @param[in] roll 欧拉角 roll
     */
    static cv::Matx31f euler2Rvec(float yaw, float pitch, float roll)
    {
        // 将欧拉角转化为旋转矩阵, 顺序为 YXZ
        cv::Matx33f rmat = euler2Mat(yaw, EulerAxis::Y) * euler2Mat(pitch, EulerAxis::X) * euler2Mat(roll, EulerAxis::Z);

        // 将旋转矩阵转化为旋转向量
        cv::Matx31f rvec;
        cv::Rodrigues(rmat, rvec);

        return rvec;
    }
    /**
     * @brief 将旋转向量转换为欧拉角
     *
     * @param[in] rvec 旋转向量
     */
    static std::tuple<float, float, float> rvec2Euler(const cv::Vec3f &rvec)
    {
        // 将旋转向量转化为旋转矩阵
        cv::Matx33f rmat;
        cv::Rodrigues(rvec, rmat);

        // 将旋转矩阵转化为欧拉角, 顺序为 YXZ
        const float &r13 = rmat(0, 2);
        const float &r21 = rmat(1, 0), &r22 = rmat(1, 1), &r23 = rmat(1, 2);
        const float &r32 = rmat(2, 1), &r33 = rmat(2, 2);

        // 计算 pitch
        float sin_pitch = -r23;
        sin_pitch = std::clamp(sin_pitch, -1.0f, 1.0f);
        float pitch = asin(sin_pitch);

        // 计算 yaw
        float yaw = std::atan2(r13, r33);

        // 计算 roll
        float roll = std::atan2(r21, r22);

        return {yaw, pitch, roll};
    }

    static cv::Matx61f cvtPos(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock)
    {
        // 将旋转向量转化为欧拉角, 顺序为 YXZ
        auto [yaw, pitch, roll] = rvec2Euler(rvec);

        // 弧度转角度
        pitch = rad2deg(pitch);
        yaw = rad2deg(yaw);
        roll = rad2deg(roll);

        // 万向锁死判断
        if (pitch > 85 || pitch < -85)
        {
            is_gimbal_lock = true;
        }
        else
        {
            is_gimbal_lock = false;
        }

        // return cv::Matx61f{tvec(0), tvec(1), tvec(2), pitch, yaw, roll};
        return cv::Matx61f{tvec(0), tvec(1), tvec(2), yaw, pitch, roll};
    }

private:
private:
    float _last_yaw = 0;
    float _last_roll = 0;
};
using DataConverter_ptr = std::shared_ptr<DataConverter>;