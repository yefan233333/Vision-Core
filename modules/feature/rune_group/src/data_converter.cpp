#include "vc/feature/rune_group.h"

/**
 * @brief 获取转化器
 */
std::shared_ptr<DataConverter> DataConverter::make_converter()
{
    return std::make_shared<DataConverter>();
}

/**
 * @brief 将可滤波形式的数据转化为平移向量和旋转向量
 *
 * @param[in] filter_pos [x y z | yaw pitch roll]
 *
 * @return std::tuple<cv::Matx31f, cv::Matx31f> 平移向量和旋转向量
 */
std::tuple<cv::Matx31f, cv::Matx31f> DataConverter::toTvecAndRvec(const cv::Matx61f &filter_pos)
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
cv::Matx61f DataConverter::toFilterForm(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock, bool is_reset)
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
cv::Matx31f DataConverter::euler2Rvec(float yaw, float pitch, float roll)
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
std::tuple<float, float, float> DataConverter::rvec2Euler(const cv::Vec3f &rvec)
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

cv::Matx61f DataConverter::cvtPos(const cv::Vec3f &tvec, const cv::Vec3f &rvec, bool &is_gimbal_lock)
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

    return cv::Matx61f{tvec(0), tvec(1), tvec(2), yaw, pitch, roll};
}