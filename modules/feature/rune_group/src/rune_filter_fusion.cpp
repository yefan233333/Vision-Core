#include "vc/feature/rune_filter_fusion.h"
#include "vc/feature/rune_filter_ekf.h"

using namespace std;
using namespace cv;


std::shared_ptr<RuneFilterStrategy> RuneFilterFusion::make_filter()
{
    return std::make_shared<RuneFilterFusion>();
}

auto RuneFilterFusion::filter(FilterInput input) -> FilterOutput
{
    auto &raw_pos = input.raw_pos;
    auto &tick = input.tick;
    auto &cam_to_gyro = input.cam_to_gyro;
    auto &is_observation = input.is_observation;

    Matx61d result{};

    // 判断是否是第一次滤波
    if (!_is_init_filter)
    {
        initFilter(raw_pos, tick);
        _is_init_filter = true;
        result = raw_pos;
    }
    else
    {
        Matx61d filtered_pos{};
        unordered_map<RuneFilter_ptr, Matx61d> filter_results;

        // 用所有子滤波器进行滤波
        for (const auto &filter : _filters)
        {
            FilterInput filter_input;
            filter_input.raw_pos = raw_pos;
            filter_input.tick = tick;
            filter_input.cam_to_gyro = cam_to_gyro;
            filter_input.is_observation = is_observation; // 是否为有效观测数据

            auto filter_output = filter->filter(filter_input);
            filter_results[filter] = filter_output.filtered_pos;
        }
        // 分类所有子滤波器
        unordered_map<RuneFilterDataType, vector<RuneFilter_ptr>> filters_map;
        for (const auto &filter : _filters)
        {
            filters_map[filter->getDataType()].push_back(filter);
        }

        // 对所有子滤波器的滤波结果进行融合
        auto &filters_xyz = filters_map[RuneFilterDataType::XYZ];
        if (!filters_xyz.empty())
        {
            // 取第一个滤波器的 xyz 项作为融合结果
            auto &xyz_result_temp = filter_results[filters_xyz[0]];
            filtered_pos(0) = xyz_result_temp(0);
            filtered_pos(1) = xyz_result_temp(1);
            filtered_pos(2) = xyz_result_temp(2);
        }
        else
        {
            // 如果没有 XYZ 滤波器，直接取原数据作为融合结果
            filtered_pos(0) = raw_pos(0);
            filtered_pos(1) = raw_pos(1);
            filtered_pos(2) = raw_pos(2);
        }

        auto &filters_yp = filters_map[RuneFilterDataType::YAW_PITCH];
        if (!filters_yp.empty())
        {
            // 取第一个滤波器的 yaw pitch 项作为融合结果
            auto &yp_result_temp = filter_results[filters_yp[0]];
            filtered_pos(3) = yp_result_temp(3);
            filtered_pos(4) = yp_result_temp(4);
        }
        else
        {
            // 如果没有 Yaw Pitch 滤波器，直接取原数据作为融合结果
            filtered_pos(3) = raw_pos(3);
            filtered_pos(4) = raw_pos(4);
        }

        auto &filters_roll = filters_map[RuneFilterDataType::ROLL];
        if (!filters_roll.empty())
        {
            // 取第一个滤波器的 roll 项作为融合结果
            auto &roll_result_temp = filter_results[filters_roll[0]];
            filtered_pos(5) = roll_result_temp(5);
        }
        else
        {
            // 如果没有 Roll 滤波器，直接取原数据作为融合结果
            filtered_pos(5) = raw_pos(5);
        }

        // 设置融合结果
        result = filtered_pos;
    }

    // 更新滤波次数
    __filter_count++;
    // 更新最新值
    __latest_value = result;

    FilterOutput output;
    output.filtered_pos = result;
    return output;
}

Matx61d RuneFilterFusion::getPredict()
{
    RuneFilter_ptr filter_xyz = nullptr;
    RuneFilter_ptr filter_yp = nullptr;
    RuneFilter_ptr filter_roll = nullptr;

    for (const auto &filter : _filters)
    {
        if (filter->getDataType() == RuneFilterDataType::XYZ)
        {
            filter_xyz = filter;
        }
        else if (filter->getDataType() == RuneFilterDataType::YAW_PITCH)
        {
            filter_yp = filter;
        }
        else if (filter->getDataType() == RuneFilterDataType::ROLL)
        {
            filter_roll = filter;
        }
    }

    if (filter_xyz == nullptr || filter_yp == nullptr || filter_roll == nullptr)
    {
        VC_THROW_ERROR("滤波器未初始化");
    }

    if (!filter_xyz->isValid() || !filter_yp->isValid() || !filter_roll->isValid())
    {
        VC_THROW_ERROR("滤波器无效");
    }

    // 获取所有子滤波器的预测值
    auto xyz_predict = filter_xyz->getPredict();
    auto yp_predict = filter_yp->getPredict();
    auto roll_predict = filter_roll->getPredict();

    // 将所有子滤波器的预测值进行融合
    Matx61d predict(xyz_predict(0), xyz_predict(1), xyz_predict(2),
                    yp_predict(3), yp_predict(4), roll_predict(5));
    return predict;
}

void RuneFilterFusion::initFilter(const Matx61d &first_pos, const int64 tick)
{
    auto filter_ekf_xyz = RuneFilterEKF_CV::make_filter(RuneFilterDataType::XYZ);
    _filters.push_back(filter_ekf_xyz);
    auto filter_ekf_yp = RuneFilterEKF_CV::make_filter(RuneFilterDataType::YAW_PITCH);
    _filters.push_back(filter_ekf_yp);
    auto filter_ekf_roll = RuneFilterEKF_CV::make_filter(RuneFilterDataType::ROLL);
    _filters.push_back(filter_ekf_roll);

    // 初始化Xyz33滤波器
    // auto filter_xyz33 = RuneFilterXyzStatic::make_filter();
    // _filters.push_back(filter_xyz33);

    // // 初始化XyzUniform滤波器
    // auto filter_xyz_uniform = RuneFilterXyzUniform::make_filter();
    // _filters.push_back(filter_xyz_uniform);

    // //初始化XyzAccelerate滤波器
    // auto filter_xyz_accelerate = RuneFilterXyzAccelerate::make_filter();
    // _filters.push_back(filter_xyz_accelerate);

    // // 初始化UvdUniform滤波器
    // auto filter_uvd_uniform = RuneFilterUvdUniform::make_filter();
    // _filters.push_back(filter_uvd_uniform);

    // // 初始化UvdAccelerate滤波器
    // auto filter_uvd_accelerate = RuneFilterUvdAccelerate::make_filter();
    // _filters.push_back(filter_uvd_accelerate);

    // // 初始化yaw pitch滤波器
    // auto filter_yp22 = RuneFilterYpStatic::make_filter();
    // _filters.push_back(filter_yp22);

    // // 初始化yaw pitch滤波器
    // auto filter_yp_uniform = RuneFilterYpUniform::make_filter();
    // _filters.push_back(filter_yp_uniform);

    // // 初始化yaw pitch滤波器
    // auto filter_yp_accelerate = RuneFilterYpAccelerate::make_filter();
    // _filters.push_back(filter_yp_accelerate);

    // // // 初始化roll滤波器
    // auto filter_roll = RuneFilterRollAccelerate::make_filter();
    // _filters.push_back(filter_roll);
}

bool RuneFilterFusion::isValid()
{

    RuneFilter_ptr filter_xyz = nullptr;
    RuneFilter_ptr filter_yp = nullptr;
    RuneFilter_ptr filter_roll = nullptr;

    for (const auto &filter : _filters)
    {
        if (filter->getDataType() == RuneFilterDataType::XYZ)
        {
            filter_xyz = filter;
        }
        else if (filter->getDataType() == RuneFilterDataType::YAW_PITCH)
        {
            filter_yp = filter;
        }
        else if (filter->getDataType() == RuneFilterDataType::ROLL)
        {
            filter_roll = filter;
        }
    }

    if(_filters.size() == 1)
    {
        if(_filters.front()->isValid())
            return true;
        else
            return false;
    }


    if (filter_xyz == nullptr || filter_yp == nullptr || filter_roll == nullptr)
    {
        return false;
    }

    if (!filter_xyz->isValid() || !filter_yp->isValid() || !filter_roll->isValid())
    {
        return false;
    }

    return true;
}
