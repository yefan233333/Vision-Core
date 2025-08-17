#pragma once

#include "vc/feature/feature_node.h"
#include "opencv2/opencv.hpp"
#include "vc/contour_proc/contour_wrapper.hpp"

// 直线类
struct Line
{
    int start_idx;
    int end_idx;
    float angle;
    cv::Point2f center;
    Line(const int &_start_idx, const int &_end_idx, const float &_angle, const cv::Point2f &_center) : start_idx(_start_idx), end_idx(_end_idx), angle(_angle), center(_center) {}
};

// 角点查找器类
class HumpDetector
{
private:
    std::vector<cv::Point> points;
    cv::Point2f direction; // 方向
    cv::Point2f avePoint;  // 中心
    int start_idx;
    int end_idx;

public:
    HumpDetector(const std::vector<cv::Point> &contour, int _start_idx, int _end_idx);
    inline cv::Point2f getDirection() { return direction; }
    inline cv::Point2f getDirection() const { return direction; }
    inline cv::Point2f getAvePoint() { return avePoint; }
    inline cv::Point2f getAvePoint() const { return avePoint; }
    inline const std::vector<cv::Point> &getPoints() { return points; }
    inline const std::vector<cv::Point> &getPoints() const { return points; }
    inline int getStartIdx() { return start_idx; }
    inline int getStartIdx() const { return start_idx; }
    inline int getEndIdx() { return end_idx; }
    inline int getEndIdx() const { return end_idx; }
    inline cv::Point2f getStartPoint() { return points.front(); }
    inline cv::Point2f getStartPoint() const { return points.front(); }
    inline cv::Point2f getEndPoint(HumpDetector) { return points.back(); }
    inline cv::Point2f getEndPoint() const { return points.back(); }

    /**
     * @brief 三点共线判断
     * @param[in] p1 点1
     * @param[in] p2 点2
     * @param[in] p3 点3
     * @param[in] max_delta_angle 最大容许偏移角（单位为角度）
     * @return  角度偏差量(单位为角度)
     */
    static double CheckCollinearity(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &p3);

    /**
     * @brief 三射线同向判断
     * @param[in] direction_1 射线1的方向
     * @param[in] direction_2 射线2的方向
     * @param[in] direction_3 射线3的方向
     * @param[in] max_delta_angle 最大容许偏移角（单位为角度）
     * @return 角度偏差量
     */
    static double CheckAlignment(const cv::Point2f &direction_1, const cv::Point2f &direction_2, const cv::Point2f &direction_3);
};

// 突起点点类
class RuneFanHump
{
protected:
    cv::Point2f center{};    // 突起的中心点
    cv::Point2f vertex{};    // 突起点
    cv::Point2f direction{}; // 突起的方向（中心点指向突起点）
public:
    virtual ~RuneFanHump() = 0;
    inline cv::Point2f getCenter() { return center; }
    inline cv::Point2f getCenter() const { return center; }
    inline cv::Point2f getVertex() { return vertex; }
    inline cv::Point2f getVertex() const { return vertex; }
    inline cv::Point2f getDirection() { return direction; }
    inline cv::Point2f getDirection() const { return direction; }

    /**
     * @brief 获取可视化的突起点轮廓
     */
    virtual Contour_ptr getContour() const
    {
        if (__contour_ptr == nullptr)
        {
            std::vector<cv::Point> contour;
            contour.push_back(center);
            contour.push_back(vertex);
            contour.push_back(center + direction * 20);
            __contour_ptr = ContourWrapper<int>::make_contour(contour);
        }
        return __contour_ptr;
    }

private:
    // 调试用的轮廓
    // Contour_ptr __contour_ptr;
    mutable Contour_ptr __contour_ptr = nullptr;
};
inline RuneFanHump::~RuneFanHump() = default;

struct TopHumpCombo;

// 顶部突起类
class TopHump : public RuneFanHump
{
private:
    enum state
    {
        UP,
        DOWN
    };

    int up_start_idx = -1;   // 上升点的起始下标
    int down_start_idx = -1; // 下降点的起始下标
    int up_end_idx = -1;     // 上升点的结束下标
    int down_end_idx = -1;   // 下降点的结束下标

    int line_pair_idx = -1; // 线段组的下标

    int up_itertaion_num = 0;   // 上升点的迭代次数
    int down_itertaion_num = 0; // 下降点的迭代次数

    int end_iteration_up_idx; // 最后一次迭代时上升点的下标
    state current_state = UP; // 当前状态(默认为上升)
public:
    TopHump() = default;
    /**
     * @brief 构造函数
     * @param[in] up_start_idx 上升点的起始下标
     * @param[in] up_end_idx 上升点的结束下标
     * @param[in] down_start_idx 下降点的起始下标
     * @param[in] down_end_idx 下降点的结束下标
     * @param[in] _direction 突起点的方向
     * @param[in] _center 突起的中心点
     */
    TopHump(int _up_start_idx, int _up_end_idx,
            int _down_start_idx, int _down_end_idx,
            int _line_pair_idx,
            const cv::Point2f &_direction, const cv::Point2f &_center);
    /**
     * @brief 使用上升点和下降点的下标构造突起点
     * @param[in] up_idx   上升点的下标
     * @param[in] down_idx 下降点的下标
     * @param[in] find_state 查找状态（上升还是下降）
     * @param[in] _direction 突起点的方向
     * @return 是否查找成功
     */
    TopHump(int up_idx, int down_idx, state find_state, const cv::Point2f &_direction);
    /**
     * @brief 获取顶部突起点数组（长度为3）
     * @param[in] contour_plus 加长后的轮廓
     * @param[in] contour_center 原轮廓的中心点
     * @param[in,out] line_pairs 待匹配的直线对
     * @return 成功则返回突起点数组,否则返回空数组
     */
    static std::vector<TopHump> getTopHumps(const std::vector<cv::Point> &contour_plus, const cv::Point2f &contour_center, const std::vector<std::tuple<Line, Line>> &line_pairs);

    /**
     * @brief 获取顶部突起点组合体数组
     */
    static std::vector<TopHumpCombo> getTopHumpCombos(const std::vector<Contour_ptr> &contours,
                                                      const std::vector<cv::Point2f> &contour_centers,
                                                      const std::vector<std::tuple<Line, Line>> &line_pairs);

    inline int getLinePairIdx() { return line_pair_idx; }
    inline int getLinePairIdx() const { return line_pair_idx; }

    static bool make_TopHumps(TopHumpCombo &hump_combo, const cv::Point2f &contour_center, double &delta);
    static bool setVertex(TopHump &hump, const std::vector<cv::Point> &contour_plus);
    /**
     * @brief 过滤异常突起点
     *
     * @param[in] contour_plus 加长后的轮廓
     * @param[in,out] humps 输入突起点集，输出过滤后的突起点集。
     */
    static bool filter(const std::vector<cv::Point> &contour_plus,
                       std::vector<TopHump> &humps);

private:
    void update(const TopHump &hump);
    static bool getAllHumps2(const std::vector<cv::Point> &contours_plus,
                             const std::vector<std::tuple<Line, Line>> &line_pairs,
                             std::vector<TopHump> &humps);
};

struct TopHumpCombo
{
    TopHumpCombo(const TopHump &hump_1, const TopHump &hump_2, const TopHump &hump_3)
    {
        __humps[0] = hump_1;
        __humps[1] = hump_2;
        __humps[2] = hump_3;
    }
    // 获取调试用的轮廓
    Contour_ptr getContour() const
    {
        if (__contour_ptr == nullptr)
        {
            __contour_ptr = ContourWrapper<int>::getConvexHull({__humps[0].getContour(), __humps[1].getContour(), __humps[2].getContour()});
        }
        return __contour_ptr;
    }
    std::tuple<TopHump, TopHump, TopHump> getHumpsTuple() const
    {
        return std::make_tuple(__humps[0], __humps[1], __humps[2]);
    }
    std::vector<TopHump> getHumpsVector()
    {
        std::vector<TopHump> humps;
        humps.push_back(__humps[0]);
        humps.push_back(__humps[1]);
        humps.push_back(__humps[2]);
        return humps;
    }

    std::array<TopHump, 3> &humps()
    {
        return __humps;
    }

private:
    // 调试用的轮廓
    std::array<TopHump, 3> __humps; // 三个突起点
    mutable Contour_ptr __contour_ptr = nullptr;
};

// 底部中心突起类
class BottomCenterHump : public RuneFanHump
{
private:
    int idx = 0; // 突起点的下标(在原轮廓中)
public:
    BottomCenterHump() = default;
    /**
     * @brief 构造函数
     * @param[in] _center 突起的中心点
     * @param[in] direction 突起的方向
     * @param[in] _idx 突起点的下标
     * @note contour[idx] != _center, _center 为 contour[idx] 和 contour[idx - 1] 的中点
     */
    BottomCenterHump(const cv::Point2f &_center, const cv::Point2f &direction, const int _idx);
    /**
     * @brief 获取底部中心突起点数组
     * @param[in] contour 原轮廓
     * @param[in] contour_center 原轮廓的中心点
     * @param[in] top_humps 辅助查找的顶部突起点数组
     * @return 成功则返回突起点数组,否则返回空数组
     */
    static std::vector<BottomCenterHump> getBottomCenterHump(const std::vector<cv::Point> &contour, const cv::Point2f &contour_center, const std::vector<TopHump> &top_humps);

private:
    /**
     * @brief 获取所有突起点
     * @param[in] contour 原轮廓
     * @param[out] humps 输出的突起点集。
     * @return 是否查找成功（找到一个以上的突起点算作成功）
     */
    static bool getAllHumps(const std::vector<cv::Point> &contour,
                            const std::vector<TopHump> &top_humps,
                            std::vector<BottomCenterHump> &humps);
    /**
     * @brief 过滤所有突起点，只保留最优的突起点
     * @param[in] contour 原轮廓
     * @param[in] top_humps 辅助查找的顶部突起点数组
     * @param[in,out] humps 输入突起点集，输出过滤后的突起点集。
     * @return 过滤后是否得到有效角点
     */
    static bool filter(const std::vector<cv::Point> &contour, const std::vector<TopHump> &top_humps, std::vector<BottomCenterHump> &humps);
};

class SideHump : public RuneFanHump
{
private:
    int up_start_idx = -1;   // 上升点的起始下标
    int down_start_idx = -1; // 下降点的起始下标
    int up_end_idx = -1;     // 上升点的结束下标
    int down_end_idx = -1;   // 下降点的结束下标
public:
    SideHump() = default;
    SideHump(const cv::Point2f &_direction, const cv::Point2f &_center);
    /**
     * @brief 获取所有的侧面突起点
     * @param[in] contour_plus 加长后的轮廓
     * @param[in] contour_center 轮廓中心
     * @param[in] top_humps 顶部突起点
     * @param[in] bottom_center_humps 底部中心突起点
     * @param[in,out] line_pairs 待识别的直线对
     * @return 所有的侧面突起点
     */
    static std::vector<SideHump> getSideHumps(const std::vector<cv::Point> &contour_plus,
                                              const cv::Point2f &contour_center,
                                              const std::vector<TopHump> &top_humps,
                                              const std::vector<BottomCenterHump> &bottom_center_humps,
                                              std::vector<std::tuple<Line, Line>> &line_pairs);

private:
    /**
     * @brief 获取所有突起点
     * @param[in] top_humps 顶部突起点
     * @param[in] line_pairs 待识别的直线对
     * @param[out] humps 输出的突起点集
     */
    static bool getAllHumps(const std::vector<TopHump> &top_humps, const std::vector<std::tuple<Line, Line>> &line_pairs, std::vector<SideHump> &humps);
    static bool setVertex(SideHump &hump, const std::vector<cv::Point> &contour_plus);
    static bool filter(std::vector<SideHump> &humps);
    static bool make_SideHumps(std::vector<SideHump> &humps, const std::vector<TopHump> &top_humps, const cv::Point2f &contour_center, double &delta);
};