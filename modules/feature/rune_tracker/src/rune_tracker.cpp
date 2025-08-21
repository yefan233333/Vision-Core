#include "vc/feature/rune_tracker.h"
#include "vc/feature/rune_tracker_param.h"


using namespace std;
using namespace cv;


void RuneTracker::updateFromRune(FeatureNode_ptr p_combo)
{
    setImageCache(p_combo->getImageCache());
    setPoseCache(p_combo->getPoseCache());
    getHistoryNodes().push_back(p_combo);
    getHistoryTicks().push_back(p_combo->getTick());
}


void RuneTracker::update(FeatureNode_ptr p_rune, int64 tick, const GyroData &gyro_data)
{
    // 数据更新
    updateFromRune(p_rune);
    auto& __history_deque = getHistoryNodes();
    auto& __tick_deque = getHistoryTicks();

    if (__history_deque.size() >= static_cast<size_t>(rune_tracker_param.MAX_DEQUE_SIZE))
        __history_deque.pop_back();
    if (__tick_deque.size() >= static_cast<size_t>(rune_tracker_param.MAX_DEQUE_SIZE))
        __tick_deque.pop_back();
}


void RuneTracker::updateVisible(bool is_visible)
{
    auto& __vanish_num = getDropFrameCount();
    if (is_visible)
    {
        __vanish_num = 0;
    }
    else
    {
        ++__vanish_num;
    }
}
