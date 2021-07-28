#include "algorithm/module/include/map.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {




/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
bool Map::add_frame(std::shared_ptr<Frame> sp_frame)
{
        std::unique_lock<std::mutex> frames_lock_{frames_mutex_};       
        vsp_frames_.push_back(sp_frame);
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
bool Map::add_keyframe(std::shared_ptr<KeyFrame> sp_keyframe)
{
        std::unique_lock<std::mutex> keyframe_lock_{keyframe_mutex_};       
        vsp_keyframes_.push_back(sp_keyframe);
        keyframe_lock_.unlock();
        //
        std::unique_lock<std::mutex> actived_keyframe_lock_{actived_keyframe_mutex_};       
        dsp_actived_keyframes_.push_back(sp_keyframe);
        if(dsp_actived_keyframes_.size() > kMaxActivedFrameNums)
        {
                DLOG_INFO << "  before erase , dsp_actived_mappoints_  num : " <<  dsp_actived_mappoints_ .size()<< std::endl;
                dsp_actived_mappoints_.erase(dsp_actived_mappoints_.begin(), 
                                                                                   dsp_actived_mappoints_.begin() + dsp_actived_keyframes_.front()->linked_mappoint3d_nums );
                int erase_num = dsp_actived_keyframes_.front()->linked_mappoint3d_nums;  //TODO(snowden) : just for debug, need be deleted;
                dsp_actived_keyframes_.pop_front();
                DLOG_INFO << "  after erase , dsp_actived_mappoints_  num : " <<  dsp_actived_mappoints_ .size()<< std::endl;

        }
        actived_keyframe_lock_.unlock();
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
bool Map::add_mappoint(std::shared_ptr<Mappoint3d> sp_mappoint)
{
        std::unique_lock<std::mutex> mappoint_lock_{mappoint_mutex_};       
        vsp_mappoints_.push_back(sp_mappoint);
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
std::vector<std::shared_ptr<Frame>> Map::get_frames()
{
        std::unique_lock<std::mutex> frames_lock_{frames_mutex_};       
        return vsp_frames_;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
std::vector<std::shared_ptr<KeyFrame>> Map::get_keyframes()
{
        std::unique_lock<std::mutex> keyframe_lock_{keyframe_mutex_};
        return vsp_keyframes_;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
std::vector<std::shared_ptr<Mappoint3d>> Map::get_mappoints()
{
        std::unique_lock<std::mutex> mappoint_lock_{mappoint_mutex_};
        return vsp_mappoints_;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
std::deque<std::shared_ptr<KeyFrame>> Map::get_actived_keyframes()
{
        std::unique_lock<std::mutex> actived_keyframe_lock_{actived_keyframe_mutex_};       
        return dsp_actived_keyframes_;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-28
 * @return 
 * @version 1.0
 */
std::deque<std::shared_ptr<Mappoint3d>> Map::get_actived_mappoints()
{
        std::unique_lock<std::mutex> actived_mappoint_lock_{actived_mappoint_mutex_};       
        return dsp_actived_mappoints_;
}



} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

