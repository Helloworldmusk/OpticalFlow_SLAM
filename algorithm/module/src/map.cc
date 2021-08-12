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
        std::unique_lock<std::mutex> keyframe_lock_{ keyframe_mutex_ };       
        vsp_keyframes_.push_back(sp_keyframe);
        keyframe_lock_.unlock();
        //add actived keyframe;
        std::unique_lock<std::mutex> actived_keyframes_lock_{ actived_keyframe_mutex_ };       
        dsp_actived_keyframes_.push_back(sp_keyframe);
        int64_t delete_counter { 0 };
        if(dsp_actived_keyframes_.size() > kMaxActivedFrameNums)
        {
                //TODO(snowden)[high] : just need delete owned mappoint, otherwise will delete one mappoint twice;

                // get the second  keyframe in deque, if a mappoint observed by the second keyframe, it will reserved, otherwise, it will be deleted;
                std::shared_ptr<KeyFrame> second_keyframe = *(dsp_actived_keyframes_.begin() + 1); 
                DLOG_INFO << " keyframe deque front id : " << (*(dsp_actived_keyframes_.begin()))->sp_frame_->id_ 
                                         << " keyframe deque front 2 id : " << (*(dsp_actived_keyframes_.begin() + 1))->sp_frame_->id_ << std::endl;
                //just consider mappoints which linked to this keyframe;
                auto end =  dsp_actived_mappoints_.begin() + dsp_actived_keyframes_.front()->sp_frame_->linked_mappoint3d_nums > dsp_actived_mappoints_.end() ? 
                                dsp_actived_mappoints_.end() : dsp_actived_mappoints_.begin() + dsp_actived_keyframes_.front()->sp_frame_->linked_mappoint3d_nums;
                //TODO(snowden)[mid] : if delete will cause iterator error ? 
                for(auto e = dsp_actived_mappoints_.begin(); e != end; e++)
                {
                        bool is_observed_by_next_keyframe { false };
                        for( auto  obs : (*e)->vwp_observers_)
                        {
                                if (obs.lock()->get_frame_linked().lock() == second_keyframe->sp_frame_)
                                {
                                        is_observed_by_next_keyframe = true;
                                        break;
                                }
                        } 
                        if(!is_observed_by_next_keyframe)
                        {
                                dsp_actived_mappoints_.erase(e);
                                delete_counter++;
                        }
                }
                dsp_actived_keyframes_.pop_front();
        }
        actived_keyframes_lock_.unlock();
        //add actived mappoint;
        std::shared_ptr<KeyFrame> second_to_last_keyframe = nullptr;
        if (dsp_actived_keyframes_.size() > 1)
        {
                second_to_last_keyframe = *(dsp_actived_keyframes_.end() -2);
        } 
        std::unique_lock<std::mutex> actived_mappoints_lock_{ actived_mappoint_mutex_ };
        int64_t push_counter { 0 };       
        int64_t link_null_mappoint_counter { 0 };
        int64_t increase_counter { 0 };
        for(auto p : sp_keyframe->sp_frame_->vsp_left_feature_)
        {
                if (nullptr != p->get_mappoint3d_linked())
                {
                        bool is_linked_last_frame { false };
                         for (auto obs : p->get_mappoint3d_linked()->vwp_observers_)
                         {
                                 if(second_to_last_keyframe && obs.lock()->get_frame_linked().lock() == second_to_last_keyframe->sp_frame_)
                                 {
                                         is_linked_last_frame = true;
                                 }
                         }
                         if(!is_linked_last_frame)
                         {
                                 dsp_actived_mappoints_.push_back(p->get_mappoint3d_linked());
                                 increase_counter++;
                         }
                } 
        }
        DLOG_INFO << " sp_keyframe->sp_frame_->vsp_left_feature_  " << sp_keyframe->sp_frame_->vsp_left_feature_.size()<< std::endl;
        DLOG_INFO << "  after erase , dsp_actived_mappoints_  num : " <<  dsp_actived_mappoints_ .size() << " total erase : " << delete_counter << " : total increase : " << increase_counter << std::endl;
        // DLOG_INFO << "dsp_actived_keyframes_.size()" <<dsp_actived_keyframes_.size()<<std::endl;
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

