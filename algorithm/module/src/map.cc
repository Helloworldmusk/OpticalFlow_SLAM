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
        if(dsp_actived_keyframes_.size() > kMaxActivedFrameNums)
        {
                //TODO(snowden)[high] : just need delete owned mappoint, otherwise will delete one mappoint twice;
                DLOG_INFO << "  before erase , dsp_actived_mappoints_  num : " <<  dsp_actived_mappoints_ .size()<< std::endl;
                auto end =  dsp_actived_mappoints_.begin() + dsp_actived_keyframes_.front()->sp_frame_->linked_mappoint3d_nums > dsp_actived_mappoints_.end() ? 
                                dsp_actived_mappoints_.end() : dsp_actived_mappoints_.begin() + dsp_actived_keyframes_.front()->sp_frame_->linked_mappoint3d_nums;
                dsp_actived_mappoints_.erase(dsp_actived_mappoints_.begin(),  end );
                // DLOG_INFO << " finished erase " << std::endl;
                int erase_num = dsp_actived_keyframes_.front()->sp_frame_->linked_mappoint3d_nums;  //TODO(snowden) : just for debug, need be deleted;
                dsp_actived_keyframes_.pop_front();
                DLOG_INFO << "  after erase , dsp_actived_mappoints_  num : " <<  dsp_actived_mappoints_ .size() << " total erase : " << erase_num << " points" << std::endl;

        }
        actived_keyframes_lock_.unlock();
        //add actived mappoint;
        std::unique_lock<std::mutex> actived_mappoints_lock_{ actived_mappoint_mutex_ };
        int64_t push_counter { 0 };       
        int64_t link_null_mappoint_counter { 0 };
        for(auto p : sp_keyframe->sp_frame_->vsp_left_feature_)
        {
                if(nullptr != p->get_mappoint3d_linked())
                {
                        //TODO(snowden)[mid] : the follow loop just for debug, can be deleted;
                        int64_t counter = 0;
                        for ( auto e : p->get_mappoint3d_linked()->vwp_observers_)
                        {
                                if(nullptr == e.lock())
                                {
                                        DLOG_FATAL << "feature id "  << p->id_ << " linked mappoint 's observers is null ???????? " << std::endl;
                                        DLOG_INFO << "p->get_mappoint3d_linked()->vwp_observers_.size()"  << p->get_mappoint3d_linked()->vwp_observers_.size() << std::endl;
                                        exit(0);
                                }
                                else if(p == e.lock())
                                {
                                        /**
                                         * TODO(snowden)[high]: many mappoints add by many different frame, 
                                         * this is will weast some cup resource, duplicate mappoint will be optimized in backend;
                                         * so, just not duplicated mappoint need to push back
                                         */  
                                        dsp_actived_mappoints_.push_back(p->get_mappoint3d_linked());
                                        push_counter++;
                                }
                                else
                                {
                                     counter++;   
                                }
                        }
                        if(counter == p->get_mappoint3d_linked()->vwp_observers_.size())
                        {
                                DLOG_INFO << "feature id : " << p->id_ << "linked with mappoint ,but mappoint not be observed by feature" << std::endl;
                        }
                }
                else
                {
                        link_null_mappoint_counter++;
                }
        }
        actived_mappoints_lock_.unlock();
        // DLOG_INFO << "######################## push_counter : " << push_counter <<  " VS  linked_mappoint3d_num : " << sp_keyframe->sp_frame_->linked_mappoint3d_nums << std::endl;
        // DLOG_INFO << " link_null_mappoint_counter " <<link_null_mappoint_counter << std::endl;
        DLOG_INFO << " sp_keyframe->sp_frame_->vsp_left_feature_  " << sp_keyframe->sp_frame_->vsp_left_feature_.size()<< std::endl;
        DLOG_INFO << "dsp_actived_keyframes_.size()" <<dsp_actived_keyframes_.size()<<std::endl;
        DLOG_INFO << "dsp_actived_mappoints_.size()" <<dsp_actived_mappoints_.size()<<std::endl;
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

