#include "algorithm/module/include/tracker.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Tracker::Tracker( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                                             const std::shared_ptr<CameraConfig> sp_camera_config , std::string dataset_path)
        : wp_map_(map), sp_slam_config_(sp_slam_config), sp_camera_config_(sp_camera_config), dataset_path_(dataset_path)
{
        is_running_.store(true);
        
        front_end_thread_ = std::thread(std::bind(&Tracker::front_end_loop,this));
        gftt_detector_ = cv::GFTTDetector::create(sp_slam_config_->features_expected_nums, 0.01, 20);
}


/**
 * @brief  join front_end_thread;
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Tracker::stop()
{
        is_running_.store(false);
        notify_all_updated_map();  
        front_end_thread_.join();
        DLOG_INFO << " Tracker stoped " << std::endl;
}


/**
 * @brief  change front end status in some constraint;
 * @return true if change successfully;
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::set_front_end_status(const FrontEndStatus &new_status)
{
        bool is_allowed_change { false };
        switch (new_status)
        {
                case FrontEndStatus::READY:
                {
                        if (enum_front_end_status_ == FrontEndStatus::UNKNOW )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::INITING:
                {
                        if (enum_front_end_status_ == FrontEndStatus::READY || enum_front_end_status_ == FrontEndStatus::RESET)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::TRACKING:
                {
                        if (enum_front_end_status_ == FrontEndStatus::INITING || enum_front_end_status_ == FrontEndStatus::NEED_INSERT_KEYFRAM)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::NEED_INSERT_KEYFRAM:
                {
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::LOST:
                {
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::RESET:
                {
                        if (enum_front_end_status_ == FrontEndStatus::LOST || enum_front_end_status_ == FrontEndStatus::INITING)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case FrontEndStatus::FINISHED:
                {
                        //TODO(snowden) : need to decide which status can change to FINISHED, may be all could;
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING || enum_front_end_status_ == FrontEndStatus::NEED_INSERT_KEYFRAM )
                        {
                                is_allowed_change = true;
                        }
                        break;
                } 
                default:
                {
                        LOG_FATAL <<  "set_front_end_status fatal: unknow status : " << static_cast<int64_t>(new_status) << std::endl; 
                        break;
                }      
         }

        if (is_allowed_change)
        {
                enum_front_end_status_ = new_status;
        }
        else
        {
                LOG_WARNING << " set_front_end_status from  " <<  static_cast<int64_t>(enum_front_end_status_)  << " to " << static_cast<int64_t>(new_status)  << " is not allowed " << std::endl;
        }

}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Tracker::front_end_loop()
{
        while(is_running_)
        {
                DLOG_INFO << " front_end loop is running " << std::endl;
                switch (get_front_end_status() )
                {
                        case FrontEndStatus::READY:
                        {
                                DLOG_INFO << " FrontEndStatus::READY " << std::endl;
                                set_front_end_status(FrontEndStatus::INITING);
                        }
                        case FrontEndStatus::INITING:
                        {
                                DLOG_INFO << " FrontEndStatus::INITING " << std::endl;
                                if (!init_front_end())
                                {
                                        LOG_ERROR << " init_front_end failed , now ,rest front end " << std::endl;
                                        set_front_end_status(FrontEndStatus::RESET);
                                }
                                else
                                {
                                        notify_all_updated_map();
                                        set_front_end_status(FrontEndStatus::TRACKING);
                                }
                                break;
                        }
                        case FrontEndStatus::TRACKING:
                        {
                                DLOG_INFO << " FrontEndStatus::TRACKING " << std::endl;
                                auto start_time =  std::chrono::steady_clock::now();
                                FrontEndStatus status =  tracking();
                                auto end_time =  std::chrono::steady_clock::now();                                
                                if (FrontEndStatus::TRACKING == status)
                                {
                                        notify_all_updated_map();
                                        auto time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(end_time - start_time);
                                        if( sp_slam_config_->per_frame_process_time - time_used.count()*1000.0 > 0)
                                        {
                                                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(sp_slam_config_->per_frame_process_time - time_used.count()*1000)));
                                        }
                                }
                                else
                                {
                                        set_front_end_status(status);
                                }
                                break;
                        }
                        //TODO(snowden) : if after TRACKING then directly NEED_INSERT_KEYFRAM, or run another turn ? need to decide;
                        case FrontEndStatus::NEED_INSERT_KEYFRAM:
                        {
                                DLOG_INFO << " FrontEndStatus::NEED_INSERT_KEYFRAM " << std::endl;
                                CHECK_EQ(insert_keyframe(), true);
                                set_front_end_status(FrontEndStatus::TRACKING);
                                break;
                        }
                        case FrontEndStatus::LOST:
                        {
                                DLOG_INFO << " FrontEndStatus::LOST " << std::endl;
                                set_front_end_status(FrontEndStatus::RESET);
                                break;
                        }
                        case FrontEndStatus::RESET:
                        {
                                DLOG_INFO << " FrontEndStatus::RESET " << std::endl;
                                CHECK_EQ(reset(), true);
                                set_front_end_status(FrontEndStatus::INITING);
                                break;
                        }
                        case FrontEndStatus::FINISHED:
                        {
                                DLOG_INFO << " FrontEndStatus::FINISHED " << std::endl;
                                notify_all_updated_map();
                                //reserve a little time for back_end to deal with last frame;
                                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                // notify_all_tracker_finished();
                                break;
                        }
                        case FrontEndStatus::UNKNOW:
                        {
                                DLOG_INFO << " FrontEndStatus is UNKNOW " << std::endl;
                                set_front_end_status(FrontEndStatus::READY);
                                break;
                        }
                        default:
                        {
                                LOG_FATAL << " illegal FrontEndStatus :  " << static_cast<int>(get_front_end_status())  << std::endl;
                                break;
                        }
                } //switch
        } //while(is_running_)
} //void Tracker::front_end_loop()


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::init_front_end()
{
        //TODO(snowden): init;
        sp_current_frame_ = get_a_frame();
        sp_current_frame_->set_left_pose(SE3(SO3(), Vec3(0,0,0)));
        sp_current_frame_->set_right_pose(SE3(SO3(),sp_camera_config_->base_line));
        if( nullptr ==  sp_current_frame_) { return false; }
        if (detect_left_image_features() < sp_slam_config_->features_init_min_threshold) { return false; }
        if (track_feature_in_right_image() < sp_slam_config_->features_init_min_threshold) { return false; }
        if (init_map() < sp_slam_config_->mappoint_init_min_threshold) { return false; }
        return true;
}


/**
 * @brief   tracking point in frames ,and give a rough pose and 3d position;
 * @return  if no image , return FINISHED;  if  tracked little point , return NEED_INSERT_KEYFRAM, 
 *                    if tracked too little point, return LOST, if track normal ,return TRACKING
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
Tracker::FrontEndStatus Tracker::tracking()
{
        SHOW_FUNCTION_INFO
        static int counter = 0;
        counter++;
        //TODO(snowden) : tracking;
        if(counter < 10)
        {
                return FrontEndStatus::TRACKING;
        }
        else if (counter < 11)
        {
                return FrontEndStatus::NEED_INSERT_KEYFRAM;
        }
        else if (counter < 12)
        {
                return FrontEndStatus::LOST;
        }
        else if(counter < 13)
        {
                return FrontEndStatus::FINISHED;
        }

}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::insert_keyframe()
{
        SHOW_FUNCTION_INFO
        //TODO(snowden): insert keyframe;
        return true;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::reset()
{
        //TODO(snowden) : )if reset ,you should set all static variable to default value; for example Frame id;
        static int64_t reset_counter = 0;
        reset_counter++;
        //TODO(snowden): there is need use a variable , not a magic;
        if(reset_counter > 5)
        {
                LOG_FATAL << " tracker reset is over " << reset_counter - 1 << " times " << std::endl;
        }
        return true;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note notify map updated to viewer thread and back_end thread;
 */
void Tracker::notify_all_updated_map()
{
        wp_map_.lock()->condition_var_is_map_updated_.notify_all();
        DLOG_INFO << " tracker notify all " << std::endl;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note notify tracker finished to slam system, optimizer, viewer thread;
 */
void Tracker::notify_all_tracker_finished()
{
        condition_variable_is_tracker_finished_.notify_all();
        DLOG_INFO << " notify all tracker finished " << std::endl;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-23
 * @version 1.0
 */
std::shared_ptr<Frame> Tracker::get_a_frame()
{
        std::shared_ptr<Frame> sp_frame = std::shared_ptr<Frame>(new Frame);
        
        cv::Mat left_image;
        cv::Mat right_image;
        std::stringstream temp;
        temp << std::setw(6) << std::setfill('0') << Frame::get_new_id();
        std::string left_image_path = dataset_path_ + "/image_0/" +temp.str() + ".png" ;
        left_image = cv::imread(left_image_path.c_str(),cv::IMREAD_GRAYSCALE);
        std::string right_image_path = dataset_path_ + "/image_1/" +temp.str() + ".png" ;
        right_image = cv::imread(right_image_path.c_str(),cv::IMREAD_GRAYSCALE);
        CHECK_EQ(left_image.data == nullptr, false);
        CHECK_EQ(right_image.data == nullptr, false);

        sp_frame->left_image_ = left_image;
        sp_frame->right_image_ = right_image;

        static std::ifstream fin(dataset_path_+"/times.txt");
        if(!fin)
        {
                LOG_FATAL << " open " << dataset_path_ << "/times.txt  failed " << std::endl; 
        }
        fin >> sp_frame->timestamp_ ;
        if(fin.eof())
        {
                return nullptr;
        }
        DLOG_INFO << " frame time stamp is " << sp_frame->timestamp_ << std::endl;
        return sp_frame;
}



/**
 * @brief  
 * @author snowden
 * @date 2021-07-23
 * @return the number of feature detected;
 * @version 1.0
 */
int64_t Tracker::detect_left_image_features()
{
        cv::Mat mask(sp_current_frame_->left_image_.size(), CV_8UC1, cv::Scalar(0));
        
        std::vector<cv::KeyPoint> keypoints;
        if(sp_last_frame_)
        {
                for(auto& feature:  sp_last_frame_->vsp_left_feature_)
                {
                        cv::rectangle(mask, feature->cv_keypoint_.pt - cv::Point2f(10,10),  feature->cv_keypoint_.pt + cv::Point2f(10,10), cv::Scalar(1), CV_FILLED);
                }
                gftt_detector_->detect(sp_current_frame_->left_image_, keypoints, mask);
        }
        else
        {
                // cv::rectangle(mask, cv::Point2f(0,0),  cv::Point2f(sp_current_frame_->left_image_.cols,sp_current_frame_->left_image_.rows)  , cv::Scalar(1), CV_FILLED);
                gftt_detector_->detect(sp_current_frame_->left_image_, keypoints);
        }

        for(auto& keypoint : keypoints)
        {
               std::shared_ptr<Feature2d> feature_point = std::shared_ptr<Feature2d>(new Feature2d(keypoint));
               feature_point->set_frame_linked(sp_current_frame_);
               sp_current_frame_->vsp_left_feature_.push_back(feature_point);
               cv::circle(sp_current_frame_->left_image_, keypoint.pt, 3, cv::Scalar(255,255,255));
        }
        // cv::imshow("frame", sp_current_frame_->left_image_);
        // cv::waitKey(1000);
        return keypoints.size();
}



int64_t Tracker::track_feature_in_right_image()
{
        std::vector<cv::Point2f> v_left_keypoints;
        std::vector<cv::Point2f> v_right_keypoints;
        // give a init position;
        for(auto& feature: sp_current_frame_->vsp_left_feature_)
        {
                // std::shared_ptr<Feature2d> new_feature = nullptr;
                v_left_keypoints.emplace_back(feature->position2d_.x(), feature->position2d_.y() );
                auto mappoint =  feature->get_mappoint3d_linked();
                if(mappoint)
                {
                        Vec2 project_position = CoordinateTransformWorldToImage(mappoint, sp_current_frame_->get_right_pose(),  sp_camera_config_->K_left);
                        v_right_keypoints.emplace_back(project_position(0),project_position(1));
                }
                else
                {
                        //for : first frame, no mappoint;
                        v_right_keypoints.emplace_back(feature->position2d_.x(), feature->position2d_.y() );
                }                
        }
        //optical flow track;
        std::vector<uchar> status;
        cv::Mat error;
        int64_t good_point_counter = 0;
        cv::calcOpticalFlowPyrLK(sp_current_frame_->left_image_, sp_current_frame_->right_image_, 
                                                              v_left_keypoints, v_right_keypoints, status, error); 
        for (size_t i = 0; i < status.size(); i++)
        {
                if (status[i])
                {
                        std::shared_ptr<Feature2d> feature 
                                = std::shared_ptr<Feature2d>(new Feature2d(Vec2(v_right_keypoints[i].x, v_right_keypoints[i].y)));
                        feature->set_frame_linked(sp_current_frame_);
                        sp_current_frame_->vsp_right_feature_.push_back(feature);
                        good_point_counter++;
                }
                else
                {
                        sp_current_frame_->vsp_right_feature_.push_back(nullptr);
                        //TODO(snowden) : is need  set the left Feature point  as outliners ? 
                        //sp_current_frame_->vsp_left_feature_[i]->is_outline_ = true;  
                }
        }
        DLOG_INFO << " track in right image : good point counter : " << good_point_counter << std::endl;
        return good_point_counter;
        //discard outliners;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-23
 ** @return the number of mappoint successfully triangulated;
 * @version 1.0
 */
int64_t Tracker::init_map()
{
        /**access each element, if right feature not null , change coordinate to normalized plane ,  construct a eq to solve , then 
         *  check if meet the requirement,  then push back 3d point to vector, 
         */
        Vec3 normalized_left_point;
        Vec3 normalized_right_point;
        Vec3 points_3d;
        int64_t counter = 0;
         if(wp_viewer_.lock())
         {
                 wp_viewer_.lock()->vsp_mappoints_.clear();
         }
        for (int i = 0; i < sp_current_frame_->vsp_left_feature_.size(); i++)
        {
                if(nullptr ==  sp_current_frame_->vsp_right_feature_[i]) { continue; }
                normalized_left_point = CoordinateTransformImageToNormalizedPlane(sp_current_frame_->vsp_left_feature_[i]->position2d_, 
                                                                                                                                                                      sp_camera_config_->K_left);
                normalized_right_point = CoordinateTransformImageToNormalizedPlane(sp_current_frame_->vsp_right_feature_[i]->position2d_, 
                                                                                                                                                                          sp_camera_config_->K_right);
                if(!TriangulateNormalizedPoint(normalized_left_point, normalized_right_point, 
                                                                                  sp_current_frame_->get_left_pose(), sp_current_frame_->get_right_pose(), points_3d ))
                {       
                        continue;
                }
                 counter++;
                //  DLOG_INFO << " ######### point3d: "  << points_3d << std::endl;
                 std::shared_ptr<Mappoint3d> new_mappoint = std::shared_ptr<Mappoint3d>(new Mappoint3d(sp_current_frame_->timestamp_, points_3d));
                 new_mappoint->vwp_observers_.push_back(sp_current_frame_->vsp_left_feature_[i]);
                 new_mappoint->vwp_observers_.push_back(sp_current_frame_->vsp_right_feature_[i]);
                 sp_current_frame_->vsp_left_feature_[i]->set_mappoint3d_linked(new_mappoint);
                 sp_current_frame_->vsp_right_feature_[i]->set_mappoint3d_linked(new_mappoint);
                 wp_map_.lock()->add_mappoint(new_mappoint);
                 sp_current_frame_->linked_mappoint3d_nums++;
                if(wp_viewer_.lock())
                {
                        wp_viewer_.lock()->vsp_mappoints_.push_back(new_mappoint);
                }
        }
        wp_map_.lock()->add_frame(sp_current_frame_);
        std::shared_ptr<KeyFrame> keyframe = std::shared_ptr<KeyFrame>(new KeyFrame(sp_current_frame_));
        wp_map_.lock()->add_keyframe(keyframe);
        DLOG_INFO << "************************************************* init map : total map point : " << counter << std::endl;
        return counter;
}



} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam