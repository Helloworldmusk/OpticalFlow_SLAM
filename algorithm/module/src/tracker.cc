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
                        if (enum_front_end_status_ == FrontEndStatus::TRACKING || enum_front_end_status_ == FrontEndStatus::NEED_INSERT_KEYFRAM)
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
        auto front_end_loop_start_time =  std::chrono::steady_clock::now();
        while(is_running_)
        {
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
                                        // notify_all_updated_map();
                                        auto time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(end_time - start_time);
                                        DLOG_INFO << "########### tracking time : " << time_used.count() * 1000 << std::endl;
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
                                if (insert_keyframe())
                                {
                                        notify_all_updated_map();
                                        set_front_end_status(FrontEndStatus::TRACKING);
                                }
                                else
                                {
                                        set_front_end_status(FrontEndStatus::LOST);
                                }

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
                                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                                set_front_end_status(FrontEndStatus::INITING);
                                break;
                        }
                        case FrontEndStatus::FINISHED:
                        {
                                DLOG_INFO << " FrontEndStatus::FINISHED " << std::endl;
                                notify_all_updated_map();
                                //reserve a little time for back_end to deal with last frame;
                                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                                notify_all_tracker_finished();
                                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
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
        auto front_end_loop_end_time =  std::chrono::steady_clock::now();
        auto front_end_loop_time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(front_end_loop_end_time - front_end_loop_start_time);
        DLOG_INFO << "###### front  end loop run total time is : " << (front_end_loop_time_used.count()  - 6)* 1000 << std::endl;
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
        CHECK_NOTNULL(sp_current_frame_);
        sp_last_frame_ = sp_current_frame_;
        if(wp_viewer_.lock())
        {
                wp_viewer_.lock()->wp_current_frame_ = sp_current_frame_;
        }
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
        Tracker::FrontEndStatus status;
        // estimate current coarse  pose;
        
        sp_current_frame_ = get_a_frame();
        if( (nullptr ==  sp_current_frame_) || (500 ==  sp_current_frame_->id_)  )
        {
                DLOG_WARNING << " tracking finished " << std::endl;
                return FrontEndStatus::FINISHED;
        }
        // if(nullptr ==  sp_current_frame_)
        // {
        //         DLOG_WARNING << " tracking finished " << std::endl;
        //         return FrontEndStatus::FINISHED;
        // }
        if(wp_viewer_.lock())
        {
                wp_viewer_.lock()->wp_current_frame_ = sp_current_frame_;
        }
        SE3 frame_pose = relative_motion_ * sp_last_frame_->get_left_pose();
        sp_current_frame_->set_left_pose(frame_pose);
        sp_current_frame_->set_right_pose(SE3(SO3(),sp_camera_config_->base_line) * frame_pose);
        // track point in current frame'left image;
         auto track_feature_in_current_image_start_time =  std::chrono::steady_clock::now();
         bool is_urgent_need_keyframe { false };
        int64_t tracked_num = track_feature_in_current_image(is_urgent_need_keyframe);
        auto track_feature_in_current_image_end_time =  std::chrono::steady_clock::now();
        auto track_feature_in_current_image_time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(track_feature_in_current_image_end_time - track_feature_in_current_image_start_time);
        DLOG_INFO << "###### track_feature_in_current_image cost time : " << track_feature_in_current_image_time_used.count() * 1000 << std::endl;              
        if(tracked_num < sp_slam_config_->mappoint_need_insert_keyframe_min_threshold)
        {
                DLOG_ERROR<< "Frame:  " << sp_current_frame_->id_ <<  "  timestamp : " << sp_current_frame_->timestamp_ 
                                            << " traced feature is less than threshold : " << sp_slam_config_->features_tracking_min_threshold << std::endl;
                return FrontEndStatus::LOST;
        }
        else
        {
                DLOG_INFO << "Frame:  " << sp_current_frame_->id_ <<  "  timestamp : " << sp_current_frame_->timestamp_ 
                                         << "  Tracked " << tracked_num << std::endl;
                if(tracked_num > sp_slam_config_->features_tracking_min_threshold)
                {
                        DLOG_INFO << "tracker status is good " << std::endl;
                }
                else
                {
                        DLOG_INFO << "tracker: need insert keyframe " << std::endl;
                }
                                         
        }
        // estimate current fine pose;
        auto estimate_current_pose_start_time =  std::chrono::steady_clock::now();
        int64_t inlier_num =  estimate_current_pose();
        auto estimate_current_pose_end_time =  std::chrono::steady_clock::now();
        auto estimate_current_pose_time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(estimate_current_pose_end_time - estimate_current_pose_start_time);
        DLOG_INFO << "###### estimate_current_pose_time_used: " << estimate_current_pose_time_used.count() * 1000<< std::endl;
        // update relative_motion_
        relative_motion_ =   sp_current_frame_->get_left_pose() * sp_last_frame_->get_left_pose().inverse();
        // check if tracked point number great than threshold, otherwise need insert keyframe;
        if(is_urgent_need_keyframe)
        {
                status = FrontEndStatus::NEED_INSERT_KEYFRAM;
        }
        else if(inlier_num >  sp_slam_config_->features_tracking_min_threshold )
        {
                sp_last_frame_ = sp_current_frame_;
                status = FrontEndStatus::TRACKING;
        }
        else if (inlier_num > sp_slam_config_->mappoint_need_insert_keyframe_min_threshold ) 
        {
                status = FrontEndStatus::NEED_INSERT_KEYFRAM;
        }
        else
        {
                DLOG_INFO << " LOST  ready terminal" << std::endl;
                cv::waitKey(0);
                status = FrontEndStatus::LOST;
        }
        wp_map_.lock()->add_frame(sp_current_frame_); 
        // DLOG_INFO << "Frame id : " << sp_current_frame_->id_ << " pose : " << std::endl << sp_current_frame_->get_left_pose().matrix().inverse() << std::endl;
        // viewer show;
        return status;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-20
 * @version 1.0
 */
bool Tracker::insert_keyframe()
{
        // SE3 right_pose = SE3(sp_current_frame_->get_left_pose().so3(), sp_current_frame_->get_left_pose().translation() +  sp_camera_config_->base_line);
        auto insert_keyframe_start_time =  std::chrono::steady_clock::now();
        sp_current_frame_->set_right_pose(SE3(SO3(),sp_camera_config_->base_line) * sp_current_frame_->get_left_pose() );
        if (detect_left_image_features() < sp_slam_config_->features_init_min_threshold) { return false; }
        if (track_feature_in_right_image() < sp_slam_config_->features_init_min_threshold) { return false; }
        if (insert_mappoints() < sp_slam_config_->mappoint_init_min_threshold) { return false; }
        std::shared_ptr<KeyFrame> keyframe = std::shared_ptr<KeyFrame>(new KeyFrame(sp_current_frame_));
        wp_map_.lock()->add_keyframe(keyframe);
        sp_last_frame_ = sp_current_frame_;
        auto insert_keyframe_end_time =  std::chrono::steady_clock::now();
        auto insert_keyframe_time_used = std::chrono::duration_cast<std::chrono::duration<double_t>>(insert_keyframe_end_time - insert_keyframe_start_time);
        DLOG_INFO << "###### insert_keyframe_time_used: " << insert_keyframe_time_used.count() * 1000<< std::endl;
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
        int64_t frame_id = Frame::get_new_id();
        temp << std::setw(6) << std::setfill('0') << frame_id;
        std::string left_image_path = dataset_path_ + "/image_0/" +temp.str() + ".png" ;
        left_image = cv::imread(left_image_path.c_str(),cv::IMREAD_GRAYSCALE);
        std::string right_image_path = dataset_path_ + "/image_1/" +temp.str() + ".png" ;
        right_image = cv::imread(right_image_path.c_str(),cv::IMREAD_GRAYSCALE);
        if(left_image.data == nullptr || right_image.data == nullptr)
        {
                return nullptr;
        }
        // CHECK_EQ(left_image.data == nullptr, false);
        // CHECK_EQ(right_image.data == nullptr, false);

        sp_frame->left_image_ = left_image;
        sp_frame->right_image_ = right_image;
        sp_frame->id_ = frame_id;

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
        
        if(sp_last_frame_->vsp_left_feature_.size())
        {
                for(auto& feature:  sp_last_frame_->vsp_left_feature_)
                {
                        cv::rectangle(mask, feature->cv_keypoint_.pt - cv::Point2f(20,20),  feature->cv_keypoint_.pt + cv::Point2f(20,20), cv::Scalar(255), CV_FILLED);
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
               std::shared_ptr<Feature2d> feature = std::shared_ptr<Feature2d>(new Feature2d(keypoint));
               feature->is_left_ = true;
               feature->set_frame_linked(sp_current_frame_);
        //        DLOG_INFO<< " current frame id : " << sp_current_frame_->id_<< std::endl;
        //        DLOG_INFO<< " feature->get_frame_linked()->id_ " <<feature->get_frame_linked()->id_ << std::endl;
               sp_current_frame_->vsp_left_feature_.push_back(feature);
               cv::circle(sp_current_frame_->left_image_, keypoint.pt, 3, cv::Scalar(255,0,0));
        //        cv::imshow("image",sp_current_frame_->left_image_);
        //        cv::waitKey(1);
        }
        DLOG_INFO << "detect_left_image_features: keypoints size() : " << keypoints.size() << std::endl;
        //  cv::imshow("rectangle frame", mask);
        //  cv::imshow("current frame", sp_current_frame_->left_image_);
        //  cv::waitKey();
        return keypoints.size();
}



int64_t Tracker::track_feature_in_right_image()
{
        std::vector<cv::Point2f> v_left_keypoints;
        std::vector<cv::Point2f> v_right_keypoints;

        if (sp_current_frame_->vsp_left_feature_.size() < sp_slam_config_->features_init_min_threshold)
        {
                DLOG_INFO <<  " sp_current_frame_  left keypoint is little, can't init normal " << std::endl;
                return 0;
        }
        // give a init position;
        for(auto& feature: sp_current_frame_->vsp_left_feature_)
        {
                // std::shared_ptr<Feature2d> new_feature = nullptr;
                v_left_keypoints.emplace_back(feature->position2d_.x(), feature->position2d_.y() );
                auto mappoint =  feature->get_mappoint3d_linked();
                if(mappoint)
                {
                        //for insert keyframe;
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
                                                              v_left_keypoints, v_right_keypoints, status, error, cv::Size(11, 11), 3,
                                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                                              cv::OPTFLOW_USE_INITIAL_FLOW); 
        for (size_t i = 0; i < status.size(); i++)
        {
                if (status[i])
                {
                        std::shared_ptr<Feature2d> feature 
                                = std::shared_ptr<Feature2d>(new Feature2d(Vec2(v_right_keypoints[i].x, v_right_keypoints[i].y)));
                        feature->is_left_ = false;
                        feature->set_frame_linked(sp_current_frame_);
                        // DLOG_INFO<< " current frame id : " << sp_current_frame_->id_<< " feature->get_frame_linked()->id_ " <<feature->get_frame_linked()->id_ << std::endl;
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
        int64_t counter  { 0 };
        int64_t bad_triangluated_point_num { 0 };
        //clear all mappoint;
        wp_map_.lock()->get_mappoints().clear();
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
                        bad_triangluated_point_num++;
                        continue;
                }
                //TODO(snowden)[high] : need use parameter rather than magic;
                if ((points_3d(2) > 300) || (points_3d(2) < 0)) 
                {
                        DLOG_INFO << "reject triangulate due distance great limit" << std::endl;
                        bad_triangluated_point_num++;
                        continue;
                }
                 counter++;
                //  DLOG_INFO << " ######### point3d: "  << points_3d << std::endl;
                 std::shared_ptr<Mappoint3d> new_mappoint = std::shared_ptr<Mappoint3d>(new Mappoint3d(sp_current_frame_->timestamp_, points_3d));
                 CHECK_NOTNULL(new_mappoint);
                 CHECK_NOTNULL(sp_current_frame_->vsp_left_feature_[i]);
                 CHECK_NOTNULL(sp_current_frame_->vsp_right_feature_[i]);
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
        DLOG_INFO << " bad_triangluated_point_num : " << bad_triangluated_point_num << std::endl;
        DLOG_INFO << "******************************************************* \n\r init map : total map point : " << counter << std::endl;
        return counter;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-29
 * @return the number of feature2d successfully tracked from last frame's left;;
 * @version 1.0
 */
int64_t Tracker::track_feature_in_current_image(bool& is_urgent_need_keyframe)
{
        std::vector<cv::Point2f> v_last_keypoints;
        std::vector<cv::Point2f> v_current_keypoints;
        if (sp_last_frame_->vsp_left_feature_.size() < sp_slam_config_->features_tracking_min_threshold)
        {
                DLOG_INFO <<  " last frame keypoint is little, can't track normal " << std::endl;
                return 0;
        }
        int64_t no_mappoint3d_linked_feature { 0 };
        for (auto& feature: sp_last_frame_->vsp_left_feature_)
        {
                // std::shared_ptr<Feature2d> new_feature = nullptr;
                if(nullptr == feature->get_mappoint3d_linked())
                {
                        no_mappoint3d_linked_feature++;
                        // continue;
                }
                v_last_keypoints.emplace_back(feature->position2d_.x(), feature->position2d_.y() );
                auto mappoint =  feature->get_mappoint3d_linked();
                if(mappoint)
                {
                        Vec2 project_position = CoordinateTransformWorldToImage(mappoint, sp_current_frame_->get_left_pose(),  
                                                                                                                                                      sp_camera_config_->K_left);
                        v_current_keypoints.emplace_back(project_position(0),project_position(1));
                }
                else
                {
                        v_current_keypoints.emplace_back(feature->position2d_.x(), feature->position2d_.y() );
                }                
        }
        //optical flow track;
        std::vector<uchar> status;
        cv::Mat error;
        int64_t good_point_counter = 0;
        cv::calcOpticalFlowPyrLK(sp_last_frame_->left_image_, sp_current_frame_->left_image_, 
                                                              v_last_keypoints, v_current_keypoints, status, error, cv::Size(11, 11), 5,
                                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                                              cv::OPTFLOW_USE_INITIAL_FLOW); 
        //count keypoint numbers in image left 1/3, mid 1/3, right 1/3;
        int64_t left_keypoint_num { 0 };
        int64_t right_keypoint_num { 0 };
        std::vector<size_t> vv_directation_angles_index[12];
        //statistic angle;
        for (size_t i  { 0 }; i < status.size(); i++)
        {
                if (status[i] && !(sp_last_frame_->vsp_left_feature_[i]->is_outline_))
                {       
                        float dx = v_current_keypoints[i].x - v_last_keypoints[i].x;
                        float dy = v_current_keypoints[i].y - v_last_keypoints[i].y;
                        // DLOG_INFO << "dx: " << dx << " dy : " << dy << std::endl;
                        // DLOG_INFO << "atan(2) " << atan(-0.2) * 180.0 / PI<< std::endl;
                        // DLOG_INFO << "atan(2) f" << atan(-0.2) * 180.0f / PI<< std::endl;
                        // float angle = atan(dy/dx) * 180.0f / PI;
                        // DLOG_INFO << "angle " << angle << std::endl;
                        int32_t index =((atan(dy/dx) * 180.0f / PI) + 180 )/30.f;
                        // DLOG_INFO << "index " << index << std::endl;
                        vv_directation_angles_index[index].push_back(i);;
                        // exit(0);
                }
        }
        //find the max angle;
        size_t max_cnt { 0 };
        size_t second_max_cnt { 0 };
        size_t max_index { 0 };
        size_t second_max_index { 0 };
        for(size_t i { 0 }; i < 12; i++)
        {
                DLOG_INFO << "angle : " << i * 30 << " ~ " << (i + 1) * 30 << " cnt : " << vv_directation_angles_index[i].size() << std::endl;
                if(vv_directation_angles_index[i].size() > max_cnt)
                {
                        second_max_cnt = max_cnt;
                        second_max_index = max_index;
                        max_cnt = vv_directation_angles_index[i].size();
                        max_index = i;
                }
                else if(vv_directation_angles_index[i].size() > second_max_cnt)
                {
                        second_max_cnt = vv_directation_angles_index[i].size();
                        second_max_index = i;
                }
        }
        DLOG_INFO << "max_cnt " <<max_cnt << "  max_index " << max_index << std::endl;
        DLOG_INFO << "second_max_cnt " << second_max_cnt<< " second_max_index " <<second_max_index << std::endl;
        //if max and second_max direction cnt is closed , push second_max to max;
        if(max_cnt - second_max_cnt < 30)
        {
                for(auto& i : vv_directation_angles_index[second_max_index])
                {
                        vv_directation_angles_index[max_index].emplace_back(i);
                }
        }
        for (auto& i : vv_directation_angles_index[max_index])
        {
                if(v_current_keypoints[i].x < sp_current_frame_->left_image_.cols / 2) 
                {
                        left_keypoint_num++;
                }
                else
                {
                        right_keypoint_num++;
                }
                std::shared_ptr<Feature2d> feature 
                        = std::shared_ptr<Feature2d>(new Feature2d(Vec2(v_current_keypoints[i].x, v_current_keypoints[i].y)));
                CHECK_NOTNULL(feature);
                feature->is_left_ = true;
                feature->set_frame_linked(sp_current_frame_);
                
                std::shared_ptr<Mappoint3d>mappoint_linked = sp_last_frame_->vsp_left_feature_[i]->get_mappoint3d_linked();
                if(nullptr != mappoint_linked)
                {
                        feature->set_mappoint3d_linked(mappoint_linked);
                        feature->get_mappoint3d_linked()->vwp_observers_.push_back(feature);
                }
                sp_current_frame_->vsp_left_feature_.push_back(feature);
                good_point_counter++;
                cv::circle(sp_current_frame_->left_image_, v_current_keypoints[i], 3, cv::Scalar(0,255,0));
                cv::line(sp_current_frame_->left_image_, v_current_keypoints[i],  v_last_keypoints[i], cv::Scalar(111, 111, 111));
        }
        // for (size_t i  { 0 }; i < status.size(); i++)
        // {
        //         if (status[i] && !(sp_last_frame_->vsp_left_feature_[i]->is_outline_))
        //         {
        //                 // DLOG_INFO << "v_current_keypoints[i].x " << v_current_keypoints[i].x << std::endl;
        //                 //count feature in half image of left and right;
        //                 if(v_current_keypoints[i].x < sp_current_frame_->left_image_.cols / 2) 
        //                 {
        //                         left_keypoint_num++;
        //                 }
        //                 else
        //                 {
        //                         right_keypoint_num++;
        //                 }
        //                 std::shared_ptr<Feature2d> feature 
        //                         = std::shared_ptr<Feature2d>(new Feature2d(Vec2(v_current_keypoints[i].x, v_current_keypoints[i].y)));
        //                 CHECK_NOTNULL(feature);
        //                 feature->is_left_ = true;
        //                 feature->set_frame_linked(sp_current_frame_);
                        
        //                 std::shared_ptr<Mappoint3d>mappoint_linked = sp_last_frame_->vsp_left_feature_[i]->get_mappoint3d_linked();
        //                 if(nullptr != mappoint_linked)
        //                 {
        //                         feature->set_mappoint3d_linked(mappoint_linked);
        //                         feature->get_mappoint3d_linked()->vwp_observers_.push_back(feature);
        //                 }
        //                 sp_current_frame_->vsp_left_feature_.push_back(feature);
        //                 good_point_counter++;
        //                 cv::circle(sp_current_frame_->left_image_, v_current_keypoints[i], 3, cv::Scalar(0,255,0));
        //                 cv::line(sp_current_frame_->left_image_, v_current_keypoints[i],  v_last_keypoints[i], cv::Scalar(111, 111, 111));
        //         }
        // }
        double_t max = left_keypoint_num > right_keypoint_num ? left_keypoint_num : right_keypoint_num;
        double_t min = left_keypoint_num < right_keypoint_num ? left_keypoint_num : right_keypoint_num;
        if((max - min ) / good_point_counter > 0.25)
        {
                is_urgent_need_keyframe = true;
        }
        else
        {
                is_urgent_need_keyframe = false;
        }
        cv::imshow("image",sp_current_frame_->left_image_);
        cv::waitKey(1);
        DLOG_INFO << " last frmae  : no_mappoint3d_linked_feature : " << no_mappoint3d_linked_feature << std::endl;
        DLOG_INFO << " not be tracked in current image : mappoints num : " << status.size() -  good_point_counter << std::endl;
        return good_point_counter;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-31
 ** @return number of inlier point ;
 * @version 1.0
 */
int64_t Tracker::estimate_current_pose()
{
        g2o::SparseOptimizer optimizer;

        //set Algorithm;
        typedef g2o::BlockSolver_6_3  BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new   g2o::OptimizationAlgorithmLevenberg(
                                                        g2o::make_unique<BlockSolverType>(
                                                                g2o::make_unique<LinearSolverType>()
                                                        )
                                                );
        optimizer.setAlgorithm(solver);

        //set Vertex;
        VertexPose* pose_vertex = new VertexPose();
        pose_vertex->setId(0);
        SE3 raw_pose =  sp_current_frame_->get_left_pose();
        pose_vertex->setEstimate(raw_pose);
        optimizer.addVertex(pose_vertex);

        //set edge;
        Mat33 K { sp_camera_config_->K_left };
        int64_t index { pose_vertex->id() + 1 };
        std::vector<std::shared_ptr<Feature2d>> vsp_feature2ds;
        std::vector<UnaryEdgePose*> vp_egdes;
        int64_t no_mappoint3d_linked_feature2d_num { 0 } ;
        int64_t too_close_mappoint_not_join_estimate_num { 0 };
        for (auto feature2d : sp_current_frame_->vsp_left_feature_)
        {
                if (feature2d->get_mappoint3d_linked())
                {
                        if(feature2d->get_mappoint3d_linked()->get_position3d()(2) < 2)
                        {
                                too_close_mappoint_not_join_estimate_num++;
                                continue;
                        }
                        //save valid feature for mark is outlier 
                        vsp_feature2ds.push_back(feature2d);
                        UnaryEdgePose * new_edge = new UnaryEdgePose(feature2d->get_mappoint3d_linked()->get_position3d() , K);
                        new_edge->setId(index);
                        new_edge->setVertex(0,pose_vertex);
                        new_edge->setMeasurement(feature2d->position2d_);
                        new_edge->setInformation(Eigen::Matrix2d::Identity());
                        new_edge->setRobustKernel(new g2o::RobustKernelHuber);
                        optimizer.addEdge(new_edge);
                        index++;
                        //save the edge to decide if a feature is outlier after optimize;
                        vp_egdes.push_back(new_edge);
                }
                else
                {
                        no_mappoint3d_linked_feature2d_num++;
                }
        }

        double_t outlier_cnt { 0.0 };
        double_t inlier_cnt { 0.0 };
        for(int k { 0 }; k < 1; k++)
        {
                //use original pose in every turn
                pose_vertex->setEstimate(raw_pose);
                //init optimizer and run optimizer x times;
                optimizer.initializeOptimization();
                optimizer.optimize(10);
                //mark outliers , use Chi-Squared Test;
                outlier_cnt = 0.0;
                inlier_cnt = 0.0;
                double_t kChi2Threshold { 5.991 };
                for (size_t i { 0 }; i < vp_egdes.size(); i++)
                {
                        // if (vsp_feature2ds[i]->is_outline_)
                        // {
                        //         vp_egdes[i]->computeError();
                        // }
                        if(vp_egdes[i]->chi2() > kChi2Threshold)
                        {
                                vsp_feature2ds[i]->is_outline_ = true;
                                vp_egdes[i]->setLevel(1);
                                outlier_cnt++;
                        }
                        else
                        {
                                vsp_feature2ds[i]->is_outline_ = false;
                                vp_egdes[i]->setLevel(0);
                                inlier_cnt++;
                        }
                }
                if(inlier_cnt / (inlier_cnt + outlier_cnt) > 0.5)
                {
                        break;
                }
                // else
                // {
                //         kChi2Threshold *= 1;
                // }
        }
        DLOG_INFO << "total optimized element = " << index << std::endl;
        DLOG_INFO << "too_close_mappoint_not_join_estimate_num : " << too_close_mappoint_not_join_estimate_num << std::endl;
        DLOG_INFO <<  "  no_mappoint3d_linked_feature2d_num : " << no_mappoint3d_linked_feature2d_num << std::endl;
        DLOG_INFO <<  "  outlier / feature size : " << outlier_cnt << " / " << vsp_feature2ds.size() << std::endl;
        sp_current_frame_->set_left_pose(pose_vertex->estimate());
        sp_current_frame_->set_right_pose( SE3(SO3(),sp_camera_config_->base_line) * pose_vertex->estimate() );

        for(auto &feature : vsp_feature2ds)
        {
                if(feature->is_outline_)
                {
                        feature->set_mappoint3d_linked(nullptr);
                        //TODO(snowden) : is_outline_ set to false, maybe can be used by other frame in local optimizer; 
                        //but you can try to delete it to make a comparrson; 
                        feature->is_outline_ = true;
                }
        }
        sp_current_frame_->linked_mappoint3d_nums =  vsp_feature2ds.size() - outlier_cnt;
        return sp_current_frame_->linked_mappoint3d_nums;
} //int64_t Tracker::estimate_current_pose()



/**
 * @brief  
 * @author snowden
 * @date 2021-07-23
 ** @return the number of mappoint successfully triangulated;
 * @version 1.0
 */
int64_t Tracker::insert_mappoints()
{
        /**access each element, if right feature not null , change coordinate to normalized plane ,  construct a eq to solve , then 
         *  check if meet the requirement,  then push back 3d point to vector, 
         */
        Vec3 normalized_left_point;
        Vec3 normalized_right_point;
        Vec3 points_3d;
        int64_t counter = 0;
        int64_t bad_triangluated_point_num { 0 };
        for (int i = 0; i < sp_current_frame_->vsp_left_feature_.size(); i++)
        {
                //pass feature's not matched feature and have linked other mappoint ;
                if(nullptr ==  sp_current_frame_->vsp_right_feature_[i]) { continue; }
                if(nullptr != sp_current_frame_->vsp_left_feature_[i]->get_mappoint3d_linked()) { continue; }
                normalized_left_point = CoordinateTransformImageToNormalizedPlane(sp_current_frame_->vsp_left_feature_[i]->position2d_, 
                                                                                                                                                                      sp_camera_config_->K_left);
                normalized_right_point = CoordinateTransformImageToNormalizedPlane(sp_current_frame_->vsp_right_feature_[i]->position2d_, 
                                                                                                                                                                          sp_camera_config_->K_right);
                if(!TriangulateNormalizedPoint(normalized_left_point, normalized_right_point, 
                                                                                  sp_current_frame_->get_left_pose(), sp_current_frame_->get_right_pose(), points_3d ))
                {       
                        bad_triangluated_point_num++;
                        continue;
                }
                //TODO(snowden)[high] : need use parameter rather than magic;
                if ((points_3d(2) > 300) || (points_3d(2) < 0))
                {
                        // DLOG_INFO << "Reject triangulate due to distance great limitation , distance : " << points_3d(2) << std::endl;
                        bad_triangluated_point_num++;
                        continue;
                }
                 counter++;
                 //change reference coordinate to world coor;
                 Vec3 world_points_3d = sp_current_frame_->get_left_pose().inverse() * points_3d;
                //  DLOG_INFO << " #########camera point3d: "  << std::endl <<  points_3d << std::endl;
                //  DLOG_INFO << " #########world point3d: "  << std::endl  << world_points_3d << std::endl;
                //  DLOG_INFO << "left pose : " << std::endl <<  sp_current_frame_->get_left_pose().matrix() << std::endl;
                //  DLOG_INFO << "right pose : " << std::endl <<  sp_current_frame_->get_right_pose().matrix() << std::endl;
                //  DLOG_INFO << "baseline " << std::endl << sp_camera_config_->base_line << std::endl;
                 std::shared_ptr<Mappoint3d> new_mappoint = std::shared_ptr<Mappoint3d>(new Mappoint3d(sp_current_frame_->timestamp_, points_3d));
                 CHECK_NOTNULL(new_mappoint);
                 CHECK_NOTNULL(sp_current_frame_->vsp_left_feature_[i]);
                 CHECK_NOTNULL(sp_current_frame_->vsp_right_feature_[i]);
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
        DLOG_INFO << " bad_triangluated_point_num : " << bad_triangluated_point_num << std::endl;
        DLOG_INFO << " insert_mappoints : " << counter << std::endl;
        return counter;
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam