#include "algorithm/module/include/optimizer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Optimizer::Optimizer( std::weak_ptr<Map> map, const std::shared_ptr<SystemConfig>  sp_slam_config, 
                                             const std::shared_ptr<CameraConfig> sp_camera_config)
        : wp_map_(map), sp_slam_config_(sp_slam_config), sp_camera_config_(sp_camera_config)
{
        is_running_.store(true);
        back_end_thread_ = std::thread(std::bind(&Optimizer::back_end_loop,this));
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
void Optimizer::stop()
{
        //TODO(snowden) : directly detach thread , if this thread will be terminated ? 
        back_end_thread_.detach();
}


/**
 * @brief set back end status in some constraint;
 * @return true if set successfully, otherwise return false;
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
bool Optimizer::set_back_end_status(const BackEndStatus &new_status)
{
        bool is_allowed_change { false };
        switch (new_status)
        {
                case BackEndStatus::READY:
                {
                        if (BackEndStatus::UNKNOW == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::INIT:
                {
                        if (BackEndStatus::READY == enum_back_end_status_ || BackEndStatus::RESET == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::IDLE:
                {
                        if (BackEndStatus::INIT == enum_back_end_status_ || BackEndStatus::FINISHED == enum_back_end_status_ || 
                             BackEndStatus::OPTIMIZING == enum_back_end_status_ )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::OPTIMIZING:
                {
                        if (BackEndStatus::IDLE == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                } 
                case BackEndStatus::FINISHED:
                {
                        if (BackEndStatus::OPTIMIZING == enum_back_end_status_)
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                case BackEndStatus::RESET:
                {
                        if (BackEndStatus::OPTIMIZING == enum_back_end_status_ )
                        {
                                is_allowed_change = true;
                        }
                        break;
                }
                default:
                {
                        LOG_ERROR << " set_back_end_status: give a unkonw new status : " << static_cast<int64_t>(new_status) << std::endl;
                        break;
                }
        }
        if(is_allowed_change)
        {
                enum_back_end_status_ = new_status;
        }
        else
        {
                LOG_ERROR << " set_back_end_status error : from " << static_cast<int64_t>(enum_back_end_status_) \
                << " to " << static_cast<int64_t>(new_status) << " is not allowed " << std::endl;
        }
        return is_allowed_change;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-19
 * @version 1.0
 */
void Optimizer::back_end_loop()
{
        while (is_running_.load())
        {
                switch (get_back_end_status())
                {
                        case BackEndStatus::READY:
                        {
                                DLOG_INFO << " BackEndStatus::READY " << std::endl;
                                set_back_end_status(BackEndStatus::INIT);
                                break;
                        }
                        case BackEndStatus::INIT:
                        {
                                DLOG_INFO << " BackEndStatus::INIT " << std::endl;
                                if (init_back_end())
                                {
                                        set_back_end_status(BackEndStatus::IDLE);
                                }
                                else
                                {
                                        set_back_end_status(BackEndStatus::RESET);
                                }
                                break;
                        }
                        case BackEndStatus::IDLE:
                        {
                                DLOG_INFO << " BackEndStatus::IDLE " << std::endl;
                                if(wait_update_map_notify())
                                {
                                        set_back_end_status(BackEndStatus::OPTIMIZING);
                                }
                                break;
                        }
                        case BackEndStatus::OPTIMIZING:
                        {
                                DLOG_INFO << " BackEndStatus::OPTIMIZING " << std::endl;
                                BackEndStatus status = backend_optimize();
                                set_back_end_status(status);
                                break;
                        }
                        case BackEndStatus::FINISHED:
                        {
                                DLOG_INFO << " BackEndStatus::FINISHED " << std::endl;
                                notify_all_updated_map();
                                set_back_end_status(BackEndStatus::IDLE);
                                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                is_running_.store(false);
                                break;
                        }
                        case BackEndStatus::RESET:
                        {
                                DLOG_INFO << " BackEndStatus::RESET " << std::endl;
                                set_back_end_status(BackEndStatus::INIT);
                                break;
                        }
                        case BackEndStatus::UNKNOW:
                        {
                                DLOG_INFO << " BackEndStatus::UNKNOW " << std::endl;
                                set_back_end_status(BackEndStatus::READY);
                                break;
                        }
                        default:
                        {
                                LOG_ERROR << " get_back_end_status :  error status : " << static_cast<int64_t>(get_back_end_status()) << std::endl;
                                break;
                        }
                } //switch (get_back_end_status())
        } //while (is_running_.load())
        DLOG_INFO << " back end will be terminated " << std::endl;
} //void Optimizer::back_end_loop()


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
bool Optimizer::init_back_end()
{
        //TODO(snowden) init;
        return true;
}


/**
 * @brief 
 * @return true if received notify;
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note wait front_end thread notify map have updated ;
 */
//TODO(snowden) : need to change the name of this function , no just wait update_map, but also end optimizer;
bool Optimizer::wait_update_map_notify()
{
        DLOG_INFO << "optimizer wait update map notify " << std::endl;
        wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
        if(!is_running_.load())
        {
                return false;
        }
        DLOG_INFO << " optimizer received map update " << std::endl;
        return true;
}


/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 * @note while optimized map, notify all map have updated, mainly for viewer thread;
 */
void Optimizer::notify_all_updated_map()
{
        wp_map_.lock()->condition_var_is_map_updated_.notify_all();
        DLOG_INFO << " optimizer notify_all_updated_map() " << std::endl;
}


//TODO(snowden)[high]:need to review;
/**
 * @brief 
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 */
Optimizer::BackEndStatus Optimizer::backend_optimize()
{
        g2o::SparseOptimizer optimizer;

        //set Algorithm;
                //6 _PoseDim, 3 _LandmarkDim
        // typedef g2o::BlockSolver_6_3  BlockSolverType;
        typedef g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType> LinearSolverType;
        auto solver = new   g2o::OptimizationAlgorithmLevenberg(
                                                        g2o::make_unique<g2o::BlockSolverX>(
                                                                g2o::make_unique<LinearSolverType>()
                                                        )
                                                );
        optimizer.setAlgorithm(solver);

        //set Vertex;
        int64_t vertex_index { 0 };
                //vertex_pose;
        int64_t actived_frame_size = wp_map_.lock()->get_actived_keyframes().size();
        if(actived_frame_size < 2)
        {
                DLOG_INFO << "optimizer: actived_frame_size < 2 "<< std::endl;
                return BackEndStatus::IDLE; 
        }
        std::deque<std::shared_ptr<KeyFrame>> dsp_actived_keyframes = wp_map_.lock()->get_actived_keyframes();
        std::vector<VertexPose*> vsp_pose_vertex;
        for(int i { 0 }; i < actived_frame_size; i++)
        {
                //add left pose vertex
               VertexPose* new_left_pose_vertex = new VertexPose();
                new_left_pose_vertex->setId(vertex_index);
                vertex_index++;
                new_left_pose_vertex->setEstimate(dsp_actived_keyframes[i]->sp_frame_->get_left_pose());
                        //the first frame is fixed;
                if(0 == dsp_actived_keyframes[i]->sp_frame_->id_)
                {
                        new_left_pose_vertex->setFixed(true);
                }
                optimizer.addVertex(new_left_pose_vertex);
                vsp_pose_vertex.push_back(new_left_pose_vertex);

                //add right pose vertex
               VertexPose* new_right_pose_vertex = new VertexPose();
                new_right_pose_vertex->setId(vertex_index);
                vertex_index++;
                new_right_pose_vertex->setEstimate(dsp_actived_keyframes[i]->sp_frame_->get_right_pose());
                        //the first frame is fixed;
                if(0 == dsp_actived_keyframes[i]->sp_frame_->id_)
                {
                        new_right_pose_vertex->setFixed(true);
                }
                optimizer.addVertex(new_right_pose_vertex);
                vsp_pose_vertex.push_back(new_right_pose_vertex);
        }
        DLOG_INFO << "optimizer : vsp_pose_vertex size : " << vsp_pose_vertex.size() << std::endl;
                //vertex_mappoint;
        // std::deque<std::shared_ptr<Mappoint3d>> wp_map_.lock()->get_actived_mappoints() = wp_map_.lock()->get_actived_mappoints(); 
        std::vector<VertexMappoint*> vsp_mappoint_vertex;
        int64_t actived_mappoint_size = wp_map_.lock()->get_actived_mappoints().size();
        for(int j { 0 }; j < actived_mappoint_size; j++ )
        {
                VertexMappoint* new_mappoint_vertex = new VertexMappoint();
                new_mappoint_vertex->setId(vertex_index);
                vertex_index++;
                new_mappoint_vertex->setEstimate(wp_map_.lock()->get_actived_mappoints()[j]->get_position3d());
                optimizer.addVertex(new_mappoint_vertex);
                vsp_mappoint_vertex.push_back(new_mappoint_vertex);
        }
        DLOG_INFO << " optimizer: actived_mappoint_size " << actived_mappoint_size << std::endl;
        //set Edge;
                //edge_index is after vertex_index;
        int64_t edge_index { vertex_index };
        EdgePoseMappoint*  new_edge { nullptr } ;
        Mat33 K { sp_camera_config_->K_left } ;
        const double_t kChi2Threshold { 5.991 };
        std::vector<EdgePoseMappoint*> vsp_pose_mappoint_edge;
        std::map<EdgePoseMappoint*, std::shared_ptr<Feature2d>> msp_edge_feature2d;
        for(int n { 0 }; n < actived_mappoint_size; n++)
        {
                for( auto observer : wp_map_.lock()->get_actived_mappoints()[n]->vwp_observers_)
                {
                        for(int m { 0 }; m < actived_frame_size; m++)
                        {
                                if( nullptr == observer.lock())
                                {
                                        DLOG_INFO << "observer.lock()->get_frame_linked()  is null " << std::endl;
                                        continue;
                                }
                                if(observer.lock()->get_frame_linked().lock()->id_ == dsp_actived_keyframes[m]->sp_frame_->id_)
                                {
                                        //left pose -> mappoint;
                                        SE3 pose;
                                        if(observer.lock()->is_left_)
                                        {       
                                                pose = dsp_actived_keyframes[m]->sp_frame_->get_left_pose();
                                        }
                                        else
                                        {
                                                pose = dsp_actived_keyframes[m]->sp_frame_->get_right_pose();
                                        }
                                        new_edge = new EdgePoseMappoint(K, pose);
                                        new_edge->setId(edge_index);
                                        edge_index++;
                                        if(observer.lock()->is_left_)
                                        {
                                                new_edge->setVertex(0, vsp_pose_vertex[2 * m]);
                                        }
                                        else
                                        {
                                                new_edge->setVertex(0, vsp_pose_vertex[2 * m + 1]);
                                        }
                                        new_edge->setVertex(1, vsp_mappoint_vertex[n]);
                                        new_edge->setMeasurement(observer.lock()->position2d_);
                                        new_edge->setInformation(Eigen::Matrix2d::Identity());
                                        auto rk_hb = new g2o::RobustKernelHuber();
                                        rk_hb->setDelta(kChi2Threshold);
                                        new_edge->setRobustKernel(rk_hb);
                                        optimizer.addEdge(new_edge);
                                        vsp_pose_mappoint_edge.push_back(new_edge);
                                        msp_edge_feature2d.emplace(new_edge,observer.lock());
                                        //one observer  only correspond  one frame; if find , then break;
                                        break;
                                }
                        }
                }
        }
        DLOG_INFO << " vsp_pose_vertex.size() : " << vsp_pose_vertex.size()  << std::endl;
        DLOG_INFO << " vsp_mappoint_vertex.size() : " << vsp_mappoint_vertex.size() << std::endl;
        DLOG_INFO << " vsp_pose_mappoint_edge.size() : " << vsp_pose_mappoint_edge.size() << std::endl;

        //init optimizer and run optimizer 
                /**
                 * setLevel(int ) is useful when you call optimizer.initializeOptimization(int ). 
                 * If you assign initializeOptimization(0), the optimizer will include all edges 
                 * up to level 0 in the optimization, and edges set to level >=1 will not be included.
                 * default level = 0
                 */ 
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        //set new pose and mappoint position;
        for(int i { 0 }; i < dsp_actived_keyframes.size(); i++)
        {
                dsp_actived_keyframes[i]->sp_frame_->set_left_pose(vsp_pose_vertex[2 * i]->estimate());
                dsp_actived_keyframes[i]->sp_frame_->set_right_pose(SE3(vsp_pose_vertex[2 * i + 1]->estimate().so3(),sp_camera_config_->base_line));
        }
        for(int j { 0 }; j < wp_map_.lock()->get_actived_mappoints().size(); j++)
        {
                wp_map_.lock()->get_actived_mappoints()[j]->set_position3d(vsp_mappoint_vertex[j]->estimate());
        }
        
        //mark and delelte outlier;
        
        int64_t outlier_cnt { 0 };
        int64_t inlier_cnt { 0 };
        double_t temp_chi2_threshold { kChi2Threshold };
                /**
                 * we hope the number of inlier is great half of total, if number of inlier is less than half, 
                 * than change the threshold;
                 */ 
        for(int k { 0 }; k < 3; k++)
        {
                outlier_cnt = 0;
                inlier_cnt = 0;
                for (size_t i { 0 }; i < vsp_pose_mappoint_edge.size(); i++)
                {
                        if (vsp_pose_mappoint_edge[i]->chi2() > temp_chi2_threshold)
                        {
                                outlier_cnt++;
                        }
                        else
                        {
                                inlier_cnt++;
                        }
                }
                if(static_cast<double_t>(inlier_cnt)/ static_cast<double_t>(inlier_cnt + outlier_cnt) > 0.5)
                {
                        break;
                }
                else
                {
                        temp_chi2_threshold *=  1.5;
                }
        }
        for(auto x : msp_edge_feature2d)
        {
                //if x is outlier, delete link fromm mappoint -> feature, and link from feature -> mappoint;
                if(x.first->chi2() > temp_chi2_threshold)
                {
                        x.second->is_outline_ = true;
                        //delete link from mappoint -> feature;
                                //TODO(snowden)[high]: why  we get nullptr ? 
                        if (nullptr ==  x.second->get_mappoint3d_linked())
                        {
                                DLOG_INFO << "x.second->get_mappoint3d_linked()  is null " << std::endl;
                                continue;
                        }
                        DLOG_INFO << " need delete mappoint id : " <<  x.second->get_mappoint3d_linked()->id_ << std::endl;  //->vwp_observers_[0].lock()->id_ << std::endl;;
                        for(int i { 0 }; i < x.second->get_mappoint3d_linked()->vwp_observers_.size(); i++)
                        {
                                //TODO(snowden)[high]: why  we get nullptr ? 
                                if (nullptr ==  x.second->get_mappoint3d_linked()->vwp_observers_[i].lock())
                                {
                                        DLOG_INFO << "x.second->get_mappoint3d_linked()->vwp_observers_[i]  is null " << std::endl;
                                        continue;
                                }
                                if (x.second->get_mappoint3d_linked()->vwp_observers_[i].lock()->id_ == x.second->id_)
                                {
                                        x.second->get_mappoint3d_linked()->vwp_observers_[i] = std::shared_ptr<Feature2d>(nullptr);
                                        break;
                                }
                        }
                        //delete link from feature -> mappoint;
                        x.second->set_mappoint3d_linked(nullptr);
                }
                else
                {
                        x.second->is_outline_ = false;
                }
        }
        DLOG_INFO << "backend optimize finished , Outlier / Inlier : " <<  outlier_cnt << " / " << inlier_cnt << std::endl;

        return BackEndStatus::IDLE; 
}


} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam