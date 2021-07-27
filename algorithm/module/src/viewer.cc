#include "algorithm/module/include/viewer.h"

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

/**
 * @brief 
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
Viewer:: Viewer(std::weak_ptr<Map> wp_map, std::shared_ptr<Tracker> sp_tracker, std::weak_ptr<Optimizer> wp_optimizer)
        :wp_map_(wp_map), sp_tracker_(sp_tracker), wp_optimizer_(wp_optimizer)
{
        is_running_.store(true);
        viewer_thread_ = std::thread(std::bind(&Viewer::viewer_loop,this));
}



/**
 * @brief  join view thread;
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::stop()
{
        //TODO(snowden) : directly detach thread , if this thread will be terminated ? 
        viewer_thread_.detach(); 
}


/**
 * @brief  viewer thread;
 * @author snowden
 * @date 2021-07-18
 * @version 1.0
 */
void Viewer::viewer_loop()
{
        pangolin::CreateWindowAndBind("OP_SLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState camera(
                pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
                );
        pangolin::Handler3D*  handler3d = new pangolin::Handler3D(camera);
        pangolin::View& displayer =
                pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(handler3d);
        while (is_running_.load())
        {
                // DLOG_INFO << "viewer is running " << std::endl;
                // SHOW_FUNCTION_INFO
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                displayer.Activate(camera);

                if(wait_update_map_notify())
                {
                        update_viewer(camera,displayer);
                }
                
        }
        DLOG_INFO << " viewer loop is finished " << std::endl;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-21
 * @version 1.0
 *  @note wait notify from front_end thread and back_end thread;
 */
bool Viewer::wait_update_map_notify()
{
        //等待地图更新的通知；
        DLOG_INFO << " viewer wait update map notify " << std::endl;
        wp_map_.lock()->condition_var_is_map_updated_.wait(wp_map_.lock()->data_lock_);
        if(!is_running_.load())
        {
                return false;
        }
        DLOG_INFO << " viewer received map update " << std::endl;
        return true;
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-26
 * @version 1.0
 */
bool Viewer::update_viewer(pangolin::OpenGlRenderState& camera, pangolin::View& displayer)
{

        DLOG_INFO << " updated viewer " << std::endl;
        draw_frame(sp_tracker_->sp_current_frame_->get_left_pose(), kGreenColor_);
        draw_mappoints(KRedColor_);
        follow_frame(sp_tracker_->sp_current_frame_->get_left_pose(), camera);
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-26
 * @version 1.0
 * @note : reference from gaoxiang'code, and make some modifiy;
 */
bool Viewer::draw_frame(const SE3 pose, const float color[3])
{
 
        SE3 Twc = pose.inverse();
        DLOG_INFO << " draw frame *********************** " << std::endl;
        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        glColor3f(0.5, 0.5, 0);

        glLineWidth(line_width);
        glBegin(GL_LINES);
    
        glVertex3f(0, 0, 0);
        glVertex3f(depth * (0 - cx) / fx, depth * (0 - cy) / fy, depth);
    
        glVertex3f(0, 0, 0);
        glVertex3f(depth * (0 - cx) / fx, depth * (height - 1 - cy) / fy, depth);
    
        glVertex3f(0, 0, 0);
        glVertex3f(depth * (width - 1 - cx) / fx, depth * (height - 1 - cy) / fy, depth);

        glVertex3f(0, 0, 0);
        glVertex3f(depth * (width - 1 - cx) / fx, depth * (0 - cy) / fy, depth);

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
        glColor3f(color[0], color[1], color[2]);

        glVertex3f(depth * (width - 1 - cx) / fx, depth * (0 - cy) / fy, depth);
        glVertex3f(depth * (width - 1 - cx) / fx, depth * (height - 1 - cy) / fy, depth);

        glVertex3f(depth * (width - 1 - cx) / fx, depth * (height - 1 - cy) / fy, depth);
        glVertex3f(depth * (0 - cx) / fx, depth * (height - 1 - cy) / fy, depth);

        glVertex3f(depth * (0 - cx) / fx, depth * (height - 1 - cy) / fy, depth);
        glVertex3f(depth * (0 - cx) / fx, depth * (0 - cy) / fy, depth);

        glVertex3f(depth * (0 - cx) / fx, depth * (0 - cy) / fy, depth);
        glVertex3f(depth * (width - 1 - cx) / fx, depth * (0 - cy) / fy, depth);

        glEnd();
        glPopMatrix();
        pangolin::FinishFrame();
}


/**
 * @brief  
 * @author snowden
 * @date 2021-07-27
 * @version 1.0
 */
bool Viewer::draw_mappoints(const float color[3])
{
        glPointSize(5);
        glBegin(GL_POINTS);
        DLOG_INFO << "############ viewer : vsp mappoint size : " << vsp_mappoint_.size() << std::endl;
        for (auto& mappoint : vsp_mappoint_) 
        {
                auto pos = mappoint->position3d_;
                glColor3f(color[0], color[1], color[2]);
                glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
        pangolin::FinishFrame();
}



/**
 * @brief  
 * @author snowden
 * @date 2021-07-26
 * @version 1.0
 * @note : reference from gaoxiang'code, and make some modifiy;
 */
void Viewer::follow_frame(const SE3 pose, pangolin::OpenGlRenderState& camera)
{
        pangolin::OpenGlMatrix m(pose.inverse().matrix());
        camera.Follow(m, true);
}

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam