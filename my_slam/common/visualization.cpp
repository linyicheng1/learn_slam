#include "visualization.h"

namespace my_slam
{
    visualization::visualization(/* args */)
    {
        delay_ = 100000;
        pthread_create(&thread_, NULL, pthread_fun, (void *) this);
    }
    void visualization::draw_trajectory()
    {
        for (size_t i = 0; i < pos_.size(); i++) 
        {
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = pos_[i], p2 = pos_[i + 1];
            glVertex3f(p1[0], p1[1], p1[2]);
            glVertex3f(p2[0], p2[1], p2[2]);
            glEnd();
        }
    }
    void visualization::draw_pose()
    {
        // not all 
        for(size_t i=0;i<q_.size();i+=20)
        {
            // 画每个位姿的三个坐标轴
            Eigen::Vector3f Ow = pos_[i];
            q_[i].normalized();
            Eigen::Vector3f Xw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(1, 0, 0)) + Ow;
            Eigen::Vector3f Yw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(0, 1, 0)) + Ow;
            Eigen::Vector3f Zw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(0, 0, 1)) + Ow;

            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3f(Ow[0], Ow[1], Ow[2]);
            glVertex3f(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3f(Ow[0], Ow[1], Ow[2]);
            glVertex3f(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3f(Ow[0], Ow[1], Ow[2]);
            glVertex3f(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
    }
    void visualization::process()
    {
        draw_trajectory();
        draw_pose();
    }

    visualization::~visualization()
    {
    }

    void* visualization::pthread_fun(void* __this)
    {
        auto * _this =(thread *)__this;
        pangolin::CreateWindowAndBind("Main",640,480);
        glEnable(GL_DEPTH_TEST);

        // Define Projection and initial ModelView matrix
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
        );

        // Create Interactive View in window
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);
        while (true)
        {
            try
            {
                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);

                // draw 3d view 
                _this->process();

                pangolin::FinishFrame();
            }
            catch (...)
            {
                break;
            }
            usleep(_this->delay_);
        }
    }
}
