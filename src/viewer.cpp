#include "myslam/viewer.h"

#include "myslam/frame.h"
#include "myslam/feature.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/config.h"


namespace myslam{

// --------------------------------------------------------------
    Viewer::Viewer(){


        thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    }


// --------------------------------------------------------------

    void Viewer::Close(){
        viewer_running_ = false;
        thread_.join();
    }

// --------------------------------------------------------------
    void Viewer::ThreadLoop(){
        std::cout << std::endl << "-------" << std::endl;
        std::cout << "Viewer Thread Loop works ..." << std::endl;

        pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);



        pangolin::OpenGlRenderState vis_camera(
                pangolin::ProjectionMatrix(1024, 768, 2000, 2000, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0,-500,-0.1, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
                pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(vis_camera));

        bool bFollow = true;

        while(!pangolin::ShouldQuit() && viewer_running_){

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            std::unique_lock<std::mutex> lock(viewer_data_);
            if(current_frame_){
                if (menuFollowCamera && bFollow){
                    FollowCurrentFrame(vis_camera);
                }else if(!menuFollowCamera && bFollow){
                    bFollow = false;
                }else if(menuFollowCamera && !bFollow){
                    FollowCurrentFrame(vis_camera);
                    vis_camera.SetModelViewMatrix(
                            pangolin::ModelViewLookAt(0,-500,-0.1, 0,0,0,0.0,-1.0, 0.0));
                    bFollow = true;
                }
                cv::Mat img = PlotFrameImage();
                cv::imshow("frame", img);
                cv::waitKey(1);
            }

            vis_display.Activate(vis_camera);
            if(current_frame_){
                DrawFrame(current_frame_, green);
            }
            if (map_){
                DrawKFsAndMPs(menuShowKeyFrames, menuShowPoints);
            }

            pangolin::FinishFrame();
            usleep(1000);
        }

        LOG(INFO)  << "Stop Viewer";
    }

// --------------------------------------------------------------
    void Viewer::AddCurrentFrame(Frame::Ptr currentFrame){
        std::unique_lock<std::mutex> lock(viewer_data_);
        current_frame_ = currentFrame;
    }

// --------------------------------------------------------------

    cv::Mat Viewer::PlotFrameImage(){
        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
        for (size_t i = 0, N = current_frame_->features_left_.size(); i < N; ++i){
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0,250,0), 2);
        }
        return img_out;
    }



// --------------------------------------------------------------

// void Viewer::UpdateMap(){
//     std::unique_lock<std::mutex> lck(_mmutexViewerData);
//     assert(_mpMap != nullptr);
//     _mumpActiveKeyFrames = _mpMap->GetActiveKeyFrames();
//     _mumpActiveMapPoints = _mpMap->GetActiveMapPoints();
//     _mumpAllKeyFrames = _mpMap->GetAllKeyFrames();
//     _mumpAllMapPoints = _mpMap->GetAllMapPoints();
//     _mbMapUpdated = true;
// }



// --------------------------------------------------------------

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera){
        SE3 Twc = current_frame_->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }


// --------------------------------------------------------------

    void Viewer::DrawFrame(Frame::Ptr frame, const float* color){
        if(frame->is_keyframe_) {
            SE3 Twc = frame->Pose().inverse();
            const float sz = 1.0;
            const int line_width = 2.0;
            const float fx = 400;
            const float fy = 400;
            const float cx = 512;
            const float cy = 384;
            const float width = 1080;
            const float height = 768;

            glPushMatrix();

            Sophus::Matrix4f m = Twc.matrix().template cast<float>();
            glMultMatrixf((GLfloat *) m.data());

            if (color == nullptr) {
                glColor3f(1, 0, 0);
            } else
                glColor3f(color[0], color[1], color[2]);

            glLineWidth(line_width);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

            glEnd();
            glPopMatrix();
        }else{
            SE3 Twc = frame->Pose().inverse();
            const float sz = 1.0;
            const int line_width = 2.0;
            const float fx = 400;
            const float fy = 400;
            const float cx = 512;
            const float cy = 384;
            const float width = 1080;
            const float height = 768;

            glPushMatrix();

            Sophus::Matrix4f m = Twc.matrix().template cast<float>();
            glMultMatrixf((GLfloat*)m.data());

            if (color == nullptr) {
                glColor3f(1, 0, 0);
            } else
                glColor3f(color[0], color[1], color[2]);

            glLineWidth(line_width);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

            glEnd();
            glPopMatrix();
        }
    }




// --------------------------------------------------------------

    void Viewer::DrawKFsAndMPs(const bool menuShowKeyFrames, const bool menuShowPoints){
        if (menuShowKeyFrames){
            for (auto& kf: map_->GetAllKeyFrames()){
                DrawFrame(kf.second, blue);
            }
        }

        glPointSize(2);
        glBegin(GL_POINTS);

        if(menuShowPoints){
            for (auto& mp : map_->GetAllMapPoints()) {
                auto pos = mp.second->Pos();
                glColor3f(red[0], red[1], red[2]);
                glVertex3d(pos[0], pos[1], pos[2]);
            }
        }
        glEnd();
    }





}  // namespace myslam
