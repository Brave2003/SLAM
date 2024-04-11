//
// Created by brave on 24-4-5.
//

#ifndef MYSLAM_LOOPCLOSURE_H
#define MYSLAM_LOOPCLOSURE_H

#include "common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/g2o_types.h"
#include "myslam/feature.h"
#include "myslam/algorithm.h"
#include "myslam/ORBextractor.h"
#include "myslam/backend.h"
#include "myslam/deeplcd.h"


namespace myslam{
    class Backend;
    class LoopClosure{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<LoopClosure> Ptr;

        LoopClosure();

        void SetCameras(Camera::Ptr left, Camera::Ptr right){
            cam_left_ = left;
            cam_right_ = right;
        }

        void SetMap(Map::Ptr map){
            map_ = map;
        }

        void InsertKeyFrame(Frame::Ptr frame);

        void UpdateMap();

        void Stop();

        void SetORB(ORBextractor::Ptr orb){
            orb_ = orb;
        }

        void SetBackend(std::shared_ptr<Backend> backend){
            backend_ = backend;
        }



    private:

        void LoopClosureLoop();

        void ProcessNewKF();

        unsigned int DetectLoop();

        bool CheckNewKeyFrames();

        bool MatchFeatures();

        bool ComputeCorrectPose();

        void Optimize(Map::KeyframesType &keyframesType, Map::LandmarksType &landmarksType);

        int OptimizeCurrentPose();

        void LoopCorrect();

        void LoopLocalFusion();

        void PoseGraphOptimization();

        void AddToDatabase();

        double min_loop_score = 0.4, max_loop_score = 0.9;
        unsigned int frame_passed_ = 0;

        std::thread loop_closure_thread_;
        std::mutex mutex_new_kfs_;
        std::condition_variable map_update_;
        std::atomic<bool> loop_closure_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
        Map::Ptr map_;

        ORBextractor::Ptr orb_;

        std::weak_ptr<Backend> backend_;

        std::list<Frame::Ptr> new_keyframes_;
        Frame::Ptr current_keyframe_ = nullptr;

        Frame::Ptr last_keyframe_ = nullptr;
        Frame::Ptr loop_keyframe_ = nullptr;

        bool show_close_result_ = true;
        bool is_correct_;

        cv::Ptr<cv::DescriptorMatcher> matcher_;
        std::set<std::pair<int, int> > current_loop_id_;

        SE3 correct_pose_;

        std::map<unsigned long, std::shared_ptr<Frame>> Database_;

        DeepLCD::Ptr deeplcd_;


    };

}

#endif //MYSLAM_LOOPCLOSURE_H
