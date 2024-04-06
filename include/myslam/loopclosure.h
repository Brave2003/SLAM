//
// Created by brave on 24-4-5.
//

#ifndef MYSLAM_LOOPCLOSURE_H
#define MYSLAM_LOOPCLOSURE_H

#include "common_include.h"
#include "myslam/keyframedatabase.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "myslam/g2o_types.h"
#include "myslam/feature.h"
#include "myslam/algorithm.h"

namespace myslam{

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

        KeyFrameDatabase::Ptr GetKeyFrameDatabase(){
            return KF_database_;
        }

    private:

        void LoopClosureLoop();

        unsigned int DetectLoop();

        void Optimize(Map::KeyframesType &keyframesType, Map::LandmarksType &landmarksType);

        double min_loop_score = 0.4, max_loop_score = 0.9;
        unsigned int frame_passed_ = 0;

        std::thread loop_closure_thread_;
        std::mutex data_mutex_;
        std::condition_variable map_update_;
        std::atomic<bool> loop_closure_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
        Map::Ptr map_;

        Frame::Ptr current_frame_;
        KeyFrameDatabase::Ptr KF_database_;
    };

}

#endif //MYSLAM_LOOPCLOSURE_H
