#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/loopclosure.h"

namespace myslam{
    class LoopClosure;
    class Backend{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            Backend();

            void SetCameras(Camera::Ptr left, Camera::Ptr right){
                cam_left_ = left;
                cam_right_ = right;
            }

            void SetMap(Map::Ptr map){
                map_ = map;
            }

            void SetLoop(std::shared_ptr<LoopClosure> loop){
                loop_ = loop;
            }

//            void UpdateMap();

            void Stop();

            void InsertKeyFrame(Frame::Ptr frame);

            void RequestPause();

            bool isPaused();

            void Resume();

            bool CheckNewKeyFrames();

            void ProcessNewKeyFrame();

        private:

            // process
            void BackendLoop();

            void Optimize();

            Map::Ptr map_;
            std::thread backend_thread_;
            std::mutex mutex_new_kf_;

            // 是一个同步原语，它允许线程等待某个条件成立。
            std::condition_variable map_update_;

            // 用于表示后端是否正在运行
            std::atomic<bool> backend_running_;
            std::atomic<bool> request_pause_;
            std::atomic<bool> pause_;

            Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

            std::list<Frame::Ptr> new_keyframes_;
            Frame::Ptr currentKF;

            std::shared_ptr<LoopClosure> loop_;

            bool need_optimization_=false;
    };
}

#endif