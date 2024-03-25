#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"

namespace myslam{
    class Map;

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

            void UpdateMap();

            void Stop();

        private:

            // process
            void BackendLoop();

            void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

            Map::Ptr map_;
            std::thread backend_thread_;
            std::mutex data_mutex_;

            // 是一个同步原语，它允许线程等待某个条件成立。
            std::condition_variable map_update_;

            // 用于表示后端是否正在运行
            std::atomic<bool> backend_running_;

            Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}

#endif