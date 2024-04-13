/*
//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

*/
/**
 * 可视化
 *//*

class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    // 更新地图
    void UpdateMap();

   private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float* color);



    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> activate_mappoints_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}  // namespace myslam

#endif  // MYSLAM_VIEWER_H
*/
#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include "myslam/common_include.h"


#include <thread>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

namespace myslam {

    class Frame;
    class KeyFrame;
    class MapPoint;
    class Map;


    class Viewer{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;
        // typedef std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>> KeyFramesType;
        // typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> MapPointsType;

        Viewer();

        void SetMap(std::shared_ptr<Map> map){
            map_ = map;
        }

        void Close();

        // add the current frame to viewer
        void AddCurrentFrame(std::shared_ptr<Frame> currentFrame);

        // get the information about kf/mp from the map
        // void UpdateMap();


    private:

        void ThreadLoop();

        // show the current frame's left image and feature points
        cv::Mat PlotFrameImage();

        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        void DrawKFsAndMPs(const bool menuShowKeyFrames, const bool menuShowPoints);

        void DrawFrame(std::shared_ptr<Frame> frame, const float* color);





    private:
        std::thread  thread_;
        std::mutex viewer_data_;

        std::shared_ptr<Frame> current_frame_ = nullptr;
        std::shared_ptr<Map> map_ = nullptr;

        bool map_update_ = false;
        bool viewer_running_ = true;

        const float red[3] = {1, 0, 0};
        const float green[3] = {0, 1, 0};
        const float blue[3] = {0, 0, 1};

        double fps_;
        int t_;




    };






}  // namespace myslam

#endif  // MYSLAM_VIEWER_H