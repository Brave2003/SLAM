#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

/**
 * @brief 地图
 * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
 */
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> MapPointsType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    /// 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);
    /// 增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    void InsertActivateMapPoint(MapPoint::Ptr map_point);

    void RemoveOldActivateMapPoints();



    void AddOutlierMapPoint(unsigned long id){
        std::unique_lock<std::mutex> lock(data_mutex_outlier_);
        outlier_mappoints_.push_back(id);
    }

    void RemoveAllOutlierMapPoints(){
        std::unique_lock<std::mutex> lock(data_mutex_);
        std::unique_lock<std::mutex> lock1(data_mutex_outlier_);
        for(auto iter = outlier_mappoints_.begin(); iter != outlier_mappoints_.end(); iter++){
            mappoints_.erase(*iter);
            activate_mappoints_.erase(*iter);
        }
        outlier_mappoints_.clear();
    }


    /// 获取所有地图点
    MapPointsType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return mappoints_;
    }
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 获取激活地图点
    MapPointsType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return activate_mappoints_;
    }

    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }



    /// 清理map中观测数量为零的点
    void CleanMap();

    void RemoveMapPoint(MapPoint::Ptr mappoint){
        std::unique_lock<std::mutex> lock(data_mutex_);
        unsigned long mappoint_id = mappoint->id_;
        mappoints_.erase(mappoint_id);
        activate_mappoints_.erase(mappoint_id);
    }

    std::mutex data_update_mutex_, data_mutex_outlier_, data_mutex_;

   private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldActivateKeyframe();


    MapPointsType mappoints_;         // all landmarks
    MapPointsType activate_mappoints_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // active key-frames
    std::list<unsigned long> outlier_mappoints_;

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 激活的关键帧数量
};
}  // namespace myslam

#endif  // MAP_H
