/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/map.h"
#include "myslam/feature.h"
#include "myslam/config.h"

namespace myslam {

void Map::InsertKeyFrame(Frame::Ptr frame) {
    // 贪心策略

    current_frame_ = frame;
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        }
        else {
            // std::cout<<frame->keyframe_id_;
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }
    }


     for(auto& feat: frame->features_left_){
         auto mp = feat->map_point_.lock();
         if(mp){
             mp->AddActivateObservation(feat);
             InsertActivateMapPoint(mp);
         }
     }

    if (active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldActivateKeyframe();
        RemoveOldActivateMapPoints();
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (mappoints_.find(map_point->id_) == mappoints_.end()) {
        mappoints_.insert(make_pair(map_point->id_, map_point));
//        activate_mappoints_.insert(make_pair(map_point->id_, map_point));
    } 
     else {
        mappoints_[map_point->id_] = map_point;
//        activate_mappoints_[map_point->id_] = map_point;
     }
}

void Map::RemoveOldActivateKeyframe() {
    if (current_frame_ == nullptr) return;
    // 寻找与当前帧最近与最远的两个关键帧
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = Config::Get<double>("min_dis_th");  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

// LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feat : frame_to_remove->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveActivateObservation(feat);
        }
    }


    CleanMap();
}

void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = activate_mappoints_.begin();
         iter != activate_mappoints_.end();) {
        // 没有观测到该地图点的视角
        // 只是删除活动视角下
        if (iter->second->observed_times_ == 0) {
            iter = activate_mappoints_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
//    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

void Map::InsertActivateMapPoint(MapPoint::Ptr map_point) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if(activate_mappoints_.find(map_point->id_) == activate_mappoints_.end()){
        activate_mappoints_.insert(make_pair(map_point->id_, map_point));
    }else{
        activate_mappoints_[map_point->id_] = map_point;
    }
}

    void Map::RemoveOldActivateMapPoints(){
        // if the mappoint has no active observation, then remove it from the active mappoints
        std::unique_lock<std::mutex> lck(data_mutex_);

        int cntActiveLandmarkRemoved = 0;
        for(auto iter = activate_mappoints_.begin(); iter != activate_mappoints_.end();){
            if(iter->second->activate_observed_times_ == 0){
                iter = activate_mappoints_.erase(iter);
                cntActiveLandmarkRemoved++;
            } else{
                ++iter;
            }
        }
        // LOG(INFO) << "Map: remove " << cntActiveLandmarkRemoved << " active landmarks";
    }

}  // namespace myslam
