//
// Created by brave on 24-4-5.
//

#include "myslam/keyframedatabase.h"

namespace myslam{
    DBoW3::Database& KeyFrameDatabase::GetDatabase(){
        std::unique_lock<std::mutex> lck(mutex_);
        return database_;
    }

    KeyFrameDatabase::KeyframesMap KeyFrameDatabase::GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(mutex_);
        return keyframes_;
    }

    void KeyFrameDatabase::InsertKeyFrame(Frame::Ptr frame) {
        if(keyframes_.find(frame->keyframe_id_)==keyframes_.end()){
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        }else{
            keyframes_[frame->keyframe_id_] = frame;
        }

        database_.add(frame->descriptors_);
    }

    bool KeyFrameDatabase::InitialDatabase() {
        //TODO
        return true;
    }
}