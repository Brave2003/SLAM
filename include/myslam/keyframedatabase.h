//
// Created by brave on 24-4-5.
//

#ifndef MYSLAM_KEYFRAMEDATABASE_H
#define MYSLAM_KEYFRAMEDATABASE_H

#include "common_include.h"
#include "frame.h"
#include "DBoW3/DBoW3.h"

namespace myslam{

    class KeyFrameDatabase{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<KeyFrameDatabase> Ptr;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesMap;

        KeyFrameDatabase():database_(DBoW3::Vocabulary(), false, 0){};

        void InsertKeyFrame(Frame::Ptr frame);

        DBoW3::Database& GetDatabase();

        KeyframesMap GetAllKeyFrames();


        bool InitialDatabase();


    private:
        DBoW3::Database database_;
        DBoW3::Vocabulary initial_database_;
        std::mutex mutex_;
        KeyframesMap keyframes_;
    };
}

#endif //MYSLAM_KEYFRAMEDATABASE_H
