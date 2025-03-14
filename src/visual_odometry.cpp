#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <chrono>

namespace myslam
{
    VisualOdometry::VisualOdometry(std::string &config_path)
        : config_file_path_(config_path) {}

    bool VisualOdometry::Init()
    {
        if (Config::SetParameterFile(config_file_path_) == false)
        {
            return false;
        }

        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        // test
        CHECK_EQ(dataset_->Init(), true);
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);
        loop_ = LoopClosure::Ptr(new LoopClosure);
        orb_ = ORBextractor::Ptr(new ORBextractor(100,1.2,8,20,7));


        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        frontend_->SetLoopColsure(loop_);
        frontend_->SetORB(orb_);

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        backend_->SetLoop(loop_);

        viewer_->SetMap(map_);

        loop_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));
        loop_->SetMap(map_);
        loop_->SetORB(orb_);
        loop_->SetBackend(backend_);

        return true;
    }

    void VisualOdometry::Run()
    {
        while (1)
        {
//            LOG(INFO) << "VO is running";
            if (Step() == false)
            {
                break;
            }
        }

        backend_->Stop();
        LOG(INFO) << "Backend is stopping";
        viewer_->Close();
        loop_->Stop();
        if(Config::Get<int>("is_save_trajectory"))
            SaveFile::SaveTrajectoryFile(Config::Get<std::string>("save_dir"), map_->GetAllKeyFrames());

        LOG(INFO) << "VO exit";
    }

    bool VisualOdometry::Step()
    {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr)
            return false;
        auto t1 = std::chrono::system_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::system_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

//        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }
}