#include "myslam/backend.h"
#include "myslam/algorithms.h"
#include "myslam/features.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace{
    Backend::Backend(){
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap(){
        std::unique_lock<std:::mutex> lock(data_mutex_);
        map_updated_.notify_one();
    }

    void Backend::Stop(){
        backend_running_.stop(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop(){
        while(backend_running_.load()){
            std::unique_lock<std::mutex>  lock(data_mutex_);
            map_update_.wait(lock);

            Map::KeyframesType active_kfs = map_->GetActiveKeyframes();
            Map::LandmarksType active_lms = map_->GetActiveMapPoints();
            Optimize(active_kfs, active_lms);
        }
    }

    void Backend::Optimize(const Map::KeyframesType &keyframes, Map::LandmarksType &landmarks){
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()
            )
        );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);


        //vertex
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes){
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose -> setId(kf->keyframe_id_);
            vertex_pose -> setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id){
                max_kf_id = kf->keyframe_id_;
            }
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_ -> pose();
        SE3 right_ext = cam_right_ -> pose();

        int index = 1;
        // TODO 设置参数
        double chi2_th = 5.991;

        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for(auto &landmarkd : landmarks){
            if (landmark.second -> is_outlier_) continue;
            unsigned long landmark_id = landmark.second -> id_;
            auto observations = landmark.second -> GetObs();
            for(auto &obs : observations){
                if(obs.lock() == nullptr) continue;
                auto feat = obs.lock();
                if(feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if(feat->is_on_left_image_){
                    edge = new EdgeProjection(K, left_ext);
                }else{
                    edge = new EdgeProjection(K, right_ext);
                }
                

            }
        }
    }
}