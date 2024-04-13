#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/config.h"

namespace myslam{
    Backend::Backend(){
        backend_running_.store(true);
        request_pause_.store(false);
        pause_.store(false);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

//    void Backend::UpdateMap(){
//        std::unique_lock<std::mutex> lock(data_update_mutex_);
//        map_update_.notify_one();
//    }

    void Backend::InsertKeyFrame(Frame::Ptr frame) {
        std::unique_lock<std::mutex> lock(mutex_new_kf_);
        new_keyframes_.push_back(frame);
        need_optimization_ = true;

    }

    void Backend::RequestPause(){
        request_pause_.store(true);
    }


    bool Backend::isPaused(){
        return (request_pause_.load()) && (pause_.load());
    }


    void Backend::Resume(){
        request_pause_.store(false);
    }


    void Backend::Stop(){
        backend_running_.store(false);
        // _mapUpdate.notify_one();
        backend_thread_.join();
    }

    bool Backend::CheckNewKeyFrames() {
        std::unique_lock<std::mutex> lock(mutex_new_kf_);
        return (!new_keyframes_.empty());
    }

    void Backend::ProcessNewKeyFrame() {
        {
            std::unique_lock<std::mutex> lock(mutex_new_kf_);
            currentKF = new_keyframes_.front();
            new_keyframes_.pop_front();
        }
        map_->InsertKeyFrame(currentKF);
        loop_->InsertKeyFrame(currentKF);
    }

    void Backend::BackendLoop(){
        while(backend_running_.load()){
            while(CheckNewKeyFrames())
                ProcessNewKeyFrame();

            while(request_pause_.load()){
                pause_.store(true);
                usleep(1000);
            }
            pause_.store(false);

            if(!CheckNewKeyFrames() && need_optimization_){
                OptimizeActiveMap();
                need_optimization_ = false;
            }
            usleep(1000);
        }
    }

    void Backend::OptimizeActiveMap(){
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        Map::KeyframesType activeKFs = map_->GetActiveKeyFrames();
        Map::MapPointsType activeMPs = map_->GetActiveMapPoints();

        // add keyframe vertices
        std::unordered_map<unsigned long, VertexPose *> vertices_kfs;
        unsigned long maxKFId = 0;
        for(auto &keyframe: activeKFs){
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);

            maxKFId = std::max(maxKFId, kf->keyframe_id_);
            vertices_kfs.insert({kf->keyframe_id_, vertex_pose});
        }
//        LOG(INFO)<<"backend optimize";
        Mat33 camK = cam_left_->K();
        SE3 camExt = cam_left_->pose();
        int index = 1; // edge index
        double chi2_th = 5.991;

        // add mappoint vertices
        // add edges
        std::unordered_map<unsigned long, VertexXYZ *> vertices_mps;
        std::unordered_map<EdgeProjection *, Feature::Ptr> edgesAndFeatures;
        for(auto &mappoint: activeMPs){

            auto mp = mappoint.second;
            if(mp->is_outlier_) {
                continue;
            }

            unsigned long mappointId = mp->id_;
            VertexXYZ *v = new VertexXYZ;
            v->setEstimate(mp->Pos());
            v->setId((maxKFId +1) + mappointId);
            v->setMarginalized(true);

            // if the KF which first observes this mappoint is not in the active map,
            // then fixed this mappoint (be included just as constraint)
            if(activeKFs.find(mp->GetObs().front().lock()->frame_.lock()->keyframe_id_) == activeKFs.end()){
                v->setFixed(true);
            }

            vertices_mps.insert({mappointId, v});
            optimizer.addVertex(v);

            // edges
            for(auto &obs: mp->GetActiveObservations()){
                auto feat = obs.lock();
                auto kf = feat->frame_.lock();

                assert(activeKFs.find(kf->keyframe_id_) != activeKFs.end());

                if(feat->is_outlier_)
                    continue;

                EdgeProjection *edge = new EdgeProjection(camK, camExt);
                edge->setId(index);
                edge->setVertex(0, vertices_kfs.at(kf->keyframe_id_));
                edge->setVertex(1, vertices_mps.at(mp->id_));
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edgesAndFeatures.insert({edge, feat});

                optimizer.addEdge(edge);
                index++;
            }
        }

        // do optimization
        int cntOutlier = 0, cntInlier = 0;
        int iteration = 0;
        LOG(INFO)<<"backend optimize";
        while(iteration < 5){
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cntOutlier = 0;
            cntInlier = 0;
            // determine if we want to adjust the outlier threshold
            for(auto &ef: edgesAndFeatures){
                if(ef.first->chi2() > chi2_th){
                    cntOutlier++;
                }else{
                    cntInlier++;
                }
            }
            double inlierRatio = cntInlier / double(cntInlier + cntOutlier);
            if(inlierRatio > 0.5){
                break;
            }else{
                // chi2_th *= 2;
                iteration++;
            }
        }

        // process the outlier edges
        // remove the link between the feature and the mappoint
        for(auto &ef: edgesAndFeatures){
            if(ef.first->chi2() > chi2_th){
                ef.second->is_outlier_ = true;
                auto mappoint = ef.second->map_point_.lock();
                mappoint->RemoveActivateObservation(ef.second);
                mappoint->RemoveObservation(ef.second);
                // if the mappoint has no good observation, then regard it as a outlier. It will be deleted later.
                if(mappoint->GetObs().empty()){
                    mappoint->is_outlier_ = true;
                    map_->AddOutlierMapPoint(mappoint->id_);
                }
                ef.second->map_point_.reset();
            }else{
                ef.second->is_outlier_ = false;
            }
        }

        { // mutex
            std::unique_lock<std::mutex> lck(map_->data_update_mutex_);
            // update the pose and landmark position
            for (auto &v: vertices_kfs) {
                activeKFs.at(v.first)->SetPose(v.second->estimate());
            }
            for (auto &v: vertices_mps){
                activeMPs.at(v.first)->SetPos(v.second->estimate());
            }

//            // delete outlier mappoints
            map_->RemoveAllOutlierMapPoints();
            map_->RemoveOldActivateMapPoints();
        } // mutex

        // LOG(INFO) << "Backend: Outlier/Inlier in optimization: " << cntOutlier << "/" << cntInlier;
    }
    void Backend::Optimize(){
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()
            )
        );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        Map::KeyframesType keyframes = map_->GetActiveKeyFrames();
        Map::MapPointsType landmarks = map_->GetActiveMapPoints();

        //vertex
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes){
            auto kf = keyframe.second;
            auto *vertex_pose = new VertexPose();
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
        auto chi2_th = Config::Get<double>("chi2_th");

        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for(auto &landmark : landmarks){
            if (landmark.second -> is_outlier_) continue;
            unsigned long landmark_id = landmark.second -> id_;
            auto mp = landmark.second;
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

                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==vertices_landmarks.end()) {
                    auto *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});

                    optimizer.addVertex(v);
                }

                edge -> setId(index++);
                edge -> setVertex(0, vertices.at(frame->keyframe_id_));
                edge -> setVertex(1, vertices_landmarks.at(landmark_id));
                edge -> setMeasurement(toVec2(feat->position_.pt));
                // 表示这条边的不确定性或权重
                // 息矩阵越大（对角线上的值越大），表示对应的观测或约束越精确或权重越大
                edge -> setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk -> setDelta(chi2_th);
                edge -> setRobustKernel(rk);
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);
            }
        }



        // 找到一个合适的chi2_th阈值，使得内点的比例超过50%
        // 增强优化迭代的鲁棒性
        int iteration = 0;
        int cnt_outlier, cnt_inlier;
        LOG(INFO)<<"backend optimize";
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        while(iteration < 5){

            cnt_outlier = 0, cnt_inlier =0;
            for(auto &ef : edges_and_features){
                if(ef.first -> chi2() > chi2_th){
                    cnt_outlier++;
                }else{
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5){
                break;
            }else{
                chi2_th *= 2;
                iteration++;
            }
        }

        for(auto &ef : edges_and_features){
            if(ef.first->chi2() > chi2_th){
                ef.second -> is_outlier_ = true;
                ef.second -> map_point_.lock() -> RemoveObservation(ef.second);
            }else{
                ef.second -> is_outlier_ = false;
            }
        }

//        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/" <<cnt_inlier;

        {
            std::unique_lock<std::mutex> lock(map_->data_update_mutex_);
            for(auto &c: vertices){
                keyframes.at(c.first)->SetPose(c.second->estimate());
            }
            for (auto &v : vertices_landmarks) {
                landmarks.at(v.first)->SetPos(v.second->estimate());
            }
        }
    }

//        void Backend::Optimize() {
//            // setup g2o
//            typedef g2o::BlockSolver_6_3 BlockSolverType;
//            typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
//            auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
//                    g2o::make_unique<LinearSolverType>()));
//            g2o::SparseOptimizer optimizer;
//            optimizer.setAlgorithm(solver);
//
//            Map::KeyframesType activeKFs = map_->GetActiveKeyFrames();
//            Map::MapPointsType activeMPs = map_->GetActiveMapPoints();
//
//            // add keyframe vertices
//            std::unordered_map<unsigned long, VertexPose *> vertices_kfs;
//            unsigned long maxKFId = 0;
//            for (auto &keyframe: activeKFs) {
//                auto kf = keyframe.second;
//                VertexPose *vertex_pose = new VertexPose();
//                vertex_pose->setId(kf->keyframe_id_);
//                vertex_pose->setEstimate(kf->Pose());
//                optimizer.addVertex(vertex_pose);
//
//                maxKFId = std::max(maxKFId, kf->keyframe_id_);
//                vertices_kfs.insert({kf->keyframe_id_, vertex_pose});
//            }
//
//            Mat33 camK = cam_left_->K();
//            SE3 camExt = cam_left_->pose_;
//            int index = 1; // edge index
//            double chi2_th = 5.991;
//
//            // add mappoint vertices
//            // add edges
//            std::unordered_map<unsigned long, VertexXYZ *> vertices_mps;
//            std::unordered_map<EdgeProjection *, Feature::Ptr> edgesAndFeatures;
//            for (auto &mappoint: activeMPs) {
//                auto mp = mappoint.second;
//                if (mp->is_outlier_) {
//                    continue;
//                }
//
//                unsigned long mappointId = mp->id_;
//                VertexXYZ *v = new VertexXYZ;
//                v->setEstimate(mp->Pos());
//                v->setId((maxKFId + 1) + mappointId);
//                v->setMarginalized(true);
//
//                // if the KF which first observes this mappoint is not in the active map,
//                // then fixed this mappoint (be included just as constraint)
//                if (activeKFs.find(mp->GetObs().front().lock()->frame_.lock()->keyframe_id_) == activeKFs.end()) {
//                    v->setFixed(true);
//                }
//
//                vertices_mps.insert({mappointId, v});
//                optimizer.addVertex(v);
//
//                // edges
//                for (auto &obs: mp->GetObs()) {
//                    auto feat = obs.lock();
//                    auto kf = feat->frame_.lock();
//
//                    assert(activeKFs.find(kf->keyframe_id_) != activeKFs.end());
//
//                    if (feat->is_outlier_)
//                        continue;
//
//                    EdgeProjection *edge = new EdgeProjection(camK, camExt);
//                    edge->setId(index);
//                    edge->setVertex(0, vertices_kfs.at(kf->keyframe_id_));
//                    edge->setVertex(1, vertices_mps.at(mp->id_));
//                    edge->setMeasurement(toVec2(feat->position_.pt));
//                    edge->setInformation(Mat22::Identity());
//                    auto rk = new g2o::RobustKernelHuber();
//                    rk->setDelta(chi2_th);
//                    edge->setRobustKernel(rk);
//                    edgesAndFeatures.insert({edge, feat});
//
//                    optimizer.addEdge(edge);
//                    index++;
//                }
//            }
//
//            // do optimization
//            int cntOutlier = 0, cntInlier = 0;
//            int iteration = 0;
//
//            while (iteration < 5) {
//                optimizer.initializeOptimization();
//                optimizer.optimize(10);
//                cntOutlier = 0;
//                cntInlier = 0;
//                // determine if we want to adjust the outlier threshold
//                for (auto &ef: edgesAndFeatures) {
//                    if (ef.first->chi2() > chi2_th) {
//                        cntOutlier++;
//                    } else {
//                        cntInlier++;
//                    }
//                }
//                double inlierRatio = cntInlier / double(cntInlier + cntOutlier);
//                if (inlierRatio > 0.5) {
//                    break;
//                } else {
//                    // chi2_th *= 2;
//                    iteration++;
//                }
//            }
//
//            // process the outlier edges
//            // remove the link between the feature and the mappoint
//            for (auto &ef: edgesAndFeatures) {
//                if (ef.first->chi2() > chi2_th) {
//                    ef.second->is_outlier_ = true;
//                    auto mappoint = ef.second->map_point_.lock();
//
//                    mappoint->RemoveObservation(ef.second);
//                    // if the mappoint has no good observation, then regard it as a outlier. It will be deleted later.
//                    if (mappoint->GetObs().empty()) {
//                        mappoint->is_outlier_ = true;
//
//                    }
//                    ef.second->map_point_.reset();
//                } else {
//                    ef.second->is_outlier_ = false;
//                }
//            }
//
//            { // mutex
//                std::unique_lock<std::mutex> lck(map_->data_update_mutex_);
//                // update the pose and landmark position
//                for (auto &v: vertices_kfs) {
//                    activeKFs.at(v.first)->SetPose(v.second->estimate());
//                }
//                for (auto &v: vertices_mps) {
//                    activeMPs.at(v.first)->SetPos(v.second->estimate());
//                }
//
//            } // mutex
//        }
}