//
// Created by brave on 24-4-5.
//

#include "myslam/loopclosure.h"

namespace myslam {

    LoopClosure::LoopClosure() {
        deeplcd_ = DeepLCD::Ptr (new DeepLCD);
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
        loop_closure_running_.store(true);
        loop_closure_thread_ = std::thread(std::bind(&LoopClosure::LoopClosureLoop, this));
    }

    void LoopClosure::Stop(){
        loop_closure_running_.store(false);
        map_update_.notify_one();
        loop_closure_thread_.join();
    }

    void LoopClosure::UpdateMap() {
        std::unique_lock<std::mutex> lck(mutex_new_kfs_);
        map_update_.notify_one();
    }

    unsigned int LoopClosure::DetectLoop() {
        std::vector<float> scores;
        float max_score = 0;
        int cnt_suspected = 0;
        unsigned long best_id = 0;

        for(auto &db: Database_){
            if(current_keyframe_->keyframe_id_ - db.first<20) break;
            float similarity_score = deeplcd_->score(current_keyframe_->descrVector_, db.second->descrVector_);
            if(similarity_score> max_score){
                max_score = similarity_score;
                best_id = db.first;
            }
            if(similarity_score > 0.92){
                cnt_suspected++;
            }
        }

        if(max_score < 0.94 || cnt_suspected >3){
            return false;
        }

        loop_keyframe_ = Database_.at(best_id);

        LOG(INFO) << "\n\n\nLoopClosing: DeepLCD find potential Candidate KF.\n\n\n";
        return true;
    }

    bool LoopClosure::CheckNewKeyFrames() {
        std::unique_lock<std::mutex> lock(mutex_new_kfs_);
        return (!new_keyframes_.empty());
    }

    void LoopClosure::InsertKeyFrame(Frame::Ptr frame) {
        std::unique_lock<std::mutex> lock(mutex_new_kfs_);
        if(last_keyframe_==nullptr || frame->keyframe_id_ - last_keyframe_->keyframe_id_ >5){
            new_keyframes_.push_back(frame);
        }else{
            frame->left_img_.release();
        }
    }

    void LoopClosure::LoopClosureLoop() {
        while(loop_closure_running_.load()){
            if(CheckNewKeyFrames()){
                ProcessNewKF();

                bool confirm = false;
                // Database_min_size_ = 50
                if(Database_.size() > 50){
                    if(DetectLoop()){
                        if(MatchFeatures()){
                            confirm = ComputeCorrectPose();
                            if(confirm){
                                LoopCorrect();
                            }
                        }
                    }
                }
                if(!confirm){
                    AddToDatabase();
                }
            }
            usleep(1000);
        }

    }

    void LoopClosure::AddToDatabase() {
        Database_.insert({current_keyframe_->keyframe_id_, current_keyframe_});
        last_keyframe_ = current_keyframe_;
    }

    void LoopClosure::LoopCorrect() {
        if(!is_correct_){
            LOG(INFO) << "LoopClosing: no need for correction.";
            return ;
        }
        auto back = backend_.lock();
        back->RequestPause();
        while(!back->isPaused()){
            usleep(1000);
        }

        // correct active KF and mp
        LoopLocalFusion();

        // optimize all the previous KFs' poses
        PoseGraphOptimization();


    }

    void LoopClosure::PoseGraphOptimization() {
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
        typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        Map::KeyframesType keyframes = map_->GetAllKeyFrames();
        std::map<unsigned long, VertexPose *> vertexs;
        for(auto &keyframe: keyframes){
            unsigned long keyframe_id = keyframe.first;
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            // 设置边缘化
            vertex_pose->setMarginalized(false);

            auto active_keyframes = map_->GetAllKeyFrames();

            if(active_keyframes.find(keyframe_id)!=active_keyframes.end() || (keyframe_id == loop_keyframe_->keyframe_id_) || keyframe_id==0){
                // 固定点
                vertex_pose->setFixed(true);
            }

            optimizer.addVertex(vertex_pose);
            vertexs.insert({kf->keyframe_id_, vertex_pose});
        }

        int index=0;
        std::map<int, EdgePoseGraph*> edges;
        for(auto& id_keyframe : keyframes){
            unsigned long keyframe_id = id_keyframe.first;
            auto keyframe = id_keyframe.second;

            auto last_keyframe = keyframe->lastKeyFrame_.lock();
            if(last_keyframe){
                EdgePoseGraph *edge = new EdgePoseGraph();
                edge->setId(index);
                edge->setVertex(0, vertexs.at(keyframe_id));
                edge->setVertex(1, vertexs.at(last_keyframe->keyframe_id_));
                edge->setMeasurement(keyframe->relativePoseToLastKF);
                edge->setInformation(Mat66::Identity());
                optimizer.addEdge(edge);
                edges.insert({index,edge});
                index++;
            }
            auto loop_keyframe = keyframe->loopKeyFrame_.lock();
            if(loop_keyframe) {
                EdgePoseGraph *edge = new EdgePoseGraph();
                edge->setId(index);
                edge->setVertex(0, vertexs.at(keyframe_id));
                edge->setVertex(1, vertexs.at(loop_keyframe->keyframe_id_));
                edge->setMeasurement(keyframe->relativePoseToLoopKF);
                edge->setInformation(Mat66::Identity());
                optimizer.addEdge(edge);
                edges.insert({index, edge});
                index++;
            }
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // correct kfs pose
        {
            std::unique_lock<std::mutex> lock(map_->data_mutex_);

            auto mappoints = map_->GetAllMapPoints();
            auto active_mappoints = map_->GetActiveMapPoints();
            for(auto iter = active_mappoints.begin(); iter != active_mappoints.end(); iter++){
                mappoints.erase((*iter).first);
            }

            for(auto& mappoint: mappoints){
                MapPoint::Ptr mp = mappoint.second;
                assert(!mp->GetObs().empty());
                auto feat = mp->GetObs().front().lock();
                auto obs_keyframe = feat->frame_.lock();
                if(vertexs.find(obs_keyframe->keyframe_id_)==vertexs.end()){
                    continue;
                }

                Vec3 pos_camera = obs_keyframe->Pose()*mp->Pos();
                SE3 T_optimized = vertexs.at(obs_keyframe->keyframe_id_)->estimate();
                mp->SetPos(T_optimized.inverse()* pos_camera);
            }

            for(auto &v : vertexs){
                keyframes.at(v.first)->SetPose(v.second->estimate());
            }
        }

        LOG(INFO) << "LoopClosing: Pose Graph optimization done.";
    }

    void LoopClosure::LoopLocalFusion(){
        // avoid conflict between tracking
        std::unique_lock<std::mutex> lock(map_->data_mutex_);

        std::unordered_map<unsigned long, SE3> correct_active_poses;
        correct_active_poses.insert({current_keyframe_->keyframe_id_, correct_pose_});

        // calculate relative pose between current KF and KFS in active map;
        for(auto &keyframe: map_->GetActiveKeyFrames()){
            unsigned long keyframe_id = keyframe.first;
            if(keyframe_id==current_keyframe_->keyframe_id_){
                continue;
            }
            SE3 t = keyframe.second->Pose()*(current_keyframe_->Pose().inverse());
            SE3 t_corrected = t * correct_pose_;
            correct_active_poses.insert({keyframe_id, t_corrected});
        }

        // correct active mps position
        for(auto &mappoint: map_->GetActiveMapPoints()){
            MapPoint::Ptr mp = mappoint.second;
            assert(!mp->GetObs().empty());

            auto feat = mp->GetObs().front().lock();
            auto mp_obs_feat_keyframe = feat->frame_.lock();
            assert(correct_active_poses.find(mp_obs_feat_keyframe->keyframe_id_)!= correct_active_poses.end());

            Vec3 pos_camera = mp_obs_feat_keyframe->Pose() * mp->Pos();
            SE3 t_corrected = correct_active_poses.at(mp_obs_feat_keyframe->keyframe_id_);
            mp->SetPos(t_corrected.inverse()*pos_camera);

        }

        //correct active keyframe pose
        for(auto iter = current_loop_id_.begin(); iter!=current_loop_id_.end();iter++){
            int current_id = (*iter).first;
            int loop_id = (*iter).second;

            auto loop_mp = loop_keyframe_->features_left_[loop_id]->map_point_.lock();
            assert(loop_mp!= nullptr);

            if(loop_mp){
                auto current_mp = current_keyframe_->features_left_[current_id]->map_point_.lock();
                if(current_mp){
                    for(auto &obs:current_mp->GetObs()){
                        auto obs_feature = obs.lock();
                        loop_mp->AddObservation(obs_feature);
                        obs_feature->map_point_ = loop_mp;
                    }

                    map_->RemoveMapPoint(current_mp);
                }else{
                    current_keyframe_->features_left_[current_id]->map_point_ = loop_mp;
                }
            }
        }
    }

    bool LoopClosure::ComputeCorrectPose() {
        //data
        std::vector<cv::Point3f> loop_points3d;
        std::vector<cv::Point2f> current_points2d, loop_points2d;
        std::vector<cv::DMatch> matches;

        // prepare data
        for(auto iter = current_loop_id_.begin(); iter!=current_loop_id_.end();iter++){
            int current_id = (*iter).first;
            int loop_id = (*iter).second;

            auto mp = loop_keyframe_->features_left_[loop_id]->map_point_.lock();

            if(mp){
                current_points2d.push_back(current_keyframe_->features_left_[current_id]->position_.pt);
                Vec3 pos = mp->Pos();
                loop_points3d.push_back(cv::Point3f(pos(0), pos(1), pos(2)));
                loop_points2d.push_back(loop_keyframe_->features_left_[loop_id]->position_.pt);

                cv::DMatch valid_match(current_id, loop_id, 10.0);
                matches.push_back(valid_match);

                iter++;
            }else{
                iter = current_loop_id_.erase(iter);
            }
        }
        LOG(INFO) << "LoopClosing: number of valid matches with mappoints: " << loop_points3d.size();

        // viewer
        if(show_close_result_){
            cv::Mat img_matches;
            cv::drawMatches(current_keyframe_->left_img_, current_keyframe_->keypoints_, loop_keyframe_->left_img_, loop_keyframe_->keypoints_, matches, img_matches);
            cv::resize(img_matches, img_matches, cv::Size(), 0.5 ,0.5);
            cv::imshow("valid matches with mappoints", img_matches);

            cv::waitKey(1);
        }

        // opencv : solve PnP with RANSAC
        cv::Mat rvec, tvec, R, K;
        cv::eigen2cv(cam_left_->K(), K);
        Eigen::Matrix3d Reigen;
        Eigen::Vector3d teigen;


        try{
            cv::solvePnPRansac(loop_points3d, current_points2d, K, cv::Mat(), rvec, tvec, false, 100, 5.991, 0.99);

        }catch (...){
            return false;
        }
        // transfrom
        cv::Rodrigues(rvec, R);
        cv::cv2eigen(R, Reigen);
        cv::cv2eigen(tvec, teigen);

        correct_pose_ = SE3(Reigen, teigen);

        int cntInliers = OptimizeCurrentPose();
        LOG(INFO) << "LoopClosing: number of match inliers (after optimization): " << cntInliers;

        if(cntInliers<10) return false;

        double error = (current_keyframe_->Pose() * correct_pose_.inverse()).log().norm();
        if(error>1) is_correct_ = false;
        else is_correct_ = true;

        if(show_close_result_){
            Vec3 t_eigen = correct_pose_.translation();
            Mat33 R_eigen = correct_pose_.rotationMatrix();
            cv::Mat R_cv, t_cv, r_cv;
            cv::eigen2cv(R_eigen, R_cv);
            cv::eigen2cv(t_eigen, t_cv);
            cv::Rodrigues(R_cv, r_cv);
            std::vector<cv::Point2f> vReprojectionPoints2d;
            std::vector<cv::Point2f> vCurrentKeyPoints;
            std::vector<cv::Point3f> vLoopMapPoints;
            for(auto iter = current_loop_id_.begin(); iter != current_loop_id_.end(); iter++){
                int currentFeatureId = (*iter).first;
                int loopFeatureId = (*iter).second;
                auto mp = loop_keyframe_->features_left_[loopFeatureId]->map_point_.lock();
                vCurrentKeyPoints.push_back(
                        current_keyframe_->features_left_[currentFeatureId]->position_.pt);
                Vec3 pos = mp->Pos();
                vLoopMapPoints.push_back(cv::Point3f(pos(0), pos(1), pos(2)));
            }
            // do the reprojection
            cv::projectPoints(vLoopMapPoints, r_cv, t_cv, K, cv::Mat(), vReprojectionPoints2d);

            //  show the reprojection result
            cv::Mat imgOut;
            cv::cvtColor(current_keyframe_->left_img_, imgOut,cv::COLOR_GRAY2RGB);
            for(size_t index = 0, N = vLoopMapPoints.size(); index < N; index++){
                cv::circle(imgOut, vCurrentKeyPoints[index], 5, cv::Scalar(0, 0, 255), -1);
                cv::line(imgOut, vCurrentKeyPoints[index], vReprojectionPoints2d[index], cv::Scalar(255, 0, 0), 2);
            }

            cv::imshow("reprojection result of match inliers", imgOut);
            cv::waitKey(1);
        }
        
        current_keyframe_->loopKeyFrame_ = loop_keyframe_;
        current_keyframe_->relativePoseToLoopKF = correct_pose_ * loop_keyframe_->Pose().inverse();
        last_keyframe_ = current_keyframe_;

        return true;
    }

    int LoopClosure::OptimizeCurrentPose(){
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(correct_pose_);
        optimizer.addVertex(vertex_pose);

        Mat33 K = cam_left_->K();

        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        edges.reserve(current_loop_id_.size());
        std::vector<bool> edge_is_outlier;
        edge_is_outlier.resize(current_loop_id_.size());
        std::vector<std::pair<int, int>> matches;
        matches.reserve(current_loop_id_.size());

        for(auto iter = current_loop_id_.begin(); iter!= current_loop_id_.end(); iter++){
            int current_id = (*iter).first;
            int loop_id = (*iter).second;

            auto mp = loop_keyframe_->features_left_[loop_id]->map_point_.lock();
            auto point2d = current_keyframe_->features_left_[current_id]->position_.pt;
            if(mp){
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->Pos(), K);
                edge -> setId(index);
                edge -> setVertex(0, vertex_pose);
                edge -> setMeasurement(toVec2(point2d));
                edge ->setInformation(Eigen::Matrix2d ::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edge_is_outlier.push_back(false);
                matches.push_back(*iter);
                optimizer.addEdge(edge);

                index++;
            }
        }

        // estimate the Pose and determine the outliers
        // start optimization
        const double chi2_th = 5.991;
        int cntOutliers = 0;
        int numIterations = 4;

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // use the same strategy as in frontend
        for(int iteration = 0; iteration < numIterations; iteration++){
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cntOutliers = 0;

            // count the outliers
            for(size_t i = 0, N = edges.size(); i < N; i++){
                auto e = edges[i];
                if(edge_is_outlier[i]){
                    e->computeError();
                }
                if(e->chi2() > chi2_th){
                    edge_is_outlier[i] = true;
                    e->setLevel(1);
                    cntOutliers++;
                } else{
                    edge_is_outlier[i] = false;
                    e->setLevel(0);
                }

                if(iteration == numIterations - 2){
                    e->setRobustKernel(nullptr);
                }
            }
        }

        // remove the outlier match
        for(size_t i = 0, N = edge_is_outlier.size(); i < N; i++){
            if(edge_is_outlier[i]){
                current_loop_id_.erase(matches[i]);
            }
        }

        correct_pose_ = vertex_pose->estimate();

        return current_loop_id_.size();
    }

    bool LoopClosure::MatchFeatures(){
        std::vector<cv::DMatch> matches;

        matcher_ ->match(loop_keyframe_->descriptors_, current_keyframe_->descriptors_, matches);

        auto min_max = std::minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2){return m1.distance<m2.distance;});
        double min_dist = min_max.first->distance;

        current_loop_id_.clear();

        for(auto& match: matches){
            if(match.distance<=std::max(2*min_dist, 30.0)){
                int loop_id = loop_keyframe_->keypoints_[match.queryIdx].class_id;
                int current_id = current_keyframe_->keypoints_[match.trainIdx].class_id;

                if(current_loop_id_.find({current_id, loop_id})==current_loop_id_.end()){
                    current_loop_id_.insert({current_id, loop_id});
                }
            }
        }

        LOG(INFO)<< "LoopClosing: number of valid feature matches: " << current_loop_id_.size();
        if(current_loop_id_.size()<10) return false;
        return true;

    }

    void LoopClosure::ProcessNewKF() {
        {
            std::unique_lock<std::mutex> lck(mutex_new_kfs_);
            current_keyframe_ = new_keyframes_.front();
            new_keyframes_.pop_front();
        }

        current_keyframe_->descrVector_ = deeplcd_->calcDescrOriginalImg(current_keyframe_->left_img_);

        int nlevel = 8;
        std::vector<cv::KeyPoint> mappoints;
        mappoints.reserve(nlevel*current_keyframe_->features_left_.size());

        for(size_t i=0,N = current_keyframe_->features_left_.size();i<N;i++){
            current_keyframe_->features_left_[i]->position_.class_id = i;
            for(int level = 0 ;level<nlevel;level++){
                cv::KeyPoint kp(current_keyframe_->features_left_[i]->position_);
                kp.octave = level;
                kp.response = -1;
                kp.class_id = i;
                mappoints.push_back(kp);
            }
        }

        orb_->ScreenAndComputeKPsParams(current_keyframe_->left_img_, mappoints, current_keyframe_->keypoints_);
        orb_->CalcDescriptors(current_keyframe_->left_img_, current_keyframe_->keypoints_, current_keyframe_->descriptors_);

        if(!show_close_result_) current_keyframe_->left_img_.release();
    }

    void LoopClosure::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks) {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()
                        )
                );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // pose 顶点，使用Keyframe id
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;

        // K 和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991;  // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier_) continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue;
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end()) {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});

                optimizer.addEdge(edge);

                index++;
            }
        }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }

        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                // remove the observation
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

        // Set pose and lanrmark position
        {
            std::unique_lock<std::mutex> lock(map_->data_mutex_);
            for(auto &c: vertices){
                keyframes.at(c.first)->SetPose(c.second->estimate());
            }
            for (auto &v : vertices_landmarks) {
                landmarks.at(v.first)->SetPos(v.second->estimate());
            }
        }

    }



} // myslam