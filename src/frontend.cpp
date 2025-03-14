//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"
#include "myslam/savefile.h"

namespace myslam
{

    Frontend::Frontend()
    {
//        if(Config::Get<std::string>("capture_feature_function")=="GFTT"){
//            gftt_ =
//                cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
//        }else{
//            orb_ =cv::ORB::create();
//        }
//        num_features_init_ = Config::Get<int>("num_features_init");
//        num_features_ = Config::Get<int>("num_features");
        orb_initial_ = ORBextractor::Ptr (new ORBextractor(300, 1.2, 8, 20, 7));
    }

    bool Frontend::AddFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;
        {
            std::unique_lock<std::mutex> lck(map_->data_update_mutex_);



            switch (status_) {

                case FrontendStatus::INITING:
                    StereoInit();
                    break;
                case FrontendStatus::TRACKING_GOOD:
                case FrontendStatus::TRACKING_BAD:
                    Track();
                    break;
                case FrontendStatus::LOST:
                    Reset();
                    break;

            }
        }

        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
        }
        last_frame_ = current_frame_;
        return true;
    }

    /**
     * @brief
     * 1.寻找左图中特征点
     * 2.按照作图特征点寻找
     *
     * @return true
     * @return false
     */
    bool Frontend::StereoInit()
    {

        DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_)
        {
            return false;
        }

        bool build_map_success = BuildInitMap();
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures()
    {
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(20, 20),
                          feat->position_.pt + cv::Point2f(20, 20), 0, CV_FILLED);
        }
        std::vector<cv::KeyPoint> keypoints;
        if(status_== FrontendStatus::INITING){
            orb_initial_->Detect(current_frame_->left_img_, mask, keypoints);
        }else{
            orb_->Detect(current_frame_->left_img_, mask, keypoints);
        }
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }
        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->map_point_.lock();
            if (mp && mp->is_outlier_== false)
            {
                // use projected points as initial guess
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->PoseRelative()*last_keyframe_->Pose().inverse());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            // create map point from triangulation
            // 转换到相机坐标系下
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();
            // 测量深度
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
//                new_map_point->AddObservation(current_frame_->features_left_[i]);
//                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
        }
        InsertKeyframe();

        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    bool Frontend::Track()
    {
        if (last_frame_)
        {
            current_frame_->SetPoseRelative(relative_motion_ * last_frame_->PoseRelative());
        }
        // 跟踪
        TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();



        if (tracking_inliers_ > 50)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > 10)
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        relative_motion_ = current_frame_->PoseRelative() * last_frame_->PoseRelative().inverse();

        if(status_ == FrontendStatus::TRACKING_BAD){
            DetectFeatures();
            FindFeaturesInRight();
            TriangulateNewPoints();
            InsertKeyframe();
        }

        return true;
    }

    void Frontend::SetObservationsForKeyFrame() {
        for (auto &feat : current_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) mp->AddObservation(feat);
        }
    }

    bool Frontend::InsertKeyframe()
    {

        Vec6 se3_zero;
        se3_zero.setZero();



        current_frame_->SetKeyFrame();
        SetObservationsForKeyFrame();
        if(status_ == FrontendStatus::INITING){
            current_frame_->SetPose(SE3::exp(se3_zero));
        }else {


            current_frame_->SetPose(current_frame_->pose_relative_ * last_keyframe_->pose_);
            current_frame_->lastKeyFrame_ = last_keyframe_;
            current_frame_->relativePoseToLastKF = current_frame_->pose_relative_;
        }

        // update backend because we have a new keyframe
        backend_->InsertKeyFrame(current_frame_);

        last_keyframe_ = current_frame_;
        current_frame_->SetPoseRelative(SE3::exp(se3_zero));

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;
        return true;
    }



    int Frontend::TriangulateNewPoints()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = (current_frame_->PoseRelative()*last_keyframe_->Pose()).inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
//                    new_map_point->AddObservation(
//                        current_frame_->features_left_[i]);
//                    new_map_point->AddObservation(
//                        current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose()
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 设置顶点
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->PoseRelative()*last_keyframe_->Pose());
        optimizer.addVertex(vertex_pose);

        Mat33 K = camera_left_->K();

        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        edges.reserve(current_frame_->features_left_.size());
        features.reserve(current_frame_->features_left_.size());
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            // 会存在 不存在对路标点的映射的情况
            if (mp && mp->is_outlier_ == false)
            {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        const double chi2_th = Config::Get<double>("chi2_th");
        int cnt_outlier = 0;
        for (int itertation = 0; itertation < 4; ++itertation)
        {

            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                }
                if (itertation == 2)
                {
                    e->setRobustKernel(nullptr); //
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier;
        current_frame_->SetPose(vertex_pose->estimate());
        current_frame_->SetPoseRelative(vertex_pose->estimate()*last_keyframe_->Pose().inverse());

//        LOG(INFO) << "Current Pose = \n"
//                  << current_frame_->Pose().matrix();

        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                auto mp = feat->map_point_.lock();
                if(mp && current_frame_->id_ - last_keyframe_->id_ <=2 ){
                    mp->is_outlier_ = true;
                    map_->AddOutlierMapPoint(mp->id_);
                }
                feat->map_point_.reset();
                feat->is_outlier_ = false;
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                // use project point
                auto mp = kp->map_point_.lock();
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->PoseRelative()*last_keyframe_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        // 只需要比较两个左图，在左右图比较时已经对左图特征点进行了处理
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i] && !last_frame_->features_left_[i]->map_point_.expired())
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    bool Frontend::Reset()
    {
        LOG(INFO) << "Reset is not implemented. ";
        return true;
    }

} // namespace myslam