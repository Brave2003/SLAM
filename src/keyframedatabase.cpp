//
// Created by brave on 24-4-5.
//

#include "myslam/keyframedatabase.h"


namespace myslam{
    DBoW3::Database& KeyFrameDatabase::GetDatabase(){
        std::unique_lock<std::mutex> lck(mutex_);
        return database_;
    }

    KeyFrameDatabase::KeyframesMap KeyFrameDatabase::GetKeyFramesMap() {
        std::unique_lock<std::mutex> lck(mutex_);
        return keyframes_;
    }

    std::vector<Frame::Ptr> KeyFrameDatabase::GetALLKeyFrames() {
        std::vector<Frame::Ptr> frames;
        for(const auto& KF : keyframes_){
            frames.push_back(KF.second);
        }
        return frames;
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
        // 设置图像文件夹路径
        std::string imagesFolder = Config::Get<std::string>("dataset_dir");
        std::vector<std::string> images = {"image_0", "image_1"};
        // 存储所有描述子的向量
        std::vector<cv::Mat> descriptors;
        cv::Ptr<cv::Feature2D> orb = cv::ORB::create();


        // 遍历文件夹中的所有图像文件
        for(const auto& s:images){
            std::string path = imagesFolder + "/" + s;
            int num=0;
            for (const auto& entry : boost::filesystem::directory_iterator(path)) {

                if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") { // 或其他图像格式
                    std::string imagePath = entry.path().string();
                    LOG(INFO)<<imagePath;

                    // 读取图像
                    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
                    if (image.empty()) {
                        std::cerr << "Error loading image: " << imagePath << std::endl;
                        continue;
                    }

                    // 初始化ORB特征提取器


                    // 提取特征
                    std::vector<cv::KeyPoint> keypoints;
                    cv::Mat descriptor;
                    orb->detectAndCompute(image, cv::noArray(), keypoints, descriptor);

                    // 将描述子添加到向量中
                    descriptors.push_back(descriptor);


                    if(++num>50) break;
                }
            }
        }

        // 检查是否提取到了足够的描述子
        if (descriptors.empty()) {
            LOG(ERROR) << "No descriptors extracted from images." << std::endl;
            return -1;
        }

        initial_database_.create( descriptors );

        initial_database_.save( Config::Get<std::string>("save_dir")+ "/"
                + Config::Get<std::string>("loop_model_name"));//保存字典压缩包

        LOG(INFO) << "Vocabulary created and saved successfully." << std::endl;

        return 0;
    }

    KeyFrameDatabase::KeyFrameDatabase() {
//        this->InitialDatabase();
        database_ = DBoW3::Database(DBoW3::Vocabulary(Config::Get<std::string>("save_dir")+ "/"
                                                      + Config::Get<std::string>("loop_model_name")), false, 0);
        LOG(INFO)<<"Initial database DBoW3 model";
    }
}