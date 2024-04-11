//
// Created by brave on 24-4-10.
//

#ifndef MYSLAM_DEEPLCD_H
#define MYSLAM_DEEPLCD_H

#include "myslam/common_include.h"
#include "caffe/caffe.hpp"
namespace myslam{
class DeepLCD{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<DeepLCD> Ptr;

    typedef Eigen::Matrix<float, 1064, 1> DescrVector;

    caffe::Net<float>* autoencoder;
    caffe::Blob<float>* autoencoder_input;
    caffe::Blob<float>* autoencoder_output;

    DeepLCD(const std::string& network_definition_file="/home/brave/Desktop/A-Simple-Stereo-SLAM-System-with-Deep-Loop-Closing/calc_model/deploy.prototxt",
            const std::string& pre_trained_model_file="/home/brave/Desktop/A-Simple-Stereo-SLAM-System-with-Deep-Loop-Closing/calc_model/calc.caffemodel",
            int gpu_id=-1);

    ~DeepLCD(){
        delete autoencoder;
    }

    const float score(const DescrVector& d1, const DescrVector& d2);

    DescrVector calcDescrOriginalImg(const cv::Mat& originalImg);

    const DescrVector calcDescr(const cv::Mat& im);

private:
};
}

#endif //MYSLAM_DEEPLCD_H
