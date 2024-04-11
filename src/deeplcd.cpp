//
// Created by brave on 24-4-10.
//
#include "myslam/deeplcd.h"


namespace myslam{
    DeepLCD::DeepLCD(const std::string &network_definition_file,
                     const std::string &pre_trained_model_file,
                     int gpu_id) {
        std::string mode = "CPU";
        if(gpu_id>0){
            caffe::Caffe::set_mode(caffe::Caffe::GPU);
            caffe::Caffe::SetDevice(gpu_id);
            mode = "GPU";
        }
        else{
            caffe::Caffe::set_mode(caffe::Caffe::CPU);
        }

        clock_t begin = clock();
        autoencoder = new caffe::Net<float>(network_definition_file, caffe::TEST);
        autoencoder->CopyTrainedLayersFrom(pre_trained_model_file);
        clock_t end = clock();
        autoencoder_input = autoencoder->input_blobs()[0];
        autoencoder_output = autoencoder->output_blobs()[0];
    }

    const float DeepLCD::score(const myslam::DeepLCD::DescrVector &d1, const myslam::DeepLCD::DescrVector &d2) {
        float  result = d1.transpose() * d2;
        return result;
    }

    DeepLCD::DescrVector DeepLCD::calcDescrOriginalImg(const cv::Mat &originalImg) {
        assert(!originalImg.empty());

        cv::GaussianBlur(originalImg, originalImg, cv::Size(7,7) , 0);
        cv::Size size(160, 120);
        cv::Mat imResize;
        cv::resize(originalImg, imResize, size);
        return calcDescr(imResize);
    }

    const DeepLCD::DescrVector DeepLCD::calcDescr(const cv::Mat &im_) {
        std::vector<cv::Mat> input_channels(1);
        int w = autoencoder_input->width();
        int h = autoencoder_input->height();

        float* input_data = autoencoder_input->mutable_cpu_data();
        cv::Mat channel(h, w, CV_32FC1, input_data);
        input_channels.emplace(input_channels.begin(), channel);
        input_data += w*h;

        cv::Mat im(im_.size(), CV_32FC1);
        // 归一化
        im_.convertTo(im, CV_32FC1, 1.0/255.0);

        cv::split(im, input_channels);

        autoencoder->Forward();

        const float * temp_descr;
        temp_descr = autoencoder_output->cpu_data();
        int p = autoencoder_output->channels();

        int sz = p * sizeof(float);
        float* descr = (float*)std::malloc(sz);

        std::memcpy(descr, temp_descr, sz);
        assert(p==1064);

        DescrVector descrVector;
        for(int i=0;i<p;i++){
            descrVector(i, 0) = *(descr + i);
        }

        descrVector = descrVector / descrVector.norm();
        return descrVector;
    }
}
