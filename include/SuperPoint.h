#ifndef SUPERPOINT_H
#define SUPERPOINT_H


#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include "engine.h"
#include <vector>

#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif


namespace ORB_SLAM2
{
    /*
    struct SuperPoint : torch::nn::Module {
        SuperPoint();

        std::vector<torch::Tensor> forward(torch::Tensor x);


        torch::nn::Conv2d conv1a;
        torch::nn::Conv2d conv1b;

        torch::nn::Conv2d conv2a;
        torch::nn::Conv2d conv2b;

        torch::nn::Conv2d conv3a;
        torch::nn::Conv2d conv3b;

        torch::nn::Conv2d conv4a;
        torch::nn::Conv2d conv4b;

        torch::nn::Conv2d convPa;
        torch::nn::Conv2d convPb;

        // descriptor
        torch::nn::Conv2d convDa;
        torch::nn::Conv2d convDb;

    };*/

    struct SuperPoint : Engine {
        SuperPoint(const Options& options):Engine(options){}
    };

    //cv::Mat SPdetect(std::shared_ptr<SuperPoint> model, cv::Mat img, std::vector<cv::KeyPoint>& keypoints, double threshold, bool nms, bool cuda);
    cv::Mat SPdetect(std::shared_ptr<SuperPoint> model_engine, cv::Mat img, std::vector<cv::KeyPoint>& keypoints, double threshold, bool nms, bool cuda);
    //torch::Tensor NMS(torch::Tensor kpts);

    class SPDetector {
    public:
        //SPDetector(std::shared_ptr<SuperPoint> _model);
        SPDetector(std::shared_ptr<SuperPoint> _model_engine);
        void detect(cv::Mat& image, bool cuda);
        void getKeyPoints(float threshold, int iniX, int maxX, int iniY, int maxY, std::vector<cv::KeyPoint>& keypoints, bool nms);
        void computeDescriptors(const std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
        

    private:
        //std::shared_ptr<SuperPoint> model;
        std::shared_ptr<SuperPoint> model_engine;
        torch::Tensor mProb;
        torch::Tensor mDesc;
        float image_scale_width;
        float image_scale_height;
        //std::vector<cv::Mat> mProb;
        //std::vector<cv::Mat> mDesc;
    };
}  // ORB_SLAM

#endif