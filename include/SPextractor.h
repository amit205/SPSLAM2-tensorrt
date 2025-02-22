/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. G�mez Rodr�guez, Jos� M.M. Montiel and Juan D. Tard�s, University of Zaragoza.
* Copyright (C) 2014-2016 Ra�l Mur-Artal, Jos� M.M. Montiel and Juan D. Tard�s, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPEXTRACTOR_H
#define SPEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include "SuperPoint.h"
#include "engine.h"
#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif



namespace ORB_SLAM2
{

    class ExtractorNode
    {
    public:
        ExtractorNode() :bNoMore(false) {}

        void DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class SPextractor
    {
    public:

        enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

        SPextractor(int nfeatures, float scaleFactor, int nlevels,
            float iniThFAST, float minThFAST);

        ~SPextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        
        void operator()(cv::InputArray _image, cv::InputArray _mask,
            std::vector<cv::KeyPoint>& _keypoints,
            cv::OutputArray _descriptors);
        /*
        void operator()(cv::InputArray _image, cv::InputArray _mask,
            std::vector<cv::KeyPoint>& _keypoints,
            cv::OutputArray _descriptors);*/

        int inline GetLevels() {
            return nlevels;
        }

        float inline GetScaleFactor() {
            return scaleFactor;
        }

        std::vector<float> inline GetScaleFactors() {
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors() {
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares() {
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares() {
            return mvInvLevelSigma2;
        }

        std::vector<cv::Mat> mvImagePyramid;

    protected:

        void ComputePyramid(cv::Mat image);
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, cv::Mat& _desc);
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int& minX,
            const int& maxX, const int& minY, const int& maxY, const int& nFeatures, const int& level);

        //void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
        //std::vector<cv::Point> pattern;

        int nfeatures;
        double scaleFactor;
        int nlevels;
        float iniThFAST;
        float minThFAST;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

        //std::shared_ptr<SuperPoint> model;
        std::shared_ptr<SuperPoint> model_engine;
    };

    typedef SPextractor ORBextractor;

} //namespace ORB_SLAM

#endif

