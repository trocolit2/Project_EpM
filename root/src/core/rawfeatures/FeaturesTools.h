/*
 * FeaturesTools.h
 *
 *  Created on: May 14, 2015
 *      Author: tiagotrocoli
 */

#ifndef SRC_CORE_RAWFEATURES_FEATURESTOOLS_H_
#define SRC_CORE_RAWFEATURES_FEATURESTOOLS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

class FeaturesTools {
public:

    static cv::Mat refineMatchesWithHomography(std::vector<cv::KeyPoint> queryKeypoints, std::vector<cv::KeyPoint> trainKeypoints, std::vector<cv::DMatch> *matches, float reprojectionThreshold = 15.0);

    static cv::Mat calcFundamentalMatrix(std::vector<cv::KeyPoint> queryKeypoints, std::vector<cv::KeyPoint> trainKeypoints, std::vector<cv::DMatch> *matches);

    static void calcEpiplesFromImages(cv::Mat fundamentalMat, cv::Point2f *epipole1, cv::Point2f *epipole2);

    static void calcFocalLength2Images(cv::Mat fundamentalMat, double *focalLenght1, double *focalLenght2 = 0);

};

#endif /* SRC_CORE_RAWFEATURES_FEATURESTOOLS_H_ */
