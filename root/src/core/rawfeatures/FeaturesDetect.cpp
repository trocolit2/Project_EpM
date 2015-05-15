/*
 * FeaturesDetect.cpp
 *
 *  Created on: May 11, 2015
 *      Author: tiagotrocoli
 */

#include "FeaturesDetect.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <iostream>

FeaturesDetect::FeaturesDetect(std::string methodName) {
    cv::initModule_nonfree();
    _featureDetector = initFeatureDetector(methodName);
}

std::vector<cv::KeyPoint> FeaturesDetect::detector(cv::Mat image) {

    std::vector<cv::KeyPoint> vectorKeys;
    _featureDetector->detect(image, vectorKeys);
    return vectorKeys;
}

cv::Mat FeaturesDetect::drawKeyFeatures(std::vector<cv::KeyPoint> vectorKeys, cv::Mat background) {
    cv::Mat out;
    cv::drawKeypoints(background, vectorKeys, out);
    return out;
}

void FeaturesDetect::changeDetectorMethod(std::string methodName) {
    _featureDetector = initFeatureDetector(methodName);
}
