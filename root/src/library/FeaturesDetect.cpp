/*
 * FeaturesDetect.cpp
 *
 *  Created on: May 11, 2015
 *      Author: tiagotrocoli
 */

#include "FeaturesDetect.h"
#include <iostream>

FeaturesDetect::FeaturesDetect(std::string methodName) {

    featureDetector = initFeatureDetector(methodName);
}

std::vector<cv::KeyPoint> FeaturesDetect::detector(cv::Mat image) {

    std::vector<cv::KeyPoint> vectorKeys;
    featureDetector->detect(image, vectorKeys);
    return vectorKeys;
}

cv::Mat FeaturesDetect::drawKeyFeatures(std::vector<cv::KeyPoint> vectorKeys, cv::Mat background) {
    cv::Mat out;
    cv::drawKeypoints(background, vectorKeys, out);
    return out;
}

void FeaturesDetect::changeDetectorMethod(std::string methodName) {
    featureDetector = initFeatureDetector(methodName);
}