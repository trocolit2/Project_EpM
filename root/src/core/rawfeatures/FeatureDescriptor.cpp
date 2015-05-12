/*
 * FeatureDescriptor.cpp
 *
 *  Created on: May 12, 2015
 *      Author: tiagotrocoli
 */

#include "FeatureDescriptor.h"
#include <iostream>

FeatureDescriptor::FeatureDescriptor(std::string descriptorName, std::string matcherName) {
    _descriptorFeature = initDescriptorExtractor(descriptorName);
    _descriptorMatcher = initDescriptorMatcher(matcherName);
}

std::vector<cv::DMatch> FeatureDescriptor::compute(cv::Mat image1, std::vector<cv::KeyPoint> keyPoints1, cv::Mat image2, std::vector<cv::KeyPoint> keyPoints2) {
    cv::Mat descriptor1, descriptor2;
    _descriptorFeature->compute(image1, keyPoints1, descriptor1);
    _descriptorFeature->compute(image1, keyPoints2, descriptor2);

    std::vector<cv::DMatch> matches;
    _descriptorMatcher->match(descriptor1, descriptor2, matches);
    return matches;
}

void FeatureDescriptor::changeMethod(std::string descriptorName, std::string matcherName) {
    _descriptorFeature = initDescriptorExtractor(descriptorName);
    _descriptorMatcher = initDescriptorMatcher(matcherName);
}

cv::Ptr<cv::DescriptorExtractor> FeatureDescriptor::initDescriptorExtractor(std::string methodName) {
    return cv::DescriptorExtractor::create(methodName);
}

cv::Ptr<cv::DescriptorMatcher> FeatureDescriptor::initDescriptorMatcher(std::string methodName) {
    return cv::DescriptorMatcher::create(methodName);
}

cv::Mat FeatureDescriptor::drawMatchImages(cv::Mat image1, std::vector<cv::KeyPoint> keyPoints1, cv::Mat image2, std::vector<cv::KeyPoint> keyPoints2, std::vector<cv::DMatch> matches) {
    cv::Mat out;
    cv::drawMatches(image1, keyPoints1, image2, keyPoints2, matches, out);
    return out;

}
