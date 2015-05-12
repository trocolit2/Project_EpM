/*
 * FeatureDescriptor.h
 *
 *  Created on: May 12, 2015
 *      Author: tiagotrocoli
 */

#ifndef SRC_LIBRARY_RAWFEATURES_FEATUREDESCRIPTOR_H_
#define SRC_LIBRARY_RAWFEATURES_FEATUREDESCRIPTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>

#define METHOD_MATCH_BRUTEFORCE_L2 "BruteForce"
#define METHOD_MATCH_BRUTEFORCE_L1 "BruteForce-L1"
#define METHOD_MATCH_BRUTEFORCE_HAMMING "BruteForce-Hamming"
#define METHOD_MATCH_BRUTEFORCE_HAMMING_2 "BruteForce-Hamming(2)"
#define METHOD_MATCH_FLANNBASED "FlannBased"

/*
 BruteForce (it uses L2 )
 BruteForce-L1
 BruteForce-Hamming
 BruteForce-Hamming(2)
 FlannBased

 link http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#descriptormatcher-create
 */

#define METHOD_DESCRIPTOR_BRIEF "BriefDescriptorExtractor"
#define METHOD_DESCRIPTOR_BRISK "BRISK"
#define METHOD_DESCRIPTOR_ORB "ORB"
#define METHOD_DESCRIPTOR_FREAK "FREAK"

/*
 "SIFT" – SIFT
 "SURF" – SURF
 "BRIEF" – BriefDescriptorExtractor
 "BRISK" – BRISK
 "ORB" – ORB
 "FREAK" – FREAK

 link http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_descriptor_extractors.html?highlight=descriptorextractor#descriptorextractor-create
 */

class FeatureDescriptor {
public:
    FeatureDescriptor(std::string descriptorName, std::string matcherName);
    std::vector<cv::DMatch> compute(cv::Mat image1, std::vector<cv::KeyPoint> keyPoints1, cv::Mat image2, std::vector<cv::KeyPoint> keyPoints2);
    void changeMethod(std::string descriptorName, std::string matcherName);

    cv::Mat drawMatchImages(cv::Mat image1, std::vector<cv::KeyPoint> keyPoints1, cv::Mat image2, std::vector<cv::KeyPoint> keyPoints2, std::vector<cv::DMatch> matches);

protected:

    cv::Ptr<cv::DescriptorExtractor> initDescriptorExtractor(std::string methodName);
    cv::Ptr<cv::DescriptorExtractor> _descriptorFeature;

    cv::Ptr<cv::DescriptorMatcher> initDescriptorMatcher(std::string methodName);
    cv::Ptr<cv::DescriptorMatcher> _descriptorMatcher;
};

#endif /* SRC_LIBRARY_RAWFEATURES_FEATUREDESCRIPTOR_H_ */
