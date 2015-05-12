/*
 * FeatureDescriptor_test.cpp
 *
 *  Created on: May 12, 2015
 *      Author: tiagotrocoli
 */

#include <core/rawfeatures/FeatureDescriptor.h>
#include <core/rawfeatures/FeaturesDetect.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "FeatureDetect_test"

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "tools/TestTools.h"

bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints, const std::vector<cv::KeyPoint>& trainKeypoints, std::vector<cv::DMatch>& matches,
        cv::Mat& homography, cv::Mat src, cv::Mat frameImg, float reprojectionThreshold=3.0) {
    const int minNumberMatchesAllowed = 4;
    if (matches.size() < minNumberMatchesAllowed)
        return false;

    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> queryPoints(matches.size());
    std::vector<cv::Point2f> trainPoints(matches.size());
    for (size_t i = 0; i < matches.size(); i++) {
        queryPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
        trainPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
    }


    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(matches.size());
    homography = cv::findHomography(queryPoints, trainPoints, CV_FM_RANSAC, reprojectionThreshold, inliersMask);
    std::vector<cv::DMatch> inliers;
    for (size_t i = 0; i < inliersMask.size(); i++)
        if (inliersMask[i])
            inliers.push_back(matches[i]);

    matches.swap(inliers);
    cv::Mat homoShow;
    cv::drawMatches(src, queryKeypoints, frameImg, trainKeypoints, matches, homoShow, cv::Scalar::all(-1), CV_RGB(255, 255, 255), cv::Mat(), 2);
    imshow("homoShow", homoShow);
    return matches.size() > minNumberMatchesAllowed;

}

BOOST_AUTO_TEST_CASE(featureDetect_SimpleTestCase) {
    std::string path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_DINO_DATASET);
    cv::Mat imageDino1 = cv::imread(path + "dinoR0001.png");
    cv::imshow("DINO 1", imageDino1);
    cv::Mat imageDino2 = cv::imread(path + "dinoR0001.png");
    cv::imshow("DINO 2", imageDino2);

    std::string detectorName = std::string(METHOD_ADAPTED_PYRAMID) + std::string(METHOD_ORG);
    FeaturesDetect featureDetector(detectorName);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    keypoints1 = featureDetector.detector(imageDino1);
    keypoints2 = featureDetector.detector(imageDino2);
    cv::Mat out1 = featureDetector.drawKeyFeatures(keypoints1, imageDino1);
    cv::Mat out2 = featureDetector.drawKeyFeatures(keypoints2, imageDino2);

    cv::imshow("out 1", out1);
    cv::imshow("out 2", out2);
//    cv::waitKey();

    FeatureDescriptor featureDescriptor(METHOD_DESCRIPTOR_ORB, METHOD_MATCH_BRUTEFORCE_L2);
    std::vector<cv::DMatch> matchs = featureDescriptor.compute(imageDino1, keypoints1, imageDino2, keypoints2);

    cv::Mat outMatch = featureDescriptor.drawMatchImages(imageDino1, keypoints1, imageDino2, keypoints2, matchs);
    cv::imshow("matchs points", outMatch);

    cv::Mat homografy;
    refineMatchesWithHomography(keypoints1,keypoints2,matchs,homografy,imageDino1,imageDino2);

    cv::waitKey();
}
