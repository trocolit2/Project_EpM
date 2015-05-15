/*
 * FeatureDescriptor_test.cpp
 *
 *  Created on: May 12, 2015
 *      Author: tiagotrocoli
 */

#include <core/rawfeatures/FeatureDescriptor.h>
#include <core/rawfeatures/FeaturesDetect.h>
#include <core/rawfeatures/FeaturesTools.h>

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

BOOST_AUTO_TEST_CASE(featureDetect_SimpleTestCase) {
    std::string path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_DINO_DATASET); // dinoR0001.png
//    std::string path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_STEREO_DATASET); // test3/dir.ppm
    cv::Mat imageDino1 = cv::imread(path + "dinoR0001.png");
    cv::imshow("DINO 1", imageDino1);
    cv::Mat imageDino2 = cv::imread(path + "dinoR0002.png");
    cv::imshow("DINO 2", imageDino2);

    std::string detectorName = std::string(METHOD_ADAPTED_PYRAMID) + std::string(METHOD_EXTRACTOR_SIFT);
    FeaturesDetect featureDetector(detectorName);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    keypoints1 = featureDetector.detector(imageDino1);
    keypoints2 = featureDetector.detector(imageDino2);
    cv::Mat out1 = featureDetector.drawKeyFeatures(keypoints1, imageDino1);
    cv::Mat out2 = featureDetector.drawKeyFeatures(keypoints2, imageDino2);

    cv::imshow("out 1", out1);
    cv::imshow("out 2", out2);

    FeatureDescriptor featureDescriptor(METHOD_DESCRIPTOR_SIFT, METHOD_MATCH_BRUTEFORCE_L2);
    std::vector<cv::DMatch> matchs = featureDescriptor.compute(imageDino1, keypoints1, imageDino2, keypoints2);

    cv::Mat outMatch = featureDescriptor.drawMatchImages(imageDino1, keypoints1, imageDino2, keypoints2, matchs);
    cv::imshow("matchs points", outMatch);

    FeaturesTools::refineMatchesWithHomography(keypoints1, keypoints2, &matchs, 20);
    cv::drawMatches(imageDino1, keypoints1, imageDino2, keypoints2, matchs, outMatch, cv::Scalar::all(-1), CV_RGB(255, 255, 255), cv::Mat(), 2);
    cv::imshow("HOMOGRAFY REFINE", outMatch);

    cv::Mat fundamentalMat = FeaturesTools::calcFundamentalMatrix(keypoints1, keypoints2, &matchs);
    cv::drawMatches(imageDino1, keypoints1, imageDino2, keypoints2, matchs, outMatch, cv::Scalar::all(-1), CV_RGB(255, 255, 255), cv::Mat(), 2);
    cv::imshow("fundamentalMatrixSelection", outMatch);

    FeaturesTools::calcEpiplesFromImages(fundamentalMat, 0, 0);

    cv::waitKey();
}
