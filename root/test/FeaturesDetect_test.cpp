/*
 * FeaturesDetect_test.cpp
 *
 *  Created on: May 11, 2015
 *      Author: tiagotrocoli
 */

#include <core/rawfeatures/FeaturesDetect.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "FeatureDetect_test"

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "tools/TestTools.h"


std::vector<std::string> listOfMethod() {

    std::vector<std::string> vector;
    vector.push_back(METHOD_EXTRACTOR_SIFT);
    vector.push_back(METHOD_EXTRACTOR_SURF);
    vector.push_back(METHOD_EXTRACTOR_ORG);
    vector.push_back(METHOD_EXTRACTOR_FAST);
    vector.push_back(METHOD_EXTRACTOR_GFTT);

    // just few points..
//    vector.push_back(METHOD_HARRIS);
//    // too slow
//    vector.push_back(METHOD_BRISK);
//    // bad results below
//    vector.push_back(METHOD_STAR);
//    vector.push_back(METHOD_DENSE);
//    vector.push_back(METHOD_MSER);
    return vector;
}

std::vector<std::string> adapterOfMethod() {

    std::vector<std::string> vector;
    vector.push_back("");
    vector.push_back(METHOD_ADAPTED_GRID);
    vector.push_back(METHOD_ADAPTED_PYRAMID);
    return vector;
}

BOOST_AUTO_TEST_CASE(featureDetect_SimpleTestCase) {

    std::string path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_DINO_DATASET);
    cv::Mat imageDino = cv::imread(path + "dinoR0001.png");
    cv::imshow("DINO", imageDino);

    path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_TEMPLE_DATASET);
    cv::Mat imageTemple = cv::imread(path + "templeR0001.png");
    cv::imshow("temple", imageTemple);

    std::vector<std::string> listMethods = listOfMethod();
    std::vector<std::string> adaptedMethods = adapterOfMethod();

    for (uint j = 0; j < adaptedMethods.size(); ++j)
        for (uint i = 0; i < listMethods.size(); ++i) {
            std::string methodName = adaptedMethods[j] + listMethods[i];
            FeaturesDetect featureDetector(methodName);
            cv::vector<cv::KeyPoint> vectorKeysDino = featureDetector.detector(imageDino.clone());
            cv::vector<cv::KeyPoint> vectorKeysTemple = featureDetector.detector(imageTemple.clone());
            cv::Mat outDino = featureDetector.drawKeyFeatures(vectorKeysDino, imageDino);
            cv::Mat outTemple = featureDetector.drawKeyFeatures(vectorKeysTemple, imageTemple);
//            cv::imshow(methodName + " DINO", outDino);
//            cv::imshow(methodName + " TEMPLE", outTemple);
//            cv::waitKey();
        }

}
