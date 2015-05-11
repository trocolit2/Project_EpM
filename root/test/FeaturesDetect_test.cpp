/*
 * FeaturesDetect_test.cpp
 *
 *  Created on: May 11, 2015
 *      Author: tiagotrocoli
 */

#include <library/FeaturesDetect.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "FeatureDetect_test"

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "tools/TestTools.h"

#define PATH_DINO_DATASET "resource/dataset/dinoRing/"

std::vector<std::string> listOfMethod() {

    std::vector<std::string> vector;
    vector.push_back(METHOD_ORG);
    vector.push_back(METHOD_FAST);
    vector.push_back(METHOD_GFTT);

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
    vector.push_back(METHOD_ADAPTED_GRID);
    vector.push_back(METHOD_ADAPTED_PYRAMID);
    return vector;
}

BOOST_AUTO_TEST_CASE(featureDetect_SimpleTestCase) {

    std::string path = std::string(PATH_RELATIVE_ROOT_TESTBIN) + std::string(PATH_DINO_DATASET);
    cv::Mat image = cv::imread(path + "dinoR0001.png");
    cv::imshow("DINO", image);

    std::vector<std::string> listMethods = listOfMethod();
    std::vector<std::string> adaptedMethods = adapterOfMethod();

    for (uint j = 0; j < adaptedMethods.size(); ++j)
        for (uint i = 0; i < listMethods.size(); ++i) {
            std::string methodName = adaptedMethods[j] + listMethods[i];
            FeaturesDetect featureDetector(methodName);
            cv::vector<cv::KeyPoint> vectorKeys = featureDetector.detector(image.clone());
            cv::Mat out = featureDetector.drawKeyFeatures(vectorKeys, image);
            cv::imshow(methodName, out);
            cv::waitKey();
        }

}
