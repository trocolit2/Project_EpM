/*
 * FeaturesDetect.h
 *
 *  Created on: May 11, 2015
 *      Author: tiagotrocoli
 */

#ifndef SOURCE_DIRECTORY__SRC_LIBRARY_FEATURESDETECT_H_
#define SOURCE_DIRECTORY__SRC_LIBRARY_FEATURESDETECT_H_

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#define METHOD_ORG "ORB"
#define METHOD_FAST "FAST"
#define METHOD_STAR "STAR"
#define METHOD_HARRIS "HARRIS"
#define METHOD_BRISK "BRISK"
#define METHOD_DENSE "Dense"
#define METHOD_GFTT "GFTT"
#define METHOD_MSER "MSER"

#define METHOD_ADAPTED_GRID "Grid"
#define METHOD_ADAPTED_PYRAMID "Pyramid"
//#define METHOD_ADAPTED_DYNAMIC "Dynamic"

/*
 "FAST" – FastFeatureDetector
 "STAR" – StarFeatureDetector
 "SIFT" – SIFT (nonfree module)
 "SURF" – SURF (nonfree module)
 "ORB" – ORB
 "BRISK" – BRISK
 "MSER" – MSER
 "GFTT" – GoodFeaturesToTrackDetector
 "HARRIS" – GoodFeaturesToTrackDetector with Harris detector enabled
 "Dense" – DenseFeatureDetector
 "SimpleBlob" – SimpleBlobDetector

 link http://docs.opencv.org/modules/features2d/doc/common_interfaces_of_feature_detectors.html?highlight=featuredetector#featuredetector
 */

class FeaturesDetect {
public:
    FeaturesDetect(std::string methodName = METHOD_FAST);

    std::vector<cv::KeyPoint> detector(cv::Mat image);
    cv::Mat drawKeyFeatures(std::vector<cv::KeyPoint>, cv::Mat background);
    void changeDetectorMethod(std::string method);

protected:
    cv::Ptr<cv::FeatureDetector> initFeatureDetector(std::string methodName) {
        return cv::FeatureDetector::create(methodName);
    }

    cv::Ptr<cv::FeatureDetector> featureDetector;
};

#endif /* SOURCE_DIRECTORY__SRC_LIBRARY_FEATURESDETECT_H_ */
