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

#define METHOD_EXTRACTOR_ORG "ORB"
#define METHOD_EXTRACTOR_FAST "FAST"
#define METHOD_EXTRACTOR_STAR "STAR"
#define METHOD_EXTRACTOR_HARRIS "HARRIS"
#define METHOD_EXTRACTOR_BRISK "BRISK"
#define METHOD_EXTRACTOR_DENSE "Dense"
#define METHOD_EXTRACTOR_GFTT "GFTT"
#define METHOD_EXTRACTOR_MSER "MSER"
#define METHOD_EXTRACTOR_SIFT "SIFT"
#define METHOD_EXTRACTOR_SURF "SURF"

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
    FeaturesDetect(std::string methodName = METHOD_EXTRACTOR_FAST);
    std::vector<cv::KeyPoint> detector(cv::Mat image);
    void changeDetectorMethod(std::string method);

    cv::Mat drawKeyFeatures(std::vector<cv::KeyPoint>, cv::Mat background);

protected:
    cv::Ptr<cv::FeatureDetector> initFeatureDetector(std::string methodName) {
        return cv::FeatureDetector::create(methodName);
    }

    cv::Ptr<cv::FeatureDetector> _featureDetector;
};

#endif /* SOURCE_DIRECTORY__SRC_LIBRARY_FEATURESDETECT_H_ */
