/*
 * FeaturesTools.cpp
 *
 *  Created on: May 14, 2015
 *      Author: tiagotrocoli
 */

#include "FeaturesTools.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

#define MIN_SAMPLE_FUNDAMENTAL_MATRIX 8

void selectMatches(std::vector<cv::DMatch> *matches, std::vector<unsigned char> inlines) {

    std::vector<cv::DMatch> inliers;
    for (size_t i = 0; i < inlines.size(); i++)
        if (inlines[i])
            inliers.push_back((*matches)[i]);
    matches->swap(inliers);
}

void splitMatchesInVectorPoints(std::vector<cv::KeyPoint> queryKeypoints, std::vector<cv::KeyPoint> trainKeypoints, std::vector<cv::Point2f> *queryPoints, std::vector<cv::Point2f> *trainPoints,
        std::vector<cv::DMatch> matches) {
    std::vector<cv::Point2f> queryPointsLocal(matches.size());
    std::vector<cv::Point2f> trainPointsLocal(matches.size());

    for (size_t i = 0; i < matches.size(); i++) {
        queryPointsLocal[i] = queryKeypoints[matches[i].queryIdx].pt;
        trainPointsLocal[i] = trainKeypoints[matches[i].trainIdx].pt;
    }

    (*trainPoints) = trainPointsLocal;
    (*queryPoints) = queryPointsLocal;
}

cv::Mat FeaturesTools::calcFundamentalMatrix(std::vector<cv::KeyPoint> queryKeypoints, std::vector<cv::KeyPoint> trainKeypoints, std::vector<cv::DMatch> *matches) {
    if (matches->size() < MIN_SAMPLE_FUNDAMENTAL_MATRIX)
        return cv::Mat();

    std::vector<cv::Point2f> queryPoints, trainPoints;
    splitMatchesInVectorPoints(queryKeypoints, trainKeypoints, &queryPoints, &trainPoints, (*matches));

    std::vector<unsigned char> inlines(matches->size());
    cv::Mat fundamentalMat = cv::findFundamentalMat(queryPoints, trainPoints, inlines, cv::FM_RANSAC, 1);

    selectMatches(matches, inlines);
    return fundamentalMat;
}

cv::Mat FeaturesTools::refineMatchesWithHomography(std::vector<cv::KeyPoint> queryKeypoints, std::vector<cv::KeyPoint> trainKeypoints, std::vector<cv::DMatch> *matches, float reprojectionThreshold) {
    if (matches->size() < MIN_SAMPLE_FUNDAMENTAL_MATRIX)
        return cv::Mat();

    std::vector<cv::Point2f> queryPoints, trainPoints;
    splitMatchesInVectorPoints(queryKeypoints, trainKeypoints, &queryPoints, &trainPoints, (*matches));

    std::vector<unsigned char> inlines(matches->size());
    cv::Mat homography = cv::findHomography(queryPoints, trainPoints, CV_FM_RANSAC, reprojectionThreshold, inlines);

    selectMatches(matches, inlines);
    return homography;
}

void FeaturesTools::calcEpiplesFromImages(cv::Mat fundamentalMat, cv::Point2f* epipole1, cv::Point2f* epipole2) {

    cv::Mat1f mat1 = cv::Mat1f::ones(3, 3);
    cv::Mat1f mat2 = cv::Mat1f::ones(3, 3);
    cv::Mat transpose, epipoleLocal1, epipoleLocal2;

    cv::transpose(fundamentalMat, transpose);

    mat1.col(0) = fundamentalMat.col(0);
    mat1.col(1) = fundamentalMat.col(2);

//    transpose.col(0).copyTo(mat2.col(0));
//    transpose.col(2).copyTo(mat2.col(1));

    cv::SVD::solveZ(cv::Mat(mat1), epipoleLocal1);
//    cv::SVD::solveZ(mat2, epipoleLocal2);

    std::cout << "FUNDAMENTAL MATRIX\n" << fundamentalMat << std::endl;
    std::cout << "MAT1\n" << mat1 << std::endl;
//    std::cout << "MAT2\n" << mat2 << std::endl;

//    std::cout << "epipoleLocal1\n" << epipoleLocal1 << std::endl;
//    std::cout << "epipoleLocal2\n" << epipoleLocal2 << std::endl;

}
