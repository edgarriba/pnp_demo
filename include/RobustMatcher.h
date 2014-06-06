/*
 * RobustMatcher.h
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#ifndef ROBUSTMATCHER_H_
#define ROBUSTMATCHER_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class RobustMatcher {
public:
  RobustMatcher() : ratio_(0.65f), refineF_(true), confidence_(0.99), distance_(3.0)
  {
    // ORB is the default feature
    detector_ = new cv::OrbFeatureDetector();
    extractor_ = new cv::OrbDescriptorExtractor();
    matcher_ = new cv::BFMatcher(cv::NORM_HAMMING, false);

  }
  virtual ~RobustMatcher();

  // Set the feature detector
  void setFeatureDetector(cv::FeatureDetector* detect) {  detector_ = detect; }

  // Set the descriptor extractor
  void setDescriptorExtractor(cv::DescriptorExtractor* desc) { extractor_ = desc; }

  // Set the matcher
  void setDescriptorMatcher(cv::DescriptorMatcher* match) {  matcher_ = match; }

  // Set confidence level
  void setConfidenceLevel(double conf) { confidence_ = conf; }

  //Set MinDistanceToEpipolar
  void setMinDistanceToEpipolar( double dist) { distance_ = dist; }

  void computeKeyPoints( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
  {
    detector_->detect(image, keypoints);
  }

  void computeDescriptors( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
    {
      extractor_->compute(image, keypoints, descriptors);
    }


  //Set ratio
  void setRatio( float rat) { ratio_ = rat; }


  // Clear matches for which NN ratio is > than threshold
  // return the number of removed points
  // (corresponding entries being cleared,
  // i.e. size will be 0)
  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches);

  // Insert symmetrical matches in symMatches vector
  void symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                     const std::vector<std::vector<cv::DMatch> >& matches2,
                     std::vector<cv::DMatch>& symMatches );

  // Match feature points using symmetry test and RANSAC
  // returns fundemental matrix
  void robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                    std::vector<cv::KeyPoint>& keypoints_frame,
                    const std::vector<cv::KeyPoint>& keypoints_model,
                    const cv::Mat& descriptors_model );

  void crossCheckMatching ( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                            std::vector<cv::KeyPoint>& keypoints_frame,
                            const std::vector<cv::KeyPoint>& keypoints_model,
                            const cv::Mat& descriptors_model );

  void simpleMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                      std::vector<cv::KeyPoint>& keypoints_frame,
                      const std::vector<cv::KeyPoint>& keypoints_model,
                      const cv::Mat& descriptors_model );


private:
  // pointer to the feature point detector object
  cv::FeatureDetector* detector_;
  // pointer to the feature descriptor extractor object
  cv::DescriptorExtractor* extractor_;
  // pointer to the matcher object
  cv::DescriptorMatcher* matcher_;

  float ratio_; // max ratio between 1st and 2nd NN
  bool refineF_; // if true will refine the F matrix
  double distance_; // min distance to epipolar
  double confidence_; // confidence level (probability)
};

#endif /* ROBUSTMATCHER_H_ */
