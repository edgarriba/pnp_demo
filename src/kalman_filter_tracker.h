/*
 * kalman_filter_tracker.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Edgar Riba
 */

#ifndef KALMAN_FILTER_TRACKER_H_
#define KALMAN_FILTER_TRACKER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

class KalmanFilterTracker
{

public:
  /* Default constructor */
  explicit
  KalmanFilterTracker(const int nStates, const int nMeasurements, const int nInputs, const double dt, const int minInliersKalman);
  ~KalmanFilterTracker();

  void initKalman(const int nStates, const int nMeasurements, const int nInputs, const double dt);
  bool predictPose(const int nInliers, cv::Mat &translation, cv::Mat &rotation);

private:
  void updateMeasurements(const cv::Mat &translation_measured, const cv::Mat &rotation_measured);

  /** The Linear Kalmnan Filter object */
  cv::KalmanFilter kf_;
  /** Matrix with the measurements */
  cv::Mat measurements_;
  /** Threshold to update the measurements matrix */
  const int minInliersKalman_;
};


#endif /* KALMAN_FILTER_TRACKER_H_ */
