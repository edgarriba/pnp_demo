/*
 * kalman_filter_tracker.h
 *
 *  Created on: Mar 13, 2015
 *      Author: Edgar Riba
 */

#include "kalman_filter_tracker.h"

#include "Utils.h"

using namespace cv; 

KalmanFilterTracker::KalmanFilterTracker(const int nStates, 
    const int nMeasurements, const int nInputs, const double dt, const int minInliersKalman) :
  kf_(),
  measurements_(nMeasurements, 1, CV_64F, 0.0),
  minInliersKalman_(minInliersKalman)
{
  initKalman(nStates, nMeasurements, nInputs, dt);
}


KalmanFilterTracker::~KalmanFilterTracker()
{
}


void KalmanFilterTracker::initKalman(const int nStates, const int nMeasurements, const int nInputs, const double dt)
{

  kf_.init(nStates, nMeasurements, nInputs, CV_64F);         // init Kalman Filter

  setIdentity(kf_.processNoiseCov, Scalar::all(1e-5));       // set process noise
  setIdentity(kf_.measurementNoiseCov, Scalar::all(1e-2));   // set measurement noise
  setIdentity(kf_.errorCovPost, Scalar::all(1));             // error covariance


                     /** DYNAMIC MODEL **/

  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

  // position
  kf_.transitionMatrix.at<double>(0,3) = dt;
  kf_.transitionMatrix.at<double>(1,4) = dt;
  kf_.transitionMatrix.at<double>(2,5) = dt;
  kf_.transitionMatrix.at<double>(3,6) = dt;
  kf_.transitionMatrix.at<double>(4,7) = dt;
  kf_.transitionMatrix.at<double>(5,8) = dt;
  kf_.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  kf_.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  kf_.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

  // orientation
  kf_.transitionMatrix.at<double>(9,12) = dt;
  kf_.transitionMatrix.at<double>(10,13) = dt;
  kf_.transitionMatrix.at<double>(11,14) = dt;
  kf_.transitionMatrix.at<double>(12,15) = dt;
  kf_.transitionMatrix.at<double>(13,16) = dt;
  kf_.transitionMatrix.at<double>(14,17) = dt;
  kf_.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  kf_.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  kf_.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);


           /** MEASUREMENT MODEL **/

  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

  kf_.measurementMatrix.at<double>(0,0) = 1;  // x
  kf_.measurementMatrix.at<double>(1,1) = 1;  // y
  kf_.measurementMatrix.at<double>(2,2) = 1;  // z
  kf_.measurementMatrix.at<double>(3,9) = 1;  // roll
  kf_.measurementMatrix.at<double>(4,10) = 1; // pitch
  kf_.measurementMatrix.at<double>(5,11) = 1; // yaw


}


bool KalmanFilterTracker::predictPose(const int nInliers, cv::Mat &translation, cv::Mat &rotation)
{
  bool good_measurement = false;

  if(nInliers > minInliersKalman_)
  {
    updateMeasurements(translation, rotation);
    good_measurement = true;
  }

  // First predict, to update the internal statePre variable
  Mat prediction = kf_.predict();

  // The "correct" phase that is going to use the predicted value and our measurement
  Mat estimated = kf_.correct(measurements_);

  // Estimated translation
  translation.at<double>(0) = estimated.at<double>(0);
  translation.at<double>(1) = estimated.at<double>(1);
  translation.at<double>(2) = estimated.at<double>(2);

  // Estimated euler angles
  Mat eulers_estimated(3, 1, CV_64F);
  eulers_estimated.at<double>(0) = estimated.at<double>(9);
  eulers_estimated.at<double>(1) = estimated.at<double>(10);
  eulers_estimated.at<double>(2) = estimated.at<double>(11);

  // Convert estimated quaternion to rotation matrix
  rotation = euler2rot(eulers_estimated);

  return good_measurement;
}


void KalmanFilterTracker::updateMeasurements(const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{

  // Convert rotation matrix to euler angles
  Mat measured_eulers(3, 1, CV_64F);
  measured_eulers = rot2euler(rotation_measured);

  // Set measurement to predict
  measurements_.at<double>(0) = translation_measured.at<double>(0); // x
  measurements_.at<double>(1) = translation_measured.at<double>(1); // y
  measurements_.at<double>(2) = translation_measured.at<double>(2); // z
  measurements_.at<double>(3) = measured_eulers.at<double>(0);      // roll
  measurements_.at<double>(4) = measured_eulers.at<double>(1);      // pitch
  measurements_.at<double>(5) = measured_eulers.at<double>(2);      // yaw

}

