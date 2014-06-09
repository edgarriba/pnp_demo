#include <iostream>
#include <time.h>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/video/tracking.hpp"

#include "Mesh.h"
#include "Model.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "ModelRegistration.h"
#include "Utils.h"

std::string img_path = "../Data/resized_IMG_3875.JPG";
std::string video_path = "../Data/box.mp4";
std::string ply_read_path = "../Data/box.ply";
std::string yml_read_path = "../Data/box.yml";

/*
 * Set up the intrinsic camera parameters: UVC WEBCAM
 */
double f = 55;
double sx = 22.3, sy = 14.9;
double width = 640, height = 480;
double params_WEBCAM[] = { width*f/sx,   // fx
                          height*f/sy,  // fy
                          width/2,      // cx
                          height/2};    // cy

/*
 * Set up some basic colors
 */
cv::Scalar red(0, 0, 255);
cv::Scalar green(0,255,0);
cv::Scalar blue(255,0,0);
cv::Scalar yellow(0,255,255);


RobustMatcher rmatcher;
int numKeyPoints = 10000;
int ratio = 80;

// RANSAC
int iterationsCount = 1000; //100 // increase
int reprojectionError = 30; //8.0 // 2.0-3.0
int minInliersCount = 30; //100 // 20-30

int min_inliers = 0; // 20
int min_confidence = 0; // 30


void onRatioTest( int, void* )
{
  rmatcher.setRatio((double)ratio/100);
}
void onNumKeyPoints( int, void* )
{
  cv::FeatureDetector* detector = new cv::OrbFeatureDetector(numKeyPoints);
  rmatcher.setFeatureDetector(detector);
}

int main(int, char**)
{

  std::cout << "!!!Hello Detection!!!" << std::endl; // prints !!!Hello World!!!

  PnPProblem pnp_detection(params_WEBCAM);
  PnPProblem pnp_detection_est(params_WEBCAM);

  Model model;
  model.load(yml_read_path); // load a mesh given the *.ply file path

  Mesh mesh;
  mesh.load(ply_read_path); // load the 3D textured object model

  // Instantiate robust Matcher: detector, extractor, matcher
  cv::FeatureDetector* detector = new cv::OrbFeatureDetector(numKeyPoints);
  cv::DescriptorExtractor* extractor = new cv::OrbDescriptorExtractor;
  cv::DescriptorMatcher* matcher = new cv::BFMatcher(cv::NORM_HAMMING, false);
  rmatcher.setFeatureDetector(detector);
  rmatcher.setDescriptorExtractor(extractor);
  rmatcher.setDescriptorMatcher(matcher);
  rmatcher.setRatio(ratio);

  // Instantiate Kalman Filter

  int dynamicsParams = 7;
  int measurementParams = 7;

  cv::KalmanFilter KF;
  cv::Mat measurement = cv::Mat_<float>::zeros(measurementParams,1);
  measurement.setTo(cv::Scalar(0));

  //init Kalman
  KF.init(dynamicsParams, measurementParams);

  cv::setIdentity(KF.measurementMatrix);
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));

  std::cout << "A " << std::endl << KF.transitionMatrix << std::endl;
  std::cout << "C " << std::endl << KF.measurementMatrix << std::endl;

  // Open the image to register
  cv::Mat img_in = cv::imread(img_path, cv::IMREAD_COLOR);

  if (!img_in.data)
  {
   std::cout << "Could not open or find the image" << std::endl;
   return -1;
  }

  // Get the MODEL INFO
  std::vector<cv::Point2f> list_points2d_model = model.get_points2d_in();
  std::vector<cv::Point3f> list_points3d_model = model.get_points3d();
  std::vector<cv::KeyPoint> keypoints_model = model.get_keypoints();
  cv::Mat descriptors_model = model.get_descriptors();

  cv::BFMatcher match(cv::NORM_HAMMING, true);
  match.add(descriptors_model);
  match.train();

  cv::ORB orb(numKeyPoints);

  // Model variance
  double model_variance = get_variance(list_points3d_model);

  // Create & Open Window
  cv::namedWindow("REAL TIME DEMO", cv::WINDOW_KEEPRATIO);

  // TUNING VALUES
  cv::createTrackbar("Num. Keypoints", "REAL TIME DEMO", &numKeyPoints, 15000, onNumKeyPoints);
  cv::createTrackbar("Ratio test", "REAL TIME DEMO", &ratio, 200, onRatioTest);
  cv::createTrackbar("Iterations Count", "REAL TIME DEMO", &iterationsCount, 3000);
  cv::createTrackbar("Reprojection Error (div10)", "REAL TIME DEMO", &reprojectionError, 200);
  cv::createTrackbar("RANSAC Inliers", "REAL TIME DEMO", &minInliersCount, 150);
  cv::createTrackbar("Pose Inliers", "REAL TIME DEMO", &min_inliers, 200);
  cv::createTrackbar("Pose Confidence", "REAL TIME DEMO", &min_confidence, 100);

  //cv::VideoCapture cap(0); // open the default camera
  cv::VideoCapture cap("../Data/box.mp4"); // open the recorded video
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  // start and end times
  time_t start, end;

  // fps calculated using number of frames / seconds
  double fps;

  // frame counter
  int counter = 0;

  // floating point seconds elapsed since start
  double sec;

  // start the clock
  time(&start);

  // Loop videostream
  for(;;)
  {
    cv::Mat frame, frame_vis;
    cap >> frame; // get a new frame from camera
    frame_vis = frame.clone();

    // Robust Match
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::KeyPoint> keypoints_scene;

    //rmatcher.simpleMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
    //rmatcher.crossCheckMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
    rmatcher.robustMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);

    cv::Mat inliers_idx;
    std::vector<cv::DMatch> matches_inliers;
    std::vector<cv::KeyPoint> keypoints_inliers;
    std::vector<cv::Point2f> list_points2d_inliers;
    std::vector<cv::Point3f> list_points3d_inliers;

    if(good_matches.size() > 0)
    {
      // -- Step 5: Find out the 2D/3D correspondences
      std::vector<cv::Point3f> list_points3d_model_match;
      std::vector<cv::Point2f> list_points2d_scene_match;
      for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
      {
        cv::Point3f point3d_model = list_points3d_model[ good_matches[match_index].trainIdx ];
        cv::Point2f point2d_scene = keypoints_scene[ good_matches[match_index].queryIdx ].pt;
        list_points3d_model_match.push_back(point3d_model);
        list_points2d_scene_match.push_back(point2d_scene);
      }

      // -- Step 6: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match,
                                       cv::EPNP, inliers_idx,
                                       iterationsCount, (double)reprojectionError/10, minInliersCount );

      // -- Step 7: Catch the inliers keypoints
      for(int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
      {
        int n = inliers_idx.at<int>(inliers_index);
        cv::Point2f point2d = list_points2d_scene_match[n];
        cv::Point3f point3d = list_points3d_model_match[n];
        list_points2d_inliers.push_back(point2d);
        list_points3d_inliers.push_back(point3d);

        unsigned int match_index = 0;
        bool is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
        while ( !is_equal && match_index < good_matches.size() )
        {
          match_index++;
          is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
        }

        matches_inliers.push_back(good_matches[match_index]);
        keypoints_inliers.push_back(keypoints_scene[good_matches[match_index].queryIdx]);
      }

      // -- Step 8: Calculate covariance
      double detection_variance = get_variance(list_points3d_inliers);
      double confidence = (detection_variance/model_variance)*100;
      drawConfidence(frame_vis, confidence, yellow);

      // -- Step 9: Draw pose
      if( !isnan(confidence) && inliers_idx.rows >= min_inliers && confidence > min_confidence)
      {
        double l = 5;
        std::vector<cv::Point2f> pose_points2d;
        pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,0,0)));
        pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(l,0,0)));
        pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,l,0)));
        pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,0,l)));
        draw3DCoordinateAxes(frame, pose_points2d);

        drawObjectMesh(frame_vis, &mesh, &pnp_detection, green);


        // -- Step 10: Kalman Filter

       // First predict, to update the internal statePre variable
       cv::Mat prediction = KF.predict();

       // Get measures
       cv::Mat measured_translation = pnp_detection.get_t_matrix();
       cv::Mat measured_rotation = pnp_detection.get_R_matrix();

       // Convert rotation matrix to quaternion
       cv::Mat measured_quaternion = rot2quat(measured_rotation);

       // Set measurement to predict
       measurement.at<float>(0) = (float)measured_translation.at<double>(0); // x
       measurement.at<float>(1) = (float)measured_translation.at<double>(1); // y
       measurement.at<float>(2) = (float)measured_translation.at<double>(2); // z
       measurement.at<float>(3) = (float)measured_quaternion.at<double>(0);  // qw
       measurement.at<float>(4) = (float)measured_quaternion.at<double>(1);  // qx
       measurement.at<float>(5) = (float)measured_quaternion.at<double>(2);  // qy
       measurement.at<float>(6) = (float)measured_quaternion.at<double>(3);  // qy

       // The "correct" phase that is going to use the predicted value and our measurement
       cv::Mat estimated = KF.correct(measurement);

       // Estimated translation
       cv::Mat translation_est(3, 1, CV_64F);
       translation_est.at<double>(0) = (double)estimated.at<float>(0);
       translation_est.at<double>(1) = (double)estimated.at<float>(1);
       translation_est.at<double>(2) = (double)estimated.at<float>(2);

       // Estimated quaternion
       cv::Mat quaternion_est(4, 1, CV_64F);
       quaternion_est.at<double>(0) = (double)estimated.at<float>(3);
       quaternion_est.at<double>(1) = (double)estimated.at<float>(4);
       quaternion_est.at<double>(2) = (double)estimated.at<float>(5);
       quaternion_est.at<double>(3) = (double)estimated.at<float>(6);

       // Convert quaternion to rotation matrix
       cv::Mat rotation_est = quat2rot(quaternion_est);

       // Set estimated projection matrix
       pnp_detection_est.set_P_matrix(rotation_est, translation_est);


      }

    }

    drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow);

    // FRAME RATE

    // see how much time has elapsed
    time(&end);

    // calculate current FPS
    ++counter;
    sec = difftime (end, start);

    fps = counter / sec;

    drawFPS(frame_vis, fps, yellow); // frame ratio


    // -- Step X: Draw correspondences

    // Switched the order due to a opencv bug
    //cv::drawMatches( img_in, keypoints_model,  // model image
    //                 frame, keypoints_scene, // scene image
    //                 matches_inliers, frame_vis, red, blue);

    // -- Step X: Draw some text for debugging purpose

    // Draw some debug text
    int inliers_int = inliers_idx.rows;
    int outliers_int = good_matches.size() - inliers_int;
    std::string inliers_str = boost::lexical_cast< std::string >(inliers_int);
    std::string outliers_str = boost::lexical_cast< std::string >(outliers_int);
    std::string n = boost::lexical_cast< std::string >(good_matches.size());
    std::string text = "Found " + inliers_str + " of " + n + " matches";
    std::string text2 = "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

    drawText(frame_vis, text, green);
    drawText2(frame_vis, text2, red);

    cv::imshow("REAL TIME DEMO", frame_vis);
    if(cv::waitKey(30) >= 0) break;
  }

  // Close and Destroy Window
  cv::destroyWindow("REAL TIME DEMO");

  std::cout << "GOODBYE ..." << std::endl;

}
