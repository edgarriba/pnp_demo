#include <iostream>
#include <time.h>
#include <boost/lexical_cast.hpp>

#include "cv.h"
#include "highgui.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
//std::string img_path = "../Data/box_test.jpg";

std::string video_path = "../Data/box3.mp4";     // video
std::string ply_read_path = "../Data/box.ply";   // mesh
std::string yml1_read_path = "../Data/box1.yml"; // 3dpts + descriptors
std::string yml2_read_path = "../Data/box2.yml";
std::string yml3_read_path = "../Data/box3.yml";

/*
 * Set up the intrinsic camera parameters: UVC WEBCAM
 */

double f = 50;
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
int numKeyPoints = 2500; // 2500
int ratio = 80; // 80

// RANSAC
int iterationsCount = 2000; //2000 // increase
int reprojectionError = 20; //2.0 // 2.0-3.0
int minInliersCount = 80; //80 // 20-30

int min_inliers = 20; // 20
int min_confidence = 5; // 5


void onRatioTest( int, void* )
{
  rmatcher.setRatio((double)ratio/100);
}
void onNumKeyPoints( int, void* )
{
  cv::FeatureDetector * detector = new cv::OrbFeatureDetector(numKeyPoints);
  rmatcher.setFeatureDetector(detector);
}

double mean(const std::vector<double> data)
{
  double mean = 0;
  for (int i = 0; i < data.size(); ++i) {
    mean += data[i];
  }
  return mean /= data.size();
}

int main(int, char**)
{

  std::cout << "!!!Hello Detection!!!" << std::endl;

  PnPProblem pnp_detection(params_WEBCAM);
  PnPProblem pnp_detection_est(params_WEBCAM);

  Model model;
  model.load(yml1_read_path); // load a mesh given the *.ply file path
  model.load(yml2_read_path); // load a mesh given the *.ply file path
  model.load(yml3_read_path); // load a mesh given the *.ply file path

  Mesh mesh;
  mesh.load(ply_read_path); // load the 3D textured object model

  // Instantiate robust Matcher: detector, extractor, matcher
  cv::FeatureDetector * detector = new cv::OrbFeatureDetector(numKeyPoints);
  bool crossMatching = true;
  cv::DescriptorMatcher * matcher = new cv::BFMatcher(cv::NORM_HAMMING, crossMatching);
  rmatcher.setFeatureDetector(detector);
  rmatcher.setDescriptorMatcher(matcher);
  rmatcher.setRatio(ratio);

  cv::Mat R_TEST(3,3,CV_64FC1);
  R_TEST.at<double>(0,0) = 0.7666923627714876;
  R_TEST.at<double>(0,1) = -0.6379407131990361;
  R_TEST.at<double>(0,2) = -0.07221126858725105;

  R_TEST.at<double>(1,0) = -0.403508228064979;
  R_TEST.at<double>(1,1) = -0.3913261702837051;
  R_TEST.at<double>(1,2) = -0.827070092758135;

  R_TEST.at<double>(2,0) = 0.4993635256521405;
  R_TEST.at<double>(2,1) = 0.6632461646283333;
  R_TEST.at<double>(2,2) = -0.5574411129025829;


  /*// Instantiate Kalman Filter

  int nStates = 13, nMeasurements = 7, nInputs = 0;

  cv::KalmanFilter KF(nStates, nMeasurements, nInputs, CV_64F);
  cv::Mat measurement(nMeasurements, 1, CV_64F); measurement.setTo(cv::Scalar(0));

  //init Kalman
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));

  double dt = 0.5; // 1/fps -- 1/8

  // dynamic model
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);

  // measurement model
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // q1
  KF.measurementMatrix.at<double>(4,10) = 1; // q2
  KF.measurementMatrix.at<double>(5,11) = 1; // q3
  KF.measurementMatrix.at<double>(6,12) = 1; // q4

  std::cout << "A " << std::endl << KF.transitionMatrix << std::endl;
  std::cout << "C " << std::endl << KF.measurementMatrix << std::endl;*/


  // Initiate alpha-beta filter

  double dt = 0.5;
  double a = 1.5;
  double b = 0.5;

  // position
  cv::Mat xk_1 = cv::Mat::zeros(3, 1, CV_64F); // x,y,z
  cv::Mat vk_1 = cv::Mat::zeros(3, 1, CV_64F); // vx,vy,vz

  cv::Mat xk(3, 1, CV_64F); // x,y,z
  cv::Mat vk(3, 1, CV_64F); // vx,vy,vz

  cv::Mat r_xk(3, 1, CV_64F); // residual error
  cv::Mat xm(3, 1, CV_64F); // measurement position

  // orientation
  cv::Mat qk_1 = cv::Mat::zeros(3, 1, CV_64F); // roll, pitch, yaw
  cv::Mat wk_1 = cv::Mat::zeros(3, 1, CV_64F); // vroll, vpitch, vyaw

  cv::Mat qk(3, 1, CV_64F); // roll, pitch, yaw
  cv::Mat wk(3, 1, CV_64F); // vroll, vpitch, vyaw

  cv::Mat r_qk(3, 1, CV_64F); // residual error
  cv::Mat qm(3, 1, CV_64F); // measurement quaternion


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

  // Model variance
  double model_variance = get_variance(list_points3d_model);

  // Create & Open Window
  cv::namedWindow("REAL TIME DEMO", CV_WINDOW_KEEPRATIO);

  // TUNING VALUES
  cv::createTrackbar("Num. Keypoints", "REAL TIME DEMO", &numKeyPoints, 15000, onNumKeyPoints);
  cv::createTrackbar("Ratio test", "REAL TIME DEMO", &ratio, 200, onRatioTest);
  cv::createTrackbar("Iterations Count", "REAL TIME DEMO", &iterationsCount, 3000);
  cv::createTrackbar("Reprojection Error (div10)", "REAL TIME DEMO", &reprojectionError, 200);
  cv::createTrackbar("RANSAC Inliers", "REAL TIME DEMO", &minInliersCount, 150);
  cv::createTrackbar("Pose Inliers", "REAL TIME DEMO", &min_inliers, 200);
  cv::createTrackbar("Pose Confidence", "REAL TIME DEMO", &min_confidence, 100);

  //cv::VideoCapture cap(0); // open the default camera
  cv::VideoCapture cap(video_path); // open the recorded video
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
  double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

  cv::Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
  cv::VideoWriter oVideoWriter ("../Data/MyVideo.avi", 0, 8, frameSize, true); //initialize the VideoWriter object

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

  double tstart2, tstop2, ttime2; // algorithm metrics
  double tstart, tstop, ttime; // algorithm metrics

  cv::Mat frame, frame_vis;
  std::vector<double> mean_match, mean_ransac, mean_stuff, mean_frame;

  // Loop videostream
  while(cap.read(frame) && cv::waitKey(30) != 27)
  {

    tstart2 = (double)clock()/CLOCKS_PER_SEC;

    //cap >> frame; // get a new frame from camera
    frame_vis = frame.clone();

    // Robust Match
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors_scene;

    tstart = (double)clock()/CLOCKS_PER_SEC;


    if(crossMatching)
    {
      rmatcher.crossOpenCVMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
    }
    else
    {
      //rmatcher.crossMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
      rmatcher.robustMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
    }

    tstop = (double)clock()/CLOCKS_PER_SEC;
    ttime = tstop-tstart; /*ttime is how long your code run */
    std::cout << "Time robustMatch: " << ttime*1000 << "ms" << std::endl;
    mean_match.push_back(ttime);

    double confidence;
    cv::Mat inliers_idx;
    std::vector<cv::DMatch> matches_inliers;
    std::vector<cv::KeyPoint> keypoints_inliers;
    std::vector<cv::Point2f> list_points2d_inliers;
    std::vector<cv::Point3f> list_points3d_inliers;

    if(good_matches.size() > 0)
    {
      tstart = (double)clock()/CLOCKS_PER_SEC;

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

      tstop = (double)clock()/CLOCKS_PER_SEC;
      ttime = tstop-tstart; /*ttime is how long your code run */
      std::cout << "Time 2D/3D corr.: " << ttime*1000 << "ms" << std::endl;
      mean_stuff.push_back(ttime);

      tstart = (double)clock()/CLOCKS_PER_SEC;

      // -- Step 6: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match,
                                       cv::ITERATIVE, inliers_idx,
                                       iterationsCount, (double)reprojectionError/10, minInliersCount );

      tstop = (double)clock()/CLOCKS_PER_SEC;
      ttime = tstop-tstart; /*ttime is how long your code run */
      std::cout << "Time RANSAC: " << ttime*1000 << "ms" << std::endl;
      mean_ransac.push_back(ttime);

      tstart = (double)clock()/CLOCKS_PER_SEC;

      // -- Step 7: Catch the inliers keypoints
      for(int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
      {
        int n = inliers_idx.at<int>(inliers_index);
        cv::Point2f point2d = list_points2d_scene_match[n];
        cv::Point3f point3d = list_points3d_model_match[n];
        list_points2d_inliers.push_back(point2d);
        list_points3d_inliers.push_back(point3d);

        /*unsigned int match_index = 0;
        bool is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
        while ( !is_equal && match_index < good_matches.size() )
        {
          match_index++;
          is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
        }

        matches_inliers.push_back(good_matches[match_index]);
        keypoints_inliers.push_back(keypoints_scene[good_matches[match_index].queryIdx]);*/
      }

      // -- Step 8: Calculate covariance
      double detection_variance = get_variance(list_points3d_inliers);
      confidence = (detection_variance/model_variance)*100;

      tstop = (double)clock()/CLOCKS_PER_SEC;
      ttime = tstop-tstart; /*ttime is how long your code run */
      std::cout << "Time 2D/3D corr.: " << ttime*1000 << "ms" << std::endl;
      mean_stuff.push_back(ttime);


      tstart = (double)clock()/CLOCKS_PER_SEC;

      /*// -- Step 10: Kalman Filter

      // GOOD MEASUREMENT
      if( !isnan(confidence) && inliers_idx.rows >= min_inliers && confidence > min_confidence)
      {
        // Get measures
        cv::Mat measured_translation(3, 1, CV_64F);
        measured_translation = pnp_detection.get_t_matrix();

        cv::Mat measured_rotation(3, 3, CV_64F);
        measured_rotation = pnp_detection.get_R_matrix();

        cv::Mat measured_quaternion(4, 1, CV_64F);
        measured_quaternion = rot2quat(measured_rotation);

        // Convert rotation matrix to euler angles
        cv::Mat measured_eulers(3, 1, CV_64F);
        measured_eulers = rot2euler(measured_rotation);
        //std::cout << "measured_eulers : " << measured_eulers << std::endl << std::endl;

        // Set measurement to predict
        measurement.at<double>(0) = measured_translation.at<double>(0); // x
        measurement.at<double>(1) = measured_translation.at<double>(1); // y
        measurement.at<double>(2) = measured_translation.at<double>(2); // z
        measurement.at<double>(3) =  measured_quaternion.at<double>(0); // qw
        measurement.at<double>(4) =  measured_quaternion.at<double>(1); // qx
        measurement.at<double>(5) =  measured_quaternion.at<double>(2); // qy
        measurement.at<double>(6) =  measured_quaternion.at<double>(3); // qz

        //drawObjectMesh(frame_vis, &mesh, &pnp_detection, green);

      }

      // First predict, to update the internal statePre variable
      cv::Mat prediction = KF.predict();
      //std::cout << "Prediction : " << prediction << std::endl << std::endl;

      // The "correct" phase that is going to use the predicted value and our measurement
      cv::Mat estimated = KF.correct(measurement);
      //std::cout << "Estimation : " << estimated << std::endl << std::endl;

      // Estimated translation
      cv::Mat translation_est(3, 1, CV_64F);
      translation_est.at<double>(0) = estimated.at<double>(0);
      translation_est.at<double>(1) = estimated.at<double>(1);
      translation_est.at<double>(2) = estimated.at<double>(2);
      //std::cout << "Trans : " << translation_est << std::endl << std::endl;

      // Estimated quaternion
      cv::Mat quaternion_est(4, 1, CV_64F);
      quaternion_est.at<double>(0) = estimated.at<double>(9);
      quaternion_est.at<double>(1) = estimated.at<double>(10);
      quaternion_est.at<double>(2) = estimated.at<double>(11);
      quaternion_est.at<double>(3) = estimated.at<double>(12);

      // Convert estimated quaternion to rotation matrix
      cv::Mat rotation_est(3, 3, CV_64F);
      rotation_est = quat2rot(quaternion_est);

      // Set estimated projection matrix
      pnp_detection_est.set_P_matrix(rotation_est, translation_est);

      drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow);*/


      // Alpha-Beta filter

      // Measurements
      cv::Mat measured_translation(3, 1, CV_64F);
      measured_translation = pnp_detection.get_t_matrix();

      cv::Mat measured_rotation(3, 3, CV_64F);
      measured_rotation = pnp_detection.get_R_matrix();

      // Convert rotation matrix to quaternion
      cv::Mat measured_eulers(3, 1, CV_64F);
      measured_eulers = rot2euler(measured_rotation);

      // Set measurement to predict
      xm.at<double>(0) = measured_translation.at<double>(0); // x
      xm.at<double>(1) = measured_translation.at<double>(1); // y
      xm.at<double>(2) = measured_translation.at<double>(2); // z
      qm.at<double>(0) = measured_eulers.at<double>(0); // roll
      qm.at<double>(1) = measured_eulers.at<double>(1); // pitch
      qm.at<double>(2) = measured_eulers.at<double>(2); // yaw

      // POSITION
      // prediction
      xk = xk_1 + vk_1.mul(dt);
      vk = vk_1;
      // residual error
      r_xk = xm - xk;
      // correction
      xk = xk + r_xk.mul(a); // xk += a * rk
      vk = vk + cv::Mat(r_xk.mul(b)).mul(1/dt); // vk += (b * rk) / dt
      // update
      xk_1 = xk;
      vk_1 = vk;

      // ORIENTATION
      // prediction
      qk = qk_1 + wk_1.mul(dt);
      wk = wk_1;
      /*wk.at<double>(0) = (qk.at<double>(0)-qk_1.at<double>(0))/dt;
      wk.at<double>(1) = (qk.at<double>(1)-qk_1.at<double>(1))/dt;
      wk.at<double>(2) = (qk.at<double>(2)-qk_1.at<double>(2))/dt;*/
      // residual error
      r_qk = qm - qk;
      // correction
      qk = qk + r_qk.mul(a);
      wk = wk + cv::Mat(r_qk.mul(b)).mul(1/dt);
      // update
      qk_1 = qk;
      wk_1 = wk;
      //std::cout << "qk : " << qk << std::endl << std::endl;

      // Estimated translation
      cv::Mat translation_est(3, 1, CV_64F);
      translation_est.at<double>(0) = xk.at<double>(0);
      translation_est.at<double>(1) = xk.at<double>(1);
      translation_est.at<double>(2) = xk.at<double>(2);
      //std::cout << "Trans : " << translation_est << std::endl << std::endl;

      // Estimated quaternion
      cv::Mat eulers_est(3, 1, CV_64F);
      eulers_est.at<double>(0) = qk.at<double>(0);
      eulers_est.at<double>(1) = qk.at<double>(1);
      eulers_est.at<double>(2) = qk.at<double>(2);

      // Convert estimated quaternion to rotation matrix
      cv::Mat rotation_est(3, 3, CV_64F);
      rotation_est = euler2rot(eulers_est);

      // Set estimated projection matrix
      pnp_detection_est.set_P_matrix(rotation_est, translation_est);

      drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow);
      drawObjectMesh(frame_vis, &mesh, &pnp_detection, green);


      tstop = (double)clock()/CLOCKS_PER_SEC;
      ttime = tstop-tstart; /*ttime is how long your code run */
      std::cout << "Time Kalman: " << ttime*1000 << "ms" << std::endl;

    }

    tstart = (double)clock()/CLOCKS_PER_SEC;

    // -- Step X: Draw pose

    double l = 5;
    std::vector<cv::Point2f> pose_points2d;
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(l,0,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,l,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,l)));
    draw3DCoordinateAxes(frame, pose_points2d);

    tstop = (double)clock()/CLOCKS_PER_SEC;
    ttime = tstop-tstart; /*ttime is how long your code run */
    std::cout << "Time draw pose: " << ttime*1000 << "ms" << std::endl;
    mean_stuff.push_back(ttime);


    // -- Step X: Draw correspondences

    // Switch due to a opencv bug
    draw2DPoints(frame_vis, list_points2d_inliers, blue);
   /* try {

      cv::drawMatches( frame, keypoints_scene, // scene image
                       img_in, keypoints_model,  // model image
                       matches_inliers, frame_vis, red, blue);

    } catch (cv::Exception e) {

      cv::drawMatches( img_in, keypoints_model,  // model image
                       frame, keypoints_scene, // scene image
                       matches_inliers, frame_vis, red, blue);

    }*/

    // FRAME RATE

    // see how much time has elapsed
    time(&end);

    // calculate current FPS
    ++counter;
    sec = difftime (end, start);

    fps = counter / sec;

    drawFPS(frame_vis, fps, yellow); // frame ratio
    drawConfidence(frame_vis, confidence, yellow);


    // -- Step X: Draw some debugging text

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
    oVideoWriter.write(frame);

    tstop2 = (double)clock()/CLOCKS_PER_SEC;
    ttime2 = tstop2-tstart2; /*ttime is how long your code run */
    std::cout << "Time frame: " << ttime2*1000 << "ms" << std::endl;
    mean_frame.push_back(ttime2);

    std::cout << " ******************* " << std::endl;

    //cv::waitKey(0);

  }

  // Close and Destroy Window
  cv::destroyWindow("REAL TIME DEMO");

  // Timings means
  std::cout << " MATCHING: " << mean(mean_match) << std::endl;
  std::cout << " RANSAC: " << mean(mean_ransac) << std::endl;
  std::cout << " STUFF: " << mean(mean_stuff) << std::endl;
  std::cout << " FRAME: " << mean(mean_frame) << std::endl;

  std::cout << " ******************* " << std::endl;


  std::cout << "GOODBYE ..." << std::endl;

}
