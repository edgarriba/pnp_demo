#include <iostream>
#include <time.h>
#include <boost/lexical_cast.hpp>

#include "cv.h"
#include "highgui.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "Mesh.h"
#include "Model.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "ModelRegistration.h"
#include "Utils.h"

//std::string video_path = "../Data/box1.mp4";     // video
std::string video_path = "../Data/box2.mp4";    // video
//std::string video_path = "../Data/box3_hd.MP4";    // video HD

//  COOKIES BOX - ORB
std::string yml1_read_path = "../Data/cookies_ORB_vga_1.yml"; // 3dpts + descriptors
std::string yml2_read_path = "../Data/cookies_ORB_vga_2.yml";
std::string yml3_read_path = "../Data/cookies_ORB_vga_3.yml";

//  COOKIES BOX HD - ORB
//std::string yml1_read_path = "../Data/cookies_ORB_hd_1.yml"; // 3dpts + descriptors
//std::string yml2_read_path = "../Data/cookies_ORB_hd_2.yml";

// COOKIES BOX MESH
std::string ply_read_path = "../Data/box.ply";   // mesh


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


// Robust Matcher parameters
int numKeyPoints = 2000; // 2500
float ratio = 0.70f; // 80

// RANSAC parameters
int iterationsCount = 500;
int reprojectionError = 2.0;
int minInliersCount = 70;

// after RANSAC
int min_inliers = 20; // 20


/**********************************************************************************************************/

void initKalmanFilter( cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);


/**********************************************************************************************************/


void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurements,
                         cv::Mat &translation_estimated, cv::Mat &rotation_estimated );

/**********************************************************************************************************/


void fillMeasurements( cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured);


/**********************************************************************************************************/


int main(int, char**)
{

  std::cout << "!!!Hello Detection!!!" << std::endl;

  PnPProblem pnp_detection(params_WEBCAM);
  PnPProblem pnp_detection_est(params_WEBCAM);

  Model model;
  //model.load(yml1_read_path); // load a mesh given the *.ply file path
  model.load(yml2_read_path); // load a mesh given the *.ply file path
 // model.load(yml3_read_path); // load a mesh given the *.ply file path

  Mesh mesh;
  mesh.load(ply_read_path); // load the 3D textured object model


  // Instantiate RobustMatcher

  RobustMatcher rmatcher;

  // set ratio test parameter
  rmatcher.setRatio(ratio);

  // Instantiate and set robust Matcher: detector
  cv::FeatureDetector * detector = new cv::OrbFeatureDetector(numKeyPoints);
  rmatcher.setFeatureDetector(detector);

  // Instantiate Flann matcher
  cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
  cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);

  // Set matcher type to RobusMatcher
  cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);
  rmatcher.setDescriptorMatcher(matcher);


  // Instantiate Kalman Filter
  int nStates = 18, nMeasurements = 6, nInputs = 0;
  double dt = 0.10; // 10 FPS

  cv::KalmanFilter KF;
  initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);

  cv::Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(cv::Scalar(0));



  // Get the MODEL INFO
  std::vector<cv::Point2f> list_points2d_model = model.get_points2d_in();
  std::vector<cv::Point3f> list_points3d_model = model.get_points3d();
  std::vector<cv::KeyPoint> keypoints_model = model.get_keypoints();
  cv::Mat descriptors_model = model.get_descriptors();


  // Create & Open Window
  cv::namedWindow("REAL TIME DEMO", CV_WINDOW_KEEPRATIO);

  //cv::VideoCapture cap(0); // open the default camera
  cv::VideoCapture cap(video_path); // open the recorded video
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

  double tstart2, tstop2, ttime2; // algorithm metrics
  double tstart, tstop, ttime; // algorithm metrics

  cv::Mat frame, frame_vis;

  // Loop videostream
  while(cap.read(frame) && cv::waitKey(30) != 27)
  {

    // get a new frame from camera
    frame_vis = frame.clone();

    // -- Step 1: Robust matching between model descriptors and scene descriptors
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::KeyPoint> keypoints_scene;

    rmatcher.robustMatch(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);
    //rmatcher.robustMatchFull(frame, good_matches, keypoints_scene, keypoints_model, descriptors_model);

    cv::Mat inliers_idx;
    std::vector<cv::DMatch> matches_inliers;
    std::vector<cv::Point2f> list_points2d_inliers;
    std::vector<cv::Point3f> list_points3d_inliers;


    if(good_matches.size() > 0) // If no matches, RANSAC crashes
    {

      // -- Step 2: Find out the 2D/3D correspondences
      std::vector<cv::Point3f> list_points3d_model_match;
      std::vector<cv::Point2f> list_points2d_scene_match;
      for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
      {
        cv::Point3f point3d_model = list_points3d_model[ good_matches[match_index].trainIdx ];
        cv::Point2f point2d_scene = keypoints_scene[ good_matches[match_index].queryIdx ].pt;
        list_points3d_model_match.push_back(point3d_model);
        list_points2d_scene_match.push_back(point2d_scene);
      }


      // Draw outliers
      draw2DPoints(frame_vis, list_points2d_scene_match, red);


      // -- Step 3: Estimate the pose using RANSAC approach
      pnp_detection.estimatePoseRANSAC( list_points3d_model_match, list_points2d_scene_match,
                                        cv::ITERATIVE, inliers_idx,
                                        iterationsCount, reprojectionError, minInliersCount );


      // -- Step 4: Catch the inliers keypoints
      for(int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
      {
        int n = inliers_idx.at<int>(inliers_index);
        cv::Point2f point2d = list_points2d_scene_match[n];
        cv::Point3f point3d = list_points3d_model_match[n];
        list_points2d_inliers.push_back(point2d);
        list_points3d_inliers.push_back(point3d);
      }


      // -- Step 5: Kalman Filter

      // GOOD MEASUREMENT
      if( inliers_idx.rows >= min_inliers )
      {
        // Get measured translation
        cv::Mat translation_measured(3, 1, CV_64F);
        translation_measured = pnp_detection.get_t_matrix();

        // Get measured rotation
        cv::Mat rotation_measured(3, 3, CV_64F);
        rotation_measured = pnp_detection.get_R_matrix();

        // fill the measurements vector
        fillMeasurements(measurements, translation_measured, rotation_measured);
      }

      // Instantiate estimated translation and rotation
      cv::Mat translation_estimated(3, 1, CV_64F);
      cv::Mat rotation_estimated(3, 3, CV_64F);

      // update the Kalman filter with good measurements
      updateKalmanFilter( KF, measurements,
                          translation_estimated, rotation_estimated);

      // -- Step 6: Set estimated projection matrix
      pnp_detection_est.set_P_matrix(rotation_estimated, translation_estimated);

    }


    // -- Step X: Draw pose

    drawObjectMesh(frame_vis, &mesh, &pnp_detection_est, yellow);
    drawObjectMesh(frame_vis, &mesh, &pnp_detection, green);

    double l = 5;
    std::vector<cv::Point2f> pose_points2d;
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(l,0,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,l,0)));
    pose_points2d.push_back(pnp_detection_est.backproject3DPoint(cv::Point3f(0,0,l)));
    draw3DCoordinateAxes(frame_vis, pose_points2d);


    // -- Step X: Draw inliers

    draw2DPoints(frame_vis, list_points2d_inliers, blue);


    // FRAME RATE

    // see how much time has elapsed
    time(&end);

    // calculate current FPS
    ++counter;
    sec = difftime (end, start);

    fps = counter / sec;

    drawFPS(frame_vis, fps, yellow); // frame ratio
    double ratio = ((double)inliers_idx.rows/(double)good_matches.size())*100;
    drawConfidence(frame_vis, ratio, yellow);


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

    //cv::waitKey(0);

  }

  // Close and Destroy Window
  cv::destroyWindow("REAL TIME DEMO");

  std::cout << "GOODBYE ..." << std::endl;

}

/**********************************************************************************************************/
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{

  //init Kalman
  KF.init(nStates, nMeasurements, nInputs, CV_64F);
  std::cout << nStates << std::endl;

  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));


  // DYNAMIC MODEL

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


  // MEASUREMENT MODEL

//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw

  std::cout << "A " << std::endl << KF.transitionMatrix << std::endl;
  std::cout << "C " << std::endl << KF.measurementMatrix << std::endl;

}

/**********************************************************************************************************/
void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                         cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{

  // First predict, to update the internal statePre variable
  cv::Mat prediction = KF.predict();

  // The "correct" phase that is going to use the predicted value and our measurement
  cv::Mat estimated = KF.correct(measurement);

  // Estimated translation
  translation_estimated.at<double>(0) = estimated.at<double>(0);
  translation_estimated.at<double>(1) = estimated.at<double>(1);
  translation_estimated.at<double>(2) = estimated.at<double>(2);

  // Estimated euler angles
  cv::Mat eulers_estimated(3, 1, CV_64F);
  eulers_estimated.at<double>(0) = estimated.at<double>(9);
  eulers_estimated.at<double>(1) = estimated.at<double>(10);
  eulers_estimated.at<double>(2) = estimated.at<double>(11);

  // Convert estimated quaternion to rotation matrix
  rotation_estimated = euler2rot(eulers_estimated);

}

/**********************************************************************************************************/
void fillMeasurements( cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
  // Convert rotation matrix to euler angles
  cv::Mat measured_eulers(3, 1, CV_64F);
  measured_eulers = rot2euler(rotation_measured);

  // Set measurement to predict
  measurements.at<double>(0) = translation_measured.at<double>(0); // x
  measurements.at<double>(1) = translation_measured.at<double>(1); // y
  measurements.at<double>(2) = translation_measured.at<double>(2); // z
  measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
  measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
  measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}
