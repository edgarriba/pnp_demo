#include <iostream>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Mesh.h"
#include "Model.h"
#include "PnPProblem.h"
#include "ModelRegistration.h"
#include "Utils.h"

std::string yml_read_path = "../Data/box.yml";
std::string img_path = "../Data/resized_IMG_3875.JPG";
std::string ply_read_path = "../Data/box.ply";

/*
 * Set up the intrinsic camera parameters: UVC WEBCAM
 */
double f = 43;
double sx = 22.3, sy = 14.9;
double width = 718, height = 480;
double params_CANON[] = { width*f/sx,   // fx
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


int main(int, char**)
{

  std::cout << "!!!Hello Detection!!!" << std::endl; // prints !!!Hello World!!!

  PnPProblem pnp_detection(params_CANON);

  Model model;
  model.load(yml_read_path);

  Mesh mesh;
  mesh.load(ply_read_path);

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
  cv::Mat descriptors_model = model.get_descriptors();

  int normType = cv::NORM_HAMMING;
  bool crossCheck = false;
  cv::BFMatcher matcher(normType, crossCheck);

  // Add the descriptor to the matcher and train
  matcher.add(descriptors_model);
  matcher.train();

  std::vector<cv::KeyPoint> keypoints_model;
  computeKeyPoints(img_in, keypoints_model, descriptors_model);

  // Create & Open Window
  cv::namedWindow("REAL TIME DEMO", cv::WINDOW_KEEPRATIO);

  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;


  // Loop videostream
  while( cv::waitKey(30) < 0)
  {
    cv::Mat frame, frame_gray, frame_vis;
    cap >> frame; // get a new frame from camera

    cv::cvtColor( frame, frame_gray, CV_RGB2GRAY );

    //-- Step 1 & 2: Calculate keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors_scene;
    computeKeyPoints(frame, keypoints_scene, descriptors_scene);

    // -- Step 3: Matching features
    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch(descriptors_scene, matches, 2); // Find two nearest matches

    // -- Step 4: Ratio test
    std::vector<cv::DMatch> good_matches;
    for (unsigned int match_index = 0; match_index < matches.size(); ++match_index)
    {
      const float ratio = 0.8; // As in Lowe's paper; can be tuned
      if (matches[match_index][0].distance < ratio * matches[match_index][1].distance)
      {
          good_matches.push_back(matches[match_index][0]);
      }
    }

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
    cv::Mat inliers;
    pnp_detection.estimatePoseRANSAC(list_points3d_model_match, list_points2d_scene_match, cv::EPNP, inliers);

    // -- Step 7: Catch the inliers keypoints
    std::vector<cv::DMatch> matches_inliers;
    std::vector<cv::Point2f> list_points2d_inliers;
    for(int inliers_index = 0; inliers_index < inliers.rows; ++inliers_index)
    {
      int n = inliers.at<int>(inliers_index);
      cv::Point2f point2d = list_points2d_scene_match[n];
      list_points2d_inliers.push_back(point2d);

      unsigned int match_index = 0;
      bool is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
      while ( !is_equal && match_index < good_matches.size() )
      {
        match_index++;
        is_equal = equal_point( point2d, keypoints_scene[good_matches[match_index].queryIdx].pt );
      }
      matches_inliers.push_back(good_matches[match_index]);
    }

    // -- Step 8: Back project
    cv::Mat P_mat = pnp_detection.get_P_matrix();
    //std::cout << "P_matrix:" << std::endl << P_mat << std::endl;

    // Draw pose
    double l = 5;
    std::vector<cv::Point2f> pose_points2d;
    pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,0,0)));
    pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(l,0,0)));
    pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,l,0)));
    pose_points2d.push_back(pnp_detection.backproject3DPoint(cv::Point3f(0,0,l)));
    draw3DCoordinateAxes(frame, pose_points2d);

    // -- Step X: Draw correspondences

    // Switched the order due to a opencv bug
    cv::drawMatches(frame, keypoints_scene, img_in, keypoints_model, matches_inliers, frame_vis, red, blue);

    // -- Step X: Draw some text for debugging purpose

    // Draw some debug text
    int inliers_int = inliers.rows;
    int outliers_int = list_points3d_model_match.size() - inliers_int;
    std::string inliers_str = boost::lexical_cast< std::string >(inliers_int);
    std::string outliers_str = boost::lexical_cast< std::string >(outliers_int);
    std::string n = boost::lexical_cast< std::string >(good_matches.size());
    std::string text = "Found " + inliers_str + " of " + n + " matches";
    std::string text2 = "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

    drawText(frame_vis, text, green);
    drawText2(frame_vis, text2, red);

    drawObjectMesh(frame_vis, &mesh, &pnp_detection, green);

    cv::imshow("REAL TIME DEMO", frame_vis);
  }

  // Close and Destroy Window
  cv::destroyWindow("REAL TIME DEMO");

  std::cout << "GOODBYE ..." << std::endl;

}
