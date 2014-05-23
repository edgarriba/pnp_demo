//============================================================================
// Name        : main.cpp
// Author      : Edgar Riba
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>

#include <assert.h>
#include <boost/lexical_cast.hpp>

#include "PnPProblem.h"
#include "ObjectMesh.h"
#include "ObjectModel.h"
#include "ModelRegistration.h"
#include "Utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>


  /*
   * Set up the images paths
   */
  std::string img_path = "../Data/resized_IMG_3875.JPG";
  //std::string img_path = "../Data/test.jpg";
  std::string ply_read_path = "../Data/box.ply";

  // Boolean the know if the registration it's done
  bool end_registration = false;

  /*
   * Set up the intrinsic camera parameters: CANON
   */
  double f = 55;
  double sx = 22.3, sy = 14.9;
  double width = 718, height = 480;
  double params[] = { width*f/sx,   // fx
                      height*f/sy,  // fy
                      width/2,      // cx
                      height/2};    // cy


  // Setup the points to register in the image
  // In the order of the *.ply file and starting at 1
  int n = 7;
  int pts[] = {1, 2, 3, 5, 6, 7, 8};


  /*
    * Set up the intrinsic camera parameters: LOGITECH QUICK PRO 5000
    */
   /*double f = 57;
   double sx = 22.3, sy = 14.9;
   double width = 640, height = 480;
   double params[] = { width*f/sx,   // fx
                       height*f/sy,  // fy
                       width/2,      // cx
                       height/2};    // cy*/

  /*
   * Set up some basic colors
   */
  cv::Scalar red(0, 0, 255);
  cv::Scalar green(0,255,0);
  cv::Scalar blue(255,0,0);

  /*
   * CREATE MODEL REGISTRATION OBJECT
   * CREATE OBJECT MESH
   * CREATE OBJECT MODEL
   * CREATE PNP OBJECT
   */
  ModelRegistration registration;
  ObjectMesh mesh;
  ObjectModel model;
  PnPProblem pnp(params);


// Mouse events for model registration
static void onMouseModelRegistration( int event, int x, int y, int, void* )
{
  if  ( event == cv::EVENT_LBUTTONUP )
  {
      int n_regist = registration.getNumRegist();
      int n_vertex = pts[n_regist];

      cv::Point2f point_2d = cv::Point2f(x,y);
      cv::Point3f point_3d = mesh.getVertex(n_vertex-1);

      bool is_registrable = registration.is_registrable();
      if (is_registrable)
      {
        registration.registerPoint(point_2d, point_3d);
        if( registration.getNumRegist() == registration.getNumMax() ) end_registration = true;
      }
  }
}


/*
 *   MAIN PROGRAM
 *
 */

int main(int, char**)
{

  std::cout << "!!!Hello Google Summer of Code!!!" << std::endl; // prints !!!Hello World!!!

  // load a mesh given the *.ply file path
  mesh.load(ply_read_path);

  // Create & Open Window
  cv::namedWindow("MODEL REGISTRATION", cv::WINDOW_KEEPRATIO);


  // Set up the mouse events
  cv::setMouseCallback("MODEL REGISTRATION", onMouseModelRegistration, 0 );

  // Open the image to register
  cv::Mat img_in = cv::imread(img_path, cv::IMREAD_COLOR);
  cv::Mat img_vis = img_in.clone();

  if (!img_in.data) {
    std::cout << "Could not open or find the image" << std::endl;
    return -1;
  }

  // Set the number of points to register
  int num_registrations = n;
  registration.setNumMax(num_registrations);

  std::cout << "Click the box corners ..." << std::endl;
  std::cout << "Waiting ..." << std::endl;

  // Loop until all the points are registered
  while ( cv::waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    // Current registered points
    std::vector<cv::Point2f> list_points2d = registration.get_points2d();
    std::vector<cv::Point3f> list_points3d = registration.get_points3d();

    // Draw current registered points
    drawPoints(img_vis, list_points2d, list_points3d, red);

    // If the registration is not finished, draw which 3D point we have to register.
    // If the registration is finished, breaks the loop.
    if (!end_registration)
    {
      // Draw debug text
      int n_regist = registration.getNumRegist();
      int n_vertex = pts[n_regist];
      cv::Point3f current_poin3d = mesh.getVertex(n_vertex-1);

      drawQuestion(img_vis, current_poin3d, green);
      drawCounter(img_vis, registration.getNumRegist(), registration.getNumMax(), red);
    }
    else
    {
      // Draw debug text
      drawText(img_vis, "END REGISTRATION", green);
      drawCounter(img_vis, registration.getNumRegist(), registration.getNumMax(), green);
      break;
    }

    // Show the image
    cv::imshow("MODEL REGISTRATION", img_vis);
  }


  /*
   *
   * COMPUTE CAMERA POSE
   *
   */

   std::cout << "COMPUTING POSE ..." << std::endl;

  //int flags = CV_ITERATIVE;
  //int flags = CV_P3P;
  int flags = CV_EPNP;

  // The list of registered points
  std::vector<cv::Point2f> list_points2d = registration.get_points2d();
  std::vector<cv::Point3f> list_points3d = registration.get_points3d();

  // Estimate pose given the registered points
  bool is_correspondence = pnp.estimatePose(list_points2d, list_points3d, flags);
  if ( is_correspondence )
  {
    std::cout << "Correspondence found" << std::endl;

    // Compute all the 2D points of the mesh to verify the algorithm and draw it
    std::vector<cv::Point2f> pts_2d_ground_truth = pnp.verify_points(&mesh);
    draw2DPoints(img_vis, pts_2d_ground_truth, green);

    // TODO: quality verification
    //p.writeUVfile(csv_write_path);

  } else {
    std::cout << "Correspondence not found" << std::endl;
  }

  // Show the image
  cv::imshow("MODEL REGISTRATION", img_vis);

  // Show image until ESC pressed
  cv::waitKey(0);

  // Close and Destroy Window
  cv::destroyWindow("MODEL REGISTRATION");


   /*
    *
    * COMPUTE 3D of the image Keypoints
    *
    */

  // Create & Open Window
  cv::namedWindow("CHECK POINTS", cv::WINDOW_KEEPRATIO);

  // Containers for keypoints and descriptors of the model
  std::vector<cv::KeyPoint> keypoints_model;
  cv::Mat descriptors_model;

  // Compute keypoints and descriptors
  computeKeyPoints(img_in, keypoints_model, descriptors_model);

  // Check if keypoints are on the surface of the registration image and add to the model
  for (unsigned int i = 0; i < keypoints_model.size(); ++i) {
    cv::Point2f point2d(keypoints_model[i].pt);
    cv::Point3f point3d;
    bool on_surface = pnp.backproject2DPoint(&mesh, point2d, point3d);
    if (on_surface)
    {
        model.add_correspondence(point2d, point3d);
        model.add_descriptor(descriptors_model.row(i));
    }
    else
    {
        model.add_outlier(point2d);
    }
  }


  // Draw keypoints
  while ( cv::waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    // The list of the points2d of the model
    std::vector<cv::Point2f> list_points_in = model.get_points2d_in();
    std::vector<cv::Point2f> list_points_out = model.get_points2d_out();

    // Draw some debug text
    std::string n = boost::lexical_cast< std::string >(list_points_in.size());
    std::string text = "There are " + n + " inliers";
    drawText(img_vis, text, green);

    // Draw some debug text
    n = boost::lexical_cast< std::string >(list_points_out.size());
    text = "There are " + n + " outliers";
    drawText2(img_vis, text, red);

    // Draw the object mesh
    drawObjectMesh(img_vis, &mesh, &pnp, blue);

    // Draw found keypoints depending on if are or not on the surface
    draw2DPoints(img_vis, list_points_in, green);
    draw2DPoints(img_vis, list_points_out, red);

    // Show the image
    cv::imshow("CHECK POINTS", img_vis);

  }

  // Close and Destroy Window
  cv::destroyWindow("CHECK POINTS");


  /*
  * DEMO LIVE:
  * READ VIDEO STREAM AND COMPUTE KEYPOINTS
  *
  */

  // Create & Open Window
  cv::namedWindow("LIVE DEMO", cv::WINDOW_KEEPRATIO);

  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  /* ORB parameters */
  int nfeatures = 2500;
  float scaleFactor = 1.2f;
  int nlevels = 8;
  int edgeThreshold = 31;
  int firstLevel = 0;
  int WTA_K = 2;
  int scoreType = cv::ORB::HARRIS_SCORE;
  int patchSize = 31;

  // Create ORB detector
  cv::ORB orb(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

  /* CREATE MATCHER
   *
   * For ORB descriptors it's used HORM_HAMMING
   *
   *  */

  int normType = cv::NORM_HAMMING;
  bool crossCheck = false;
  cv::BFMatcher matcher(normType, crossCheck);

  // Add the descriptor to the matcher and train
  matcher.add(descriptors_model);
  matcher.train();

  // Loop videostream
  while( cv::waitKey(30) < 0)
  {
      cv::Mat frame, frame_vis;
      cap >> frame; // get a new frame from camera

      std::vector<cv::KeyPoint> keypoints_scene, keypoints_match;
      cv::Mat descriptors_scene;

      //-- Step 1: Calculate keypoints
      orb.detect( frame, keypoints_scene );

      //-- Step 2: Calculate descriptors (feature std::vectors)
      orb.compute( frame, keypoints_scene, descriptors_scene );

      // -- Step 3: Match the found keypoints
      //std::vector<std::vector<cv::DMatch> > matches;
      std::vector<std::vector<cv::DMatch> > matches;
      //matcher.match(model_descriptors, scene_descriptors, matches);
      matcher.knnMatch(descriptors_scene, matches, 10);

      // Catch the matched keypoints to draw it
      for(unsigned int i = 0; i < matches.size(); i++)
      {
        cv::DMatch match = matches[i][0];
        cv::KeyPoint kp = keypoints_model[match.queryIdx];
        keypoints_match.push_back(kp);
      }

      cv::DrawMatchesFlags flag;
      cv::drawKeypoints(frame, keypoints_match, frame, blue, flag.DEFAULT);
      cv::drawKeypoints(frame, keypoints_scene, frame, red, flag.DEFAULT);
      //cv::drawMatches(img_in, keypoints_model, frame, keypoints_vis, matches, frame);


      // Draw some debug text
      std::string n = boost::lexical_cast< std::string >(matches.size());
      std::string text = "Found " + n + " matches";
      drawText(frame, text, green);

      cv::imshow("LIVE DEMO", frame);
  }

  // Close and Destroy Window
  cv::destroyWindow("LIVE DEMO");


  std::cout << "GOODBYE ..." << std::endl;

  return 0;
}
