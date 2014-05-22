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

#include "opencv2/core/core.hpp"


  /*
   * Set up the images paths
   */
  std::string img_path = "../Data/resized_IMG_3875.JPG";
  std::string ply_read_path = "../Data/box.ply";

  // Boolean the know if the registration it's done
  bool end_registration = false;

  /*
   * Set up the intrinsic camera parameters
   */
  double f = 55;
  double sx = 22.3, sy = 14.9;
  double width = 718, height = 480;
  double params[] = { width*f/sx,   // fx
                      height*f/sy,  // fy
                      width/2,      // cx
                      height/2};    // cy

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
  ModelRegistration modelReg;
  ObjectMesh objMesh;
  ObjectModel objModel;
  PnPProblem pnpProb(params);


/*
 *  MODEL REGISTRATION: mouse events callback
 *  Register a point after left button click
 *  Ask next point to register after right click
 *
 */


  // Setup the points to register in the image
  int n = 7;
  int pts[] = {1, 2, 3, 5, 6, 7, 8};


static void onMouseModelRegistration( int event, int x, int y, int, void* )
{
  if  ( event == cv::EVENT_LBUTTONUP )
  {
      int n_regist = modelReg.getNumRegist();
      int n_vertex = pts[n_regist];

      cv::Point2f point_2d = cv::Point2f(x,y);
      cv::Point3f point_3d = objMesh.getVertex(n_vertex-1);

      bool is_registrable = modelReg.is_registrable();
      if (is_registrable)
      {
        modelReg.registerPoint(point_2d, point_3d);
        if( modelReg.getNumRegist() == modelReg.getNumMax() ) end_registration = true;
      }
  }
}


/*
 *   MAIN PROGRAM
 *
 */

int main(int, char**)
{

  std::cout << "!!!Hello Computer Vision!!!" << std::endl; // prints !!!Hello World!!!


  /*
   *  CREATE & OPEN MESH OBJECT
   *
   *  1) Create mesh object
   *  2) Set identification number
   *  3) Load the mesh file (*.ply)
   *
   */

  // Set the mesh id and load the *.ply file
  objMesh.loadMesh(ply_read_path);

  // Create & Open Main Window
  cv::namedWindow("MODEL REGISTRATION", cv::WINDOW_KEEPRATIO);


  // Set up the mouse events
  cv::setMouseCallback("MODEL REGISTRATION", onMouseModelRegistration, 0 );


  /*
   *  CREATE & OPEN INPUT IMAGE
   *
   *  1) Read the input image from a given path
   *  2) Create an image for debug some info
   *  3) Check if the image is not empty
   *
   */

  cv::Mat img_in = cv::imread(img_path, cv::IMREAD_COLOR);
  cv::Mat img_vis = img_in.clone();

  if (!img_in.data) {
    std::cout << "Could not open or find the image" << std::endl;
    return -1;
  }


   /*
    *  MODEL REGISTRATION
    *
    *  1) Set max number of registration points
    *  2) Show the image until the points are not added
    *  3) Show the image until the ESC key is pressed
    *  4) Draw some debug info into the image
    *
    */

  // Get the total number of vertices of the mesh
  int num_registrations = n;
  modelReg.setNumMax(num_registrations);

  std::cout << "Click the box corners ..." << std::endl;
  std::cout << "Waiting ..." << std::endl;


  while ( cv::waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    // Current registered points
    std::vector<cv::Point2f> list_2d_points = modelReg.get_2d_points();
    std::vector<cv::Point3f> list_3d_points = modelReg.get_3d_points();

    // Draw points
    drawPoints(img_vis, list_2d_points, list_3d_points, red);

    if (!end_registration)
    {
      // Draw debug text
      int n_regist = modelReg.getNumRegist();
      int n_vertex = pts[n_regist];
      cv::Point3f current_point_3d = objMesh.getVertex(n_vertex-1);

      drawQuestion(img_vis, current_point_3d, green);
      drawCounter(img_vis, modelReg.getNumRegist(), modelReg.getNumMax(), red);
    }
    else
    {
      // Draw debug text
      drawText(img_vis, "END REGISTRATION", green);
      drawCounter(img_vis, modelReg.getNumRegist(), modelReg.getNumMax(), green);
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

  std::vector<cv::Point2f> list_2d_points = modelReg.get_2d_points();
  std::vector<cv::Point3f> list_3d_points = modelReg.get_3d_points();

  bool is_correspondence = pnpProb.estimatePose(list_2d_points, list_3d_points, flags);
  if ( is_correspondence )
  {
    std::cout << "Correspondence found" << std::endl;

    // Compute all the 2D points of the mesh to verify the algorithm
    std::vector<cv::Point2f> pts_2d_ground_truth = pnpProb.verify_points(&objMesh);
    draw2DPoints(img_vis, pts_2d_ground_truth, green);

    // TODO: quality verification
    //p.writeUVfile(csv_write_path);

  } else {
    std::cout << "Correspondence not found" << std::endl;
  }

  // Show the images
  cv::imshow("MODEL REGISTRATION", img_vis);

  // Show image until ESC pressed
  cv::waitKey(0);

  // Close and Destroy Main Window
  cv::destroyWindow("MODEL REGISTRATION");


   /*
    *
    * COMPUTE 3D of the image Keypoints
    *
    */

  // Create & Open Main Window
  cv::namedWindow("CHECK POINTS", cv::WINDOW_KEEPRATIO);

  // Containers for keypoints and descriptors
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  // Compute keypoints and descriptors
  computeKeyPoints(img_in.clone(), keypoints, descriptors);

  // Check if keypoints are on the surface
  for (unsigned int i = 0; i < keypoints.size(); ++i) {
    cv::Point2f point_2d(keypoints[i].pt);
    cv::Point3f point_3d;
    bool on_surface = pnpProb.backproject2DPoint(&objMesh, point_2d, point_3d);
    if (on_surface)
    {
        objModel.add_correspondence(point_2d, point_3d);
        objModel.add_descriptor(descriptors.row(i));
    }
    else
    {
        objModel.add_outlier(point_2d);
    }
  }


  // Show keypoints
  while ( cv::waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    std::vector<cv::Point2f> points_in = objModel.get_2d_points_in();
    std::vector<cv::Point2f> points_out = objModel.get_2d_points_out();

    // Draw some debug text
    std::string n = boost::lexical_cast< std::string >(points_in.size());
    std::string text = "There are " + n + " inliers";
    drawText(img_in, text, green);

    // Draw some debug text
    n = boost::lexical_cast< std::string >(points_out.size());
    text = "There are " + n + " outliers";
    drawText2(img_in, text, red);

    // Draw the object mesh
    drawObjectMesh(img_in, &objMesh, &pnpProb, blue);

    // Draw found keypoints
    draw2DPoints(img_in, points_in, green);
    draw2DPoints(img_in, points_out, red);

    // Show the images
    cv::imshow("CHECK POINTS", img_in);

  }

  // Close and Destroy Main Window
  cv::destroyWindow("CHECK POINTS");


  /*
  *
  * READ VIDEO STREAM AND COMPUTE KEYPOINTS
  *
  */

  // Create & Open Main Window
  cv::namedWindow("LIVE DEMO", cv::WINDOW_KEEPRATIO);

  cv::VideoCapture cap(1); // open the default camera
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

  cv::ORB orb(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

  while( cv::waitKey(30) < 0)
  {
      cv::Mat frame;
      cap >> frame; // get a new frame from camera

      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;

      //-- Step 1: Calculate keypoints
      orb.detect( frame, keypoints );

      //-- Step 2: Calculate descriptors (feature std::vectors)
      orb.compute( frame, keypoints, descriptors );

      cv::DrawMatchesFlags flag;
      cv::drawKeypoints(frame, keypoints, frame, blue, flag.DEFAULT);

      cv::imshow("LIVE DEMO", frame);
  }

  // the camera will be deinitialized autocv::Matically in VideoCapture destructor
  cv::destroyWindow("LIVE DEMO");


  std::cout << "GOODBYE ..." << std::endl;

  return 0;
}
