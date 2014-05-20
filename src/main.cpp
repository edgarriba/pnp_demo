//============================================================================
// Name        : epnp_demo.cpp
// Author      : Edgar Riba
// Version     : 0.1
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <sstream>

#include <assert.h>
#include <boost/lexical_cast.hpp>

#include "PnPProblem.h"
#include "ObjectMesh.h"
#include "ObjectModel.h"
#include "ModelRegistration.h"
#include "Utils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace std;
using namespace cv;
FlannBasedMatcher matcher_;


  /*
   * Set up the images paths
   */
  string img_path = "../Data/resized_IMG_3875.JPG";
  string ply_read_path = "../Data/box.ply";


  /*
   * Set up windows strings
   */
  string str_winame_model_registration = "MODEL REGISTRATION";
  string str_winame_check_points = "CHECK POINTS";
  string str_winame_video_stream = "LIVE DEMO";


  bool end_registration = false;

  /*
   * Set up the intrinsic camera parameters
   */
  double f = 55;
  double sx = 22.3, sy = 14.9;
  double width = 718, height = 480;
  double params[] = {width*f/sx,height*f/sy,width/2,height/2}; // fx, fy, cx, cy

  /*
   * Set up some basic colors
   */
  Scalar red(0, 0, 255);
  Scalar green(0,255,0);
  Scalar blue(255,0,0);

  /*
   * CREATE MODEL REGISTRATION OBJECT
   * CREATE OBJECT MESH
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
static void onMouseModelRegistration( int event, int x, int y, int, void* )
{
  if  ( event == EVENT_LBUTTONUP )
  {
      int vertex_it = objMesh.getVertexIter();
      bool is_registrable = modelReg.is_registrable(vertex_it);

      Point2f point_2d = Point2f(x,y);
      Point3f point_3d = objMesh.getVertex(vertex_it).getPoint();

      if (is_registrable)
      {
        modelReg.registerPoint(make_pair(point_2d, point_3d));
      }
      else if (!end_registration)
      {
        modelReg.registerPoint(make_pair(point_2d, point_3d));
        end_registration = true;
      }
      objMesh.incrVertexIterator();
  }
  else if  ( event == EVENT_RBUTTONUP )
  {
      objMesh.incrVertexIterator();
  }
}


/*
 *   MAIN PROGRAM
 *
 */

int main(int, char**)
{

  cout << "!!!Hello Computer Vision!!!" << endl; // prints !!!Hello World!!!


  /*
   *  CREATE & OPEN MESH OBJECT
   *
   *  1) Create mesh object
   *  2) Set identification number
   *  3) Load the mesh file (*.ply)
   *
   */

  objMesh.setMeshID(0);
  objMesh.loadMesh(ply_read_path);

  // Create & Open Main Window
  namedWindow(str_winame_model_registration,CV_WINDOW_KEEPRATIO);

  // Set up the mouse events
  setMouseCallback(str_winame_model_registration, onMouseModelRegistration, 0 );


  /*
   *  CREATE & OPEN INPUT IMAGE
   *
   *  1) Read the input image from a given path
   *  2) Create an image for debug some info
   *  3) Check if the image is not empty
   *
   */

  Mat img_in = imread(img_path, CV_LOAD_IMAGE_COLOR);
  Mat img_vis = img_in.clone();

  if (!img_in.data) {
    cout << "Could not open or find the image" << endl;
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

  modelReg.setNumMax(objMesh.getNumVertices());

  cout << "Click the box corners ..." << endl;
  cout << "Waiting ..." << endl;


  while ( waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    // Draw coordinates axes
    //draw3DCoordinateAxes(img_vis, red);

    // Draw points
    drawPoints(img_vis, &modelReg, red);

    if (!end_registration)
    {
      // Draw debug text
      Point3f current_point_3d = objMesh.getVertex(objMesh.getVertexIter()).getPoint();
      drawQuestion(img_vis, current_point_3d, green);
      drawCounter(img_vis, modelReg.getNum(), modelReg.getNumMax(), red);
    }
    else
     {
       // Draw debug text
       drawText(img_vis, "END REGISTRATION", green);
       drawCounter(img_vis, modelReg.getNum(), modelReg.getNumMax(), green);
       break;
     }

    // Show the image
    imshow(str_winame_model_registration, img_vis);
  }


  /*
   *
   * COMPUTE CAMERA POSE
   *
   */

   cout << "COMPUTING POSE ..." << endl;

  //int flags = CV_ITERATIVE;
  //int flags = CV_P3P;
  int flags = CV_EPNP;

  vector<pair<int,pair<Point2f,Point3f> > > list_correspondences = modelReg.getAllCorrespondences();
  if ( pnpProb.estimatePose(list_correspondences, flags) )
  {
    cout << "Correspondence found" << endl;

    // Compute 2D points to verify the algorithm
    vector<Point2f> pts_2D_verified = pnpProb.verify_points(&objMesh);
    draw2DPoints(img_vis, pts_2D_verified, green);

    // TODO: quality verification
    //p.writeUVfile(csv_write_path);

  } else {
    cout << "Correspondence not found" << endl;
  }

  // Show the images
  imshow(str_winame_model_registration, img_vis);

  // Show image until ESC pressed
  waitKey(0);

  // Close and Destroy Main Window
  destroyWindow(str_winame_model_registration);


   /*
    *
    * COMPUTE 3D of the image Keypoints
    *
    */

  // Create & Open Main Window
  namedWindow(str_winame_check_points,CV_WINDOW_KEEPRATIO);

  // Containers for keypoints and descriptors
  std::vector<KeyPoint> keypoints;
  Mat descriptors;

  // Compute keypoints and descriptors
  computeKeyPoints(img_in.clone(), keypoints, descriptors);

  // Check if keypoints are on the surface or not
  for (unsigned int i = 0; i < keypoints.size(); ++i) {
    Point2f point_2d(keypoints[i].pt);
    Point3f point_3d;
    bool on_surface = pnpProb.backproject2DPoint(&objMesh, point_2d, point_3d);
    if (on_surface)
    {
        objModel.add_correspondence(point_2d, point_3d);
        objModel.add_descriptor(descriptors.row(i));
    }
    else
    {
        objModel.add_outer_point(point_2d);
    }
  }

  cout << "Size descriptors: " << objModel.get_numDescriptors()  << endl;


  // Show keypoints
  while ( waitKey(30) < 0 )
  {
    // Refresh debug image
    img_vis = img_in.clone();

    vector<Point2f> points_in = objModel.get_2d_points_in();
    vector<Point2f> points_out = objModel.get_2d_points_out();

    // Draw some debug text
    string n = boost::lexical_cast< string >(points_in.size());
    string text = "There are " + n + " inliers";
    drawText(img_in, text, green);

    // Draw some debug text
    n = boost::lexical_cast< string >(points_out.size());
    text = "There are " + n + " outliers";
    drawText2(img_in, text, red);

    // Draw the object mesh
    drawObjectMesh(img_in, &objMesh, &pnpProb, blue);

    // Draw found keypoints
    draw2DPoints(img_in, points_in, green);
    draw2DPoints(img_in, points_out, red);

    // Show the images
    imshow(str_winame_check_points, img_in);

  }

  // Close and Destroy Main Window
  destroyWindow(str_winame_check_points);


  /*
  *
  * READ VIDEO STREAM AND COMPUTE KEYPOINTS
  *
  */

  // Create & Open Main Window
  namedWindow(str_winame_video_stream,CV_WINDOW_KEEPRATIO);

  VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  /* ORB parameters */
  int nfeatures = 2500;
  float scaleFactor = 1.2f;
  int nlevels = 8;
  int edgeThreshold = 31;
  int firstLevel = 0;
  int WTA_K = 2;
  int scoreType = ORB::HARRIS_SCORE;
  int patchSize = 31;

  ORB orb(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);

  while( waitKey(30) < 0)
  {
      Mat frame;
      cap >> frame; // get a new frame from camera

      std::vector<KeyPoint> keypoints;
      Mat descriptors;

      //-- Step 1: Calculate keypoints
      orb.detect( frame, keypoints );

      //-- Step 2: Calculate descriptors (feature vectors)
      orb.compute( frame, keypoints, descriptors );

      DrawMatchesFlags flag;
      drawKeypoints(frame, keypoints, frame, blue, flag.DRAW_RICH_KEYPOINTS);

      imshow(str_winame_video_stream, frame);
  }

  // the camera will be deinitialized automatically in VideoCapture destructor
  destroyWindow(str_winame_video_stream);


  cout << "GOODBYE ..." << endl;

  return 0;
}
