/*
 * ModelRegistration.cpp
 *
 *  Created on: Apr 18, 2014
 *      Author: edgar
 */

#include <iostream>

#include <cv.h>
#include <highgui.h>
#include "ModelRegistration.h"

using namespace cv;
using namespace std;

/** The default constructor of the Class */
ModelRegistration::ModelRegistration() : list_correspondences_(0)
{
  n_registrations_ = 0;
  max_registrations_ = 0;
}

/** The default destructor of the Class */
ModelRegistration::~ModelRegistration()
{
  // TODO Auto-generated destructor stub
}

/** Add a point from the image to list_uv_points */
void ModelRegistration::registerPoint(const pair<Point2f,Point3f> correspondence)
{
  // add correspondence at the end of the vector
  n_registrations_++;
  list_correspondences_.push_back(make_pair(n_registrations_, correspondence));
}

bool ModelRegistration::is_registrable(const int vertex_iterator)
{
  if (vertex_iterator < max_registrations_-1)
  {
    return true;
  }
  else
  {
    return false;
  }
}




