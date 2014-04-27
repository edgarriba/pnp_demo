/*
 * ModelRegistration.h
 *
 *  Created on: Apr 18, 2014
 *      Author: edgar
 */

#ifndef MODELREGISTRATION_H_
#define MODELREGISTRATION_H_

#include <iostream>

#include <cv.h>
#include <highgui.h>

#include "PnPProblem.h"

using namespace cv;
using namespace std;

class ModelRegistration
{
public:
  /**
  * The default constructor of the ModelRegistration Class
  *
  * @param
  * @return
  *
  */
  ModelRegistration();

  /** The default destructor of the Class */
  virtual ~ModelRegistration();

  /**
   * Add a point from the image to list_uv_points
   *
   * @param point - The 2D point to add to the list
   * @return
   *
   */
  void registerPoint(const pair<Point2f,Point3f> correspondence);

  /**
   * Set the number of points to register the model
   *
   * @param n - The total number of points to register
   * @return
   *
   */
  void setNumMax(int n) { max_registrations_ = n; }

  /**
  * Get the number of points to register
  *
  * @param
  * @return - The total number of points to register the model
  *
  */
  int getNumMax() const { return max_registrations_; }

  /**
  * Get the current number of registered points
  *
  * @param
  * @return - The current number of registered points
  *
  */
  int getNum() const { return n_registrations_; }

  bool is_registrable(const int vertex_iterator);
  pair<int,pair<Point2f,Point3f> > getCorrespondence(int n) const { return list_correspondences_.at(n); };
  vector<pair<int,pair<Point2f,Point3f> > > getAllCorrespondences() const { return list_correspondences_; };

private:
/** The current number of registered points */
int n_registrations_;
/** The total number of points to register */
int max_registrations_;
/** The list of 2D points to register the model */
vector<pair<int,pair<Point2f,Point3f> > > list_correspondences_;
};

#endif /* MODELREGISTRATION_H_ */
