/*
 * ModelRegistration.h
 *
 *  Created on: Apr 18, 2014
 *      Author: edgar
 */

#ifndef MODELREGISTRATION_H_
#define MODELREGISTRATION_H_

#include <iostream>
#include <opencv2/core.hpp>

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
  void registerPoint(const cv::Point2f &point2d, const cv::Point3f &point3d)
  {
    // add correspondence at the end of the vector
     list_points2d_.push_back(point2d);
     list_points3d_.push_back(point3d);
     n_registrations_++;
  }

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
  int getNumRegist() const { return n_registrations_; }

  std::vector<cv::Point2f> get_2d_points() const { return list_points2d_; }
  std::vector<cv::Point3f> get_3d_points() const { return list_points3d_; }

  bool is_registrable() const { return (n_registrations_ < max_registrations_); }


private:
/** The current number of registered points */
int n_registrations_;
/** The total number of points to register */
int max_registrations_;
/** The list of 2D points to register the model */
std::vector<cv::Point2f> list_points2d_;
/** The list of 3D points to register the model */
std::vector<cv::Point3f> list_points3d_;
};

#endif /* MODELREGISTRATION_H_ */
