/*
 * Model.h
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#ifndef MODEL_H_
#define MODEL_H_

#include <iostream>
#include <opencv2/core.hpp>

class Model
{
public:
  Model();
  virtual ~Model();

  std::vector<cv::Point2f> get_points2d_in() const { return list_points2d_in_; }
  std::vector<cv::Point2f> get_points2d_out() const { return list_points2d_out_; }
  cv::Mat get_descriptors() const { return descriptors_; }
  int get_numDescriptors() const { return descriptors_.rows; }

  void add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d);
  void add_outlier(const cv::Point2f &point2d);
  void add_descriptor(const cv::Mat &descriptor);

private:
  /** The current number of correspondecnes */
  int n_correspondences_;
  /** The list of 2D points on the model surface */
  std::vector<cv::Point2f> list_points2d_in_;
  /** The list of 2D points outside the model surface */
  std::vector<cv::Point2f> list_points2d_out_;
  /** The list of 3D points on the model surface */
  std::vector<cv::Point3f> list_points3d_in_;
  /** The list of 2D points descriptors */
  cv::Mat descriptors_;
};

#endif /* OBJECTMODEL_H_ */
