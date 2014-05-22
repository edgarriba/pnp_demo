/*
 * ObjectModel.h
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#ifndef OBJECTMODEL_H_
#define OBJECTMODEL_H_

#include <iostream>
#include <opencv2/core.hpp>

class ObjectModel
{
public:
  ObjectModel();
  virtual
  ~ObjectModel();

  std::vector<cv::Point2f> get_2d_points_in() const { return list_points2d_in_; }
  std::vector<cv::Point2f> get_2d_points_out() const { return list_points2d_out_; }
  int get_numDescriptors() const { return descriptors_.rows; }

  void add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d)
  {
    list_points2d_in_.push_back(point2d);
    list_points3d_in_.push_back(point3d);
    num_correspondences_++;
  }

  void add_outlier(const cv::Point2f &point2d)
  {
    list_points2d_out_.push_back(point2d);
  }

  void add_descriptor(const cv::Mat &descriptor)
  {
    descriptors_.push_back(descriptor);
  }

private:
  int id_;
  int num_correspondences_;
  std::vector<cv::Point2f> list_points2d_in_;
  std::vector<cv::Point2f> list_points2d_out_;
  std::vector<cv::Point3f> list_points3d_in_;
  cv::Mat descriptors_;
};

#endif /* OBJECTMODEL_H_ */
