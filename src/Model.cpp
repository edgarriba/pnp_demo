/*
 * Model.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "Model.h"

Model::Model() : list_points2d_in_(0), list_points2d_out_(0), list_points3d_in_(0)
{
  n_correspondences_ = 0;
}

Model::~Model()
{
  // TODO Auto-generated destructor stub
}

void Model::add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d)
{
  list_points2d_in_.push_back(point2d);
  list_points3d_in_.push_back(point3d);
  n_correspondences_++;
}

void Model::add_outlier(const cv::Point2f &point2d)
{
  list_points2d_out_.push_back(point2d);
}

void Model::add_descriptor(const cv::Mat &descriptor)
{
  descriptors_.push_back(descriptor);
}

cv::Point3f Model::get_correspondence3d(const cv::Point2f &point2d)
{
  unsigned int i = 0;
  bool is_equal = (point2d.x == list_points3d_in_[i].x) && (point2d.y == list_points3d_in_[i].y);
  while( !is_equal && i < list_points3d_in_.size() )
  {
    i++;
    is_equal = (point2d.x == list_points3d_in_[i].x) && (point2d.y == list_points3d_in_[i].y);
  }
    return list_points3d_in_[i];
}


