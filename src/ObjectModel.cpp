/*
 * ObjectModel.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "ObjectModel.h"

ObjectModel::ObjectModel() : list_points2d_in_(0), list_points2d_out_(0), list_points3d_in_(0)
{
  n_correspondences_ = 0;
}

ObjectModel::~ObjectModel()
{
  // TODO Auto-generated destructor stub
}

void ObjectModel::add_correspondence(const cv::Point2f &point2d, const cv::Point3f &point3d)
{
  list_points2d_in_.push_back(point2d);
  list_points3d_in_.push_back(point3d);
  n_correspondences_++;
}

void ObjectModel::add_outlier(const cv::Point2f &point2d)
{
  list_points2d_out_.push_back(point2d);
}

void ObjectModel::add_descriptor(const cv::Mat &descriptor)
{
  descriptors_.push_back(descriptor);
}

