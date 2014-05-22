/*
 * ObjectModel.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: edgar
 */

#include "ObjectModel.h"

ObjectModel::ObjectModel() : list_points2d_in_(0), list_points2d_out_(0), list_points3d_in_(0)
{
  id_ = 0;
  num_correspondences_ = 0;
}

ObjectModel::~ObjectModel()
{
  // TODO Auto-generated destructor stub
}

