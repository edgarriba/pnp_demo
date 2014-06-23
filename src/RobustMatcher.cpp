/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "RobustMatcher.h"
#include <time.h>

#include <opencv2/features2d/features2d.hpp>

RobustMatcher::~RobustMatcher() {
  // TODO Auto-generated destructor stub
}

int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches)
{
  int removed=0;
  // for all matches
  for ( std::vector<std::vector<cv::DMatch> >::iterator
       matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
      // check distance ratio
      if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio_)
      {
        matchIterator->clear(); // remove match
        removed++;
      }
    }
    else
    { // does not have 2 neighbours
      matchIterator->clear(); // remove match
      removed++;
    }
  }
  return removed;
}


void RobustMatcher::symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                     const std::vector<std::vector<cv::DMatch> >& matches2,
                     std::vector<cv::DMatch>& symMatches )
{
  // for all matches image 1 -> image 2
   for (std::vector<std::vector<cv::DMatch> >::const_iterator
       matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
   {
      // ignore deleted matches
      if (matchIterator1->size() < 2)
          continue;
      // for all matches image 2 -> image 1
      for (std::vector<std::vector<cv::DMatch> >::const_iterator
          matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
      {
          // ignore deleted matches
          if (matchIterator2->size() < 2)
             continue;
          // Match symmetry test
          if ((*matchIterator1)[0].queryIdx ==
              (*matchIterator2)[0].trainIdx &&
              (*matchIterator2)[0].queryIdx ==
              (*matchIterator1)[0].trainIdx) {
              // add symmetrical match
                symMatches.push_back(
                  cv::DMatch((*matchIterator1)[0].queryIdx,
                             (*matchIterator1)[0].trainIdx,
                             (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
          }
      }
   }
}

void RobustMatcher::robustMatchFull( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
              std::vector<cv::KeyPoint>& keypoints_frame, const std::vector<cv::KeyPoint>& keypoints_model, const cv::Mat& descriptors_model )
{

  // 1a. Detection of the ORB features
  this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches12, matches21;

  // 2a. From image 1 to image 2
  matcher_->clear();
  matcher_->add(descriptors_model);
  matcher_->train();
  matcher_->knnMatch(descriptors_frame, matches12, 2); // return 2 nearest neighbours

  // 2b. From image 2 to image 1
  matcher_->clear();
  matcher_->add(descriptors_frame);
  matcher_->train();
  matcher_->knnMatch(descriptors_model, matches21, 2); // return 2 nearest neighbours

  // 3. Remove matches for which NN ratio is > than threshold
  // clean image 1 -> image 2 matches
  int removed1 = ratioTest(matches12);
  // clean image 2 -> image 1 matches
  int removed2 = ratioTest(matches21);

  // 4. Remove non-symmetrical matches
  std::vector<cv::DMatch> symMatches;
  symmetryTest(matches12, matches21, symMatches);

  good_matches.clear();
  good_matches = symMatches;

}

void RobustMatcher::robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                 std::vector<cv::KeyPoint>& keypoints_frame,
                                 const std::vector<cv::KeyPoint>& keypoints_model,
                                 const cv::Mat& descriptors_model )
{
  good_matches.clear();

  // 1a. Detection of the ORB features
  this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches;
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2);

  // 3. Remove matches for which NN ratio is > than threshold
  int removed = ratioTest(matches);

  for ( std::vector<std::vector<cv::DMatch> >::iterator
         matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
      // check distance ratio
      if ((*matchIterator)[0].distance < ratio_ * (*matchIterator)[1].distance)
      {
        good_matches.push_back((*matchIterator)[0]);
      }
    }
  }

}
