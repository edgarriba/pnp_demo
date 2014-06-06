/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "RobustMatcher.h"


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

void RobustMatcher::crossCheckMatching( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                         std::vector<cv::KeyPoint>& keypoints_frame,
                         const std::vector<cv::KeyPoint>& keypoints_model,
                         const cv::Mat& descriptors_model )
{
    good_matches.clear();

    // 1a. Detection of the SURF features
    this->computeKeyPoints(frame, keypoints_frame);

    // 1b. Extraction of the SURF descriptors
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

    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            cv::DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    good_matches.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}

void RobustMatcher::robustMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
              std::vector<cv::KeyPoint>& keypoints_frame, const std::vector<cv::KeyPoint>& keypoints_model, const cv::Mat& descriptors_model )
{

  // 1a. Detection of the SURF features
  this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the SURF descriptors
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

  //matcher_->match(descriptors_frame, descriptors_model, good_matches);
}

void RobustMatcher::simpleMatch( const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
                                 std::vector<cv::KeyPoint>& keypoints_frame,
                                 const std::vector<cv::KeyPoint>& keypoints_model,
                                 const cv::Mat& descriptors_model )
{
  good_matches.clear();

  // 1a. Detection of the SURF features
  this->computeKeyPoints(frame, keypoints_frame);

  // 1b. Extraction of the SURF descriptors
  cv::Mat descriptors_frame;
  this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches;

  // 2a. From image 1 to image 2
  matcher_->clear();
  matcher_->knnMatch(descriptors_model, descriptors_frame, matches, 2); // return 2 nearest neighbours

  // -- Step 4: Ratio test
    for (unsigned int match_index = 0; match_index < matches.size(); ++match_index)
    {
     const float ratio = 0.8; // As in Lowe's paper; can be tuned
     if (matches[match_index][0].distance < ratio * matches[match_index][1].distance)
     {
         good_matches.push_back(matches[match_index][0]);
     }
    }
}
