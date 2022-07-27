// /*
//  * map_coverage_node.cpp
//  *
//  *  Created on: April 26, 2021
//  *      Author: yakir huri
//  *
//  *
//  * Cogniteam LTD CONFIDENTIAL
//  *
//  * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
//  *
//  * NOTICE:  All information contained  herein  is,  and  remains the property
//  * of Cogniteam.   The   intellectual   and   technical   concepts  contained
//  * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
//  * Foreign Patents, patents in process,  and  are  protected  by trade secret
//  * or copyright law. Dissemination of  this  information  or  reproduction of
//  * this material is strictly forbidden unless  prior  written  permission  is
//  * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
//  * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
//  * managers   or   contractors   who   have   executed   Confidentiality  and
//  * Non-disclosure    agreements    explicitly    covering     such     access
//  *
//  * The copyright notice  above  does  not  evidence  any  actual  or intended
//  * publication  or  disclosure    of    this  source  code,   which  includes
//  * information that is confidential  and/or  proprietary,  and  is  a   trade
//  * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
//  * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
//  * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
//  * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
//  * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
//  * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
//  * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
//  * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
//  *
//  */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <map>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>

using namespace cv;

using namespace std;

#include "../include/DisantanceMapCoverage.h"
#include "../include/GoalCalculator.h"
#include "../include/AstarPathManager.h"
#include "../include/MoveBaseController.h"

cv::Mat currentAlgoMap_;
cv::Point startCoveragePoint;
cv::Point goalCoveragePoint;
cv::Point aStarStart;
cv::Point globalStart(startCoveragePoint.x, startCoveragePoint.y);

int getDistanceBetweenGoalsPix(float mapResolution = 0.050000, float distM = 0.5)
{
  float pixDist = (1.0 / mapResolution) * distM;

  return (int)pixDist;
}

cv::Point fixLocationOnGrid(const cv::Point& goal)
{
  int detlaY = abs(goal.y + globalStart.y) % getDistanceBetweenGoalsPix();
  int detlaX = abs(goal.x + globalStart.x) % getDistanceBetweenGoalsPix();

  cv::Point p(goal.x - detlaX, goal.y - detlaY);
  int val = currentAlgoMap_.at<uchar>(p.y, p.x);
  if (val == free_space)
  {
    detlaY = abs(p.y + globalStart.y) % getDistanceBetweenGoalsPix();
    detlaX = abs(p.x + globalStart.x) % getDistanceBetweenGoalsPix();

    return p;
  }

  int scalar = 1;

  for (int i = 0; i < 20; i++)
  {
    // Mat binarayImage = currentAlgoMap_.clone();
    // cvtColor(binarayImage, binarayImage, COLOR_GRAY2BGR);

    cv::Point left(p.x - (getDistanceBetweenGoalsPix() * scalar), p.y);
    cv::Point right(p.x + (getDistanceBetweenGoalsPix() * scalar), p.y);
    cv::Point up(p.x, p.y - (getDistanceBetweenGoalsPix() * scalar));
    cv::Point down(p.x, p.y + (getDistanceBetweenGoalsPix() * scalar));
    cv::Point downLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)),
                       p.y + ((getDistanceBetweenGoalsPix() * scalar)));
    cv::Point downRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)),
                        p.y + ((getDistanceBetweenGoalsPix() * scalar)));
    cv::Point topLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)), p.y - ((getDistanceBetweenGoalsPix() * scalar)));
    cv::Point topRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)),
                       p.y - ((getDistanceBetweenGoalsPix() * scalar)));

    // circle(binarayImage,p, 2, Scalar(0, 255, 0), -1, 8, 0);

    // circle(binarayImage,left, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,right, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,up, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,down, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,downLeft, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,downRight, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,topLeft, 2, Scalar(0, 0, 255), -1, 8, 0);
    // circle(binarayImage,topRight, 2, Scalar(0, 0, 255), -1, 8, 0);

    // imshow("binarayImage",binarayImage);
    // waitKey(0);

    if (left.y > 0 && left.x > 0 && left.y < currentAlgoMap_.rows && left.x < currentAlgoMap_.cols)
    {
      int valLeft = currentAlgoMap_.at<uchar>(left.y, left.x);

      if (valLeft == free_space)
      {
        return left;
      }
    }
    if (right.y > 0 && right.x > 0 && right.y < currentAlgoMap_.rows && right.x < currentAlgoMap_.cols)
    {
      int valRight = currentAlgoMap_.at<uchar>(right.y, right.x);

      if (valRight == free_space)
      {
        return right;
      }
    }
    if (up.y > 0 && up.x > 0 && up.y < currentAlgoMap_.rows && up.x < currentAlgoMap_.cols)
    {
      int valUp = currentAlgoMap_.at<uchar>(up.y, up.x);
      if (valUp == free_space)
      {
        return up;
      }
    }

    if (down.y > 0 && down.x > 0 && down.y < currentAlgoMap_.rows && down.x < currentAlgoMap_.cols)
    {
      int valDown = currentAlgoMap_.at<uchar>(down.y, down.x);
      if (valDown == free_space)
      {
        return down;
      }
    }
    if (downLeft.y > 0 && downLeft.x > 0 && downLeft.y < currentAlgoMap_.rows && downLeft.x < currentAlgoMap_.cols)
    {
      int valDownLeft = currentAlgoMap_.at<uchar>(downLeft.y, downLeft.x);
      if (valDownLeft == free_space)
      {
        return downLeft;
      }
    }
    if (downRight.y > 0 && downRight.x > 0 && downRight.y < currentAlgoMap_.rows && downRight.x < currentAlgoMap_.cols)
    {
      int valDownRight = currentAlgoMap_.at<uchar>(downRight.y, downRight.x);
      if (valDownRight == free_space)
      {
        return downRight;
      }
    }
    if (topLeft.y > 0 && topLeft.x > 0 && topLeft.y < currentAlgoMap_.rows && topLeft.x < currentAlgoMap_.cols)
    {
      int vaLtopLeft = currentAlgoMap_.at<uchar>(topLeft.y, topLeft.x);
      if (vaLtopLeft == free_space)
      {
        return topLeft;
      }
    }
    if (topRight.y > 0 && topRight.x > 0 && topRight.y < currentAlgoMap_.rows && topRight.x < currentAlgoMap_.cols)
    {
      int valTopRight = currentAlgoMap_.at<uchar>(topRight.y, topRight.x);
      if (valTopRight == free_space)
      {
        return topRight;
      }
    }

    scalar *= 2;
  }

  cerr << " failed fffffffix location " << endl;

  p.x = -1;
  p.y = -1;
  return p;
}

DistanceTransformGoalCalculator distanceTransformGoalCalculator;
DisantanceMapCoverage disantanceMapCoverage;
GoalCalculator goalCalculator;
AstarPathManager astarPathManager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  //////////////////////////////
  currentAlgoMap_ = imread("/home/yakir/distance_transform_coverage_ws/flow/mymap1.pgm", 0);
  startCoveragePoint =cv::Point(117, 303);
  
 
  ////////////////////////////////////////////////////////////////////

  // calculate the first goal
  auto res = goalCalculator.getInitialGoal(currentAlgoMap_, startCoveragePoint, goalCoveragePoint,
                                           getDistanceBetweenGoalsPix(), globalStart);

  if (!res)
  {
    cerr << " first map bad !!  " << endl;
    return -1;
  }

//   startCoveragePoint = fixLocationOnGrid(goalCoveragePoint);

   //calculate goal-distance-transform-map
    cv::Mat distanceTransformImg;


    // calc the distance-transform-img from current goal
    distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_, 
            goalCoveragePoint, distanceTransformImg, 1);


    // if gloalPath_ isnt empty, marks the visited points on map
    // distanceTransformGoalCalculator.setScoreVisitedPoint(
    //         distanceTransformImg, gloalPath_, LARGE_NUM);      


    // calc the path-coverage of the current blob

    vector<cv::Point> path = 
            disantanceMapCoverage.getCoveragePath(currentAlgoMap_, startCoveragePoint,
            goalCoveragePoint ,distanceTransformImg, getDistanceBetweenGoalsPix(), true );


  imshow("currentAlgoMap_",currentAlgoMap_);
  waitKey(0);

  return 0;
}
