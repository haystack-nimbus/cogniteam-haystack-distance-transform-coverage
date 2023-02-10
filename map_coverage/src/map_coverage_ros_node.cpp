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
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

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
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <map>
#include <iostream>
#include <random>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <numeric>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <ctime>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace cv;
using namespace std::chrono;
using namespace std;

#include "../include/DisantanceMapCoverage.h"
#include "../include/GoalCalculator.h"
#include "../include/MoveBaseController.h"
#include "../include/logManger.h"

geometry_msgs::PoseStamped startingLocation_;
string startingTime_;

bool exit_ = false;
bool exitPerson_ = false;

enum GoalState
{
  UN_COVERED = 0,
  REJECTED = 1,
  COVERED_BY_ROBOT_PATH = 2,
  COVERED_BY_OBSTACLE = 3,
};

struct Path_with_Status
{
  vector<geometry_msgs::PoseStamped> coveragePathPoses_;

  vector<GoalState> status_;

  vector<cv::Point> path_;  // for display on image

  void initStatusList()
  {
    status_.resize(coveragePathPoses_.size());

    for (int i = 0; i < status_.size(); i++)
    {
      setStatByIndex(i, UN_COVERED);
    }
  }

  void setStatByIndex(int index, GoalState status)
  {
    status_[index] = status;
  }

  void setPixelsPath(const vector<cv::Point>& path)
  {
    path_.resize(path.size());

    for (int i = 0; i < path.size(); i++)
    {
      path_.push_back(cv::Point2d(path[i].x, path[i].y));
    }
  }
};

 
string getCurrentTime()
{
  time_t rawtime;
  struct tm* timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%F/%H_%M", timeinfo);
  string curr_time_Str = strdup(buffer);
  std::replace(curr_time_Str.begin(), curr_time_Str.end(), '/', '_');
  std::replace(curr_time_Str.begin(), curr_time_Str.end(), '-', '_');

  return curr_time_Str;
}

void addDilationForGlobalMap(Mat& imgMap, float walls_inflation_m, float mapResolution)
{
  try
  {
    int dilationPix = 1;  // (1.0 / mapResolution) * (walls_inflation_m);

    cv::Mat binary = imgMap.clone();
    binary.setTo(0, imgMap != 0);
    binary.setTo(255, imgMap == 0);
    dilate(binary, binary, Mat(), Point(-1, -1), dilationPix, 1, 1);

    imgMap.setTo(0, binary == 255);
  }
  catch (cv::Exception& e)
  {
    const char* err_msg = e.what();
    std::cerr << "exception caught: " << err_msg << std::endl;
  }
}

void addFreeSpaceDilation(Mat& grayscaleImg)
{
  Mat binary = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, CV_8UC1, cv::Scalar(0));

  binary.setTo(255, grayscaleImg >= 254);
  dilate(binary, binary, Mat(), Point(-1, -1), 1, 1, 1);

  Mat newImg = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, CV_8UC1, cv::Scalar(205));

  newImg.setTo(254, binary >= 254);
  newImg.setTo(0, grayscaleImg == 0);

  grayscaleImg = newImg;
}

class MapCoverageManager
{
public:
  MapCoverageManager(bool debug = false)
  {
    if (!debug)
    {
      cerr << " wait for move-base server " << endl;
      moveBaseController_.waitForServer(ros::Duration(100.0));
      ros::Duration(1).sleep();
      cerr << " map-coverage is now connecting with move-base !! " << endl;
      disableReverse();
    }

    // rosparam
    ros::NodeHandle nodePrivate("~");
    nodePrivate.param("distance_between_goals_m", distBetweenGoalsM_, 0.5);
    nodePrivate.param("walls_inflation_m", walls_inflation_m_, 0.3);
    nodePrivate.param("robot_w_m", robot_w_m_, 0.53);
    nodePrivate.param("robot_h_m", robot_h_m_, 0.53);
    nodePrivate.param("sanitization_radius", sanitization_radius_, 1.0);
    nodePrivate.param("path_deg_angle_threshold", path_deg_angle_threshold_, 100.0);

    nodePrivate.param<string>("COVERED_BY_OBSTACLE_COLOR", COVERED_BY_OBSTACLE_COLOR, string("FF0000"));
    nodePrivate.param<string>("COVERED_BY_ROBOT_PATH_COLOR", COVERED_BY_ROBOT_PATH_COLOR, string("0000FF"));
    nodePrivate.param<string>("UN_COVERED_COLOR", UN_COVERED_COLOR, string("008000"));
    nodePrivate.param<string>("REJECTED_COLOR", REJECTED_COLOR, string("FFA500"));

    nodePrivate.param("duration_wait_for_move_base_response", duration_wait_for_move_base_response_, 15.0);
    nodePrivate.param<string>("coverage_image_path", coverage_img_path_, string(""));
    nodePrivate.param<string>("base_frame", baseFrame_, string("base_link"));
    nodePrivate.param<string>("global_frame", globalFrame_, string("map"));

    nodePrivate.param("reducing_goals", reducing_goals_, true);

    nodePrivate.param("show_live_video", show_live_video_, false);

    nodePrivate.param("/coverage/percentage", percentCoverage_, 0.0);
    nodePrivate.param("/coverage/state", state_, string("IDLE"));
    nodePrivate.param("/coverage/image_name", image_name_, string(""));

    logManager_.setLogPath(coverage_img_path_);

    // subs
    global_map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &MapCoverageManager::globalMapCallback, this);

    // subs
    global_cost_map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1,
                                                                    &MapCoverageManager::globalCostMapCallback, this);

    camera_scan_sub_ =
        node_.subscribe<sensor_msgs::LaserScan>("/ttt", 1, &MapCoverageManager::cameraScanCallback, this);

    odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom", 1, &MapCoverageManager::odomCallback, this);

    is_person_detected_sub_ =
        node_.subscribe<std_msgs::Bool>("/is_person_detected", 1, &MapCoverageManager::personsCallback, this);

    // timer

    lampTimer_ = node_.createTimer(ros::Rate(10), &MapCoverageManager::updateTimerCallback, this);
    // pubs

    image_transport::ImageTransport it(node_);

    liveMapPub_ = it.advertise("/live_map", 1);
    node_.param("htjpeg_quality", 20);

    reverse_cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

    cuurentCoveragePathPub_ = node_.advertise<nav_msgs::Path>("/coverage_path", 1, false);

    robot_history_path_pub_ = node_.advertise<nav_msgs::Path>("/robot_history_path", 1, false);

    waypoints_with_status_pub_ =
        node_.advertise<visualization_msgs::MarkerArray>("/waypoints_with_status_marker_arr", 10);

    edges_frontires_marker_array_Pub =
        node_.advertise<visualization_msgs::MarkerArray>("/edges_frontiers_marker_arr", 10);

    directions_marker_array_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/directions_marker_arr", 10);

    sanitization_radius_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/sanitization_radius", 10);

    backward_goal_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/backward_goal", 10);

    uv_lamp_set_state_pub_ = node_.advertise<std_msgs::Bool>("/uv_lamp_set_state", 10);

    initSlamMap_ = false;

    // global cost map
    startLocalCostMap_ = high_resolution_clock::now();

    /// params
    mapResolution_ = -1;
    map_origin_position_x = -1;
    map_origin_position_y = -1;

    startingCoverageTime_ = high_resolution_clock::now();

    float robotWidthPix = (1.0 / mapResolution_) * robot_w_m_;
    float robotHeightPix = (1.0 / mapResolution_) * robot_h_m_;

    DisantanceMapCoverage disantanceMapCoverage(true);
    disantanceMapCoverage.setRectFreeSapceDim(robotWidthPix, robotHeightPix);
  }

  ~MapCoverageManager()
  {
    // moveBaseController_.cancelNavigation();
    ros::Duration(1).sleep();

    turnOffLamp();

    lampTimer_.stop();

    logManager_.closeFile();

    cerr << "MapCoverageManager distructor " << endl;
    saveCoverageImg();

    ros::shutdown();
  }

  void turnOffLamp()
  {
    std_msgs::Bool msg;
    msg.data = false;
    uv_lamp_set_state_pub_.publish(msg);
  }

  void setState(string state)
  {
    node_.setParam("/coverage/state", state);
    state_ = state;

    if (state_ == "USER_CTRL_C")
    {
      logManager_.writeToLog("USER_CTRL_C");
      logManager_.closeFile();
    }

    if (state_ == "PERSON_DETECTED")
    {
      logManager_.writeToLog("PERSON_DETECTED");
    }
  }

  bool explore()
  {
    vector<cv::Point2d> unreachedPointFromFronitiers;

    while (ros::ok())
    {
      ros::spinOnce();

      if (exit_)
      {
        setState("USER_CTRL_C");
        saveCoverageImg();
        turnOffLamp();

        return false;
      }

      if (exitPerson_)
      {
        turnOffLamp();
        setState("PERSON_DETECTED");
        saveCoverageImg();

        return false;
      }

      if (!initSlamMap_ || !initGlobalCostMap_)
      {
        // << "map not recieved !!" << endl;

        continue;
      }

      if (!updateRobotLocation())
      {
        cerr << "can't update robot location !!" << endl;
        continue;
      }

      node_.setParam("/coverage/state", "RUNNING");
      state_ = "RUNNING";

      currentAlgoMap_ = getCurrentMap();

      // create the exploration map (gmapping + global cost map)
      Mat explorationImgMap = currentAlgoMap_.clone();
      addDilationByGlobalCostMap(costMapImg_, explorationImgMap, convertPoseToPix(robotPose_));
      addFreeSpaceDilation(explorationImgMap);

      switch (explore_state_)
      {
        case IDLE: {
          cerr << "IDLE " << endl;

          //startingTime_ = getCurrentTime();

          // set the global starting location
          startingLocation_ = robotPose_;
          robotHistoryPathMsg_.poses.push_back(startingLocation_);

          explore_state_ = NAV_TO_NEXT_FRONTIER;

          break;
        }
        case NAV_TO_NEXT_FRONTIER: {
          cerr << "NAV_TO_NEXT_FRONTIER " << endl;

          updateRobotLocation();

          auto robotPix = convertPoseToPix(robotPose_);

          std::vector<Frontier> currentEdgesFrontiers;

          //  try to find next exploration goal
          cerr << " try to find next exploration goal " << endl;
          logManager_.writeToLog("try to find next exploration goal");
          goalCalculator.calcEdgesFrontiers(explorationImgMap, currentEdgesFrontiers, robotPix, mapResolution_,
                                            unreachedPointFromFronitiers);

          if (currentEdgesFrontiers.size() == 0)
          {
            explore_state_ = FINISH_EXPLORE;
            break;
          }

          geometry_msgs::Quaternion q;
          auto goalHeading = -1 * atan2(currentEdgesFrontiers[0].center.y - robotPix.y,
                                        currentEdgesFrontiers[0].center.x - robotPix.x);
          tf2::Quaternion orientation;
          orientation.setRPY(0, 0, -1 * goalHeading);
          q.w = orientation.getW();
          q.x = orientation.getX();
          q.y = orientation.getY();
          q.z = orientation.getZ();

          // pick the closest (from the robot) frontier goal
          auto nextFrontierGoal = convertPixToPose(currentEdgesFrontiers[0].center, q);

          publishEdgesFrontiers(currentEdgesFrontiers);

          /// calculate the  path
          nav_msgs::Path wanted_path;
          bool resMakePlan = makePlan(wanted_path, robotPose_, nextFrontierGoal);
          cerr << " resMakePlan " << resMakePlan << endl;

          if (resMakePlan)
          {
            logManager_.writeToLog("trobot was able to make plan to the frontier goal");

            bool reverseDone = reverseLogic(wanted_path, nextFrontierGoal);

            logManager_.writeToLog("the reverse (backward to the safe goal) was down");
            cerr << " reverseDone " << reverseDone << endl;

            // If the robot traveled in reverse mode,
            // it means that it did not travel to a frontier goal,
            // but to a safe point,
            // so in the next iteration it is necessary to recalculate
            if (reverseDone)
            {
              explore_state_ = NAV_TO_NEXT_FRONTIER;
              break;
            }
          }
          else
          {
            cerr << " failed to calaculate path to this goal " << endl;
            logManager_.writeToLog("failed to calaculate path to this frontier goal");

            unreachedPointFromFronitiers.push_back(currentEdgesFrontiers[0].center);
            explore_state_ = NAV_TO_NEXT_FRONTIER;
            break;
          }

          // the robot was able to calculate path to this goal witout reverse logic
          cerr << " sending goal to the next froniter pix " << currentEdgesFrontiers[0].center << endl;
          logManager_.writeToLog("ending goal to the next froniter pix");

          bool result = sendGoal(nextFrontierGoal, -1, true);

          if (!result)
          {
            cerr << " failed to reach exploration goal " << endl;
            logManager_.writeToLog("failed to reach exploration goal, push to black list");

            unreachedPointFromFronitiers.push_back(currentEdgesFrontiers[0].center);
          }

          // if the robot in farward mode, was able to make plan, but goal aborted,
          // Maybe the robot is stuck and needs a little reverse
          if (resMakePlan && !result)
          {
            Mat safetyMap;
            // check if robot cant rotate in place, if so make small reverse
            if (getSafetyMap(safetyMap) && safetyMap.data)
            {
              bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

              if (!canRotateInPlace)
              {
                cerr << " CAN'T ROTATE TO THE TARGET !! " << endl;

                logManager_.writeToLog("cant rotate to the goal, make small reverese");

                smallReverseAllowed_ = true;

                makeSmallReverse();

                smallReverseAllowed_ = false;
              }
            }
          }

          cerr << "move_base result for NAV_TO_NEXT_FRONTIER " << result << endl;

          explore_state_ = NAV_TO_NEXT_FRONTIER;

          break;
        }
        case ERROR_EXPLORE: {
          lampTimer_.stop();

          setState("STOPPED");
          saveCoverageImg();

          logManager_.writeToLog("ERROR_EXPLORE");

          cerr << "ERROR_EXPLORE " << endl;
          break;
        }
        case FINISH_EXPLORE: {
          cerr << "FINISH_EXPLORE " << endl;

          return true;
        }
      }
    }

    return false;
  }

  void makeSmallReverse()
  {
    // make sure that it sage to do small revers
    {
      if (!initSlamMap_ || !initGlobalCostMap_)
      {
        cerr << "map not recieved !!" << endl;
        return;
      }

      if (!updateRobotLocation())
      {
        cerr << "can't update robot location !!" << endl;

        return;
      }

      cv::Mat safetyMap;
      if (!getSafetyMap(safetyMap) || !safetyMap.data)
      {
        cerr << "failed to get getSafetyMap " << endl;
        return;  /// TO-DO -> THIS IS ERROR, NEED TO UNDERSTAND
      }

      float distBackwaredM = 0.5;
      // get robot heading
      auto robotPix = convertPoseToPix(robotPose_);

      float robotHeading = atan2((2.0 * (robotPose_.pose.orientation.w * robotPose_.pose.orientation.z +
                                         robotPose_.pose.orientation.x * robotPose_.pose.orientation.y)),
                                 (1.0 - 2.0 * (robotPose_.pose.orientation.y * robotPose_.pose.orientation.y +
                                               robotPose_.pose.orientation.z * robotPose_.pose.orientation.z)));

      cv::Point2d backwardPixLocation(robotPix.x + (distBackwaredM / mapResolution_) * cos(robotHeading),
                                      robotPix.y + (distBackwaredM / mapResolution_) * sin(robotHeading));

      if (backwardPixLocation.x < 0 || backwardPixLocation.y < 0 || backwardPixLocation.x > safetyMap.cols ||
          backwardPixLocation.y > safetyMap.rows)
      {
        cerr << " bad  backwardPixLocation " << endl;
        return;
      }
      // check if the backwardPixLocation is blocked !!
      int valn = safetyMap.at<uchar>(backwardPixLocation.y, backwardPixLocation.x);
      if (valn == 0)
      {
        cerr << " next backward goal is blocked !! the val is " << valn << endl;
        return;
      }

      // check collision to next pix backward
      cv::LineIterator it_map_back(safetyMap, robotPix, backwardPixLocation, 4);  // 4 more dense than 8

      for (int j = 0; j < it_map_back.count; j++, ++it_map_back)
      {
        cv::Point2d pointBeam = it_map_back.pos();
        int val = safetyMap.at<uchar>(pointBeam.y, pointBeam.x);
        if (val == 0)
        {
          cerr << " collision detected  !! the al is " << val << endl;
          return;
        }
      }
    }

    geometry_msgs::Twist velocity;
    velocity.linear.x = -0.1;

    auto startTime = ros::WallTime::now();
    // small rverse
    ros::Rate rate(1);
    while (ros::ok())
    {
      cerr << " SMALL REEEEEEVERSE !!!!!!!!!! " << endl;
      reverse_cmd_vel_pub_.publish(velocity);
      rate.sleep();

      if (!updateRobotLocation())
      {
        cerr << " failed update rbot location " << endl;
        return;
      }

      auto end = ros::WallTime::now();

      auto duration = (end - startTime).toSec();
      if (duration > 2)
      {
        break;
      }

      ros::spinOnce();
    }

    cerr << " FINISHED SMALL REVERSE " << endl;
  }
  bool reverseLogic(const nav_msgs::Path& wanted_path, const geometry_msgs::PoseStamped& wantedGoal)
  {
    ros::spinOnce();
    updateRobotLocation();

    /// check if the path in front or behind the robot

    // CALCUALTE ROBOT HEADING
    float robotHeading = atan2((2.0 * (robotPose_.pose.orientation.w * robotPose_.pose.orientation.z +
                                       robotPose_.pose.orientation.x * robotPose_.pose.orientation.y)),
                               (1.0 - 2.0 * (robotPose_.pose.orientation.y * robotPose_.pose.orientation.y +
                                             robotPose_.pose.orientation.z * robotPose_.pose.orientation.z)));
    float robotHeadingDeg = angles::to_degrees(robotHeading);
    int minNumOfPointsOnPath = 10;
    if (wanted_path.poses.size() < minNumOfPointsOnPath)
    {
      cerr << " path backward to small !" << endl;
      logManager_.writeToLog("path backward to small !");

      return false;  /// TO-DO -> THIS IS ERROR, NEED TO UNDERSTAND
    }

    // CALCULATE THE HEADING OF THE PATH FROM THE ROBOT
    float avgAnglesFromR = 0.0;
    for (int i = 0; i < minNumOfPointsOnPath; i++)
    {
      float angleFromRobot = atan2(wanted_path.poses[i].pose.position.y - robotPose_.pose.position.y,
                                   wanted_path.poses[i].pose.position.x - robotPose_.pose.position.x);

      avgAnglesFromR = avgAnglesFromR + angleFromRobot;
    }
    avgAnglesFromR = angles::to_degrees(avgAnglesFromR / minNumOfPointsOnPath);

    /// CALCUALTE THE DIFF BETWEEN ROBOT HEADING TO PATH HEADING
    cerr << " robotHeading " << robotHeadingDeg << " , avgAnglesFromR " << avgAnglesFromR << endl;
    float diffAngles = fabs(calculateDifferenceBetweenAngles(robotHeadingDeg, avgAnglesFromR));
    // cerr<<" the diff is "<<diffAngles<<endl;

    /// THE path IS BEHIND THE ROBOT !!!!!!!
    if ((diffAngles) > path_deg_angle_threshold_)
    {
      cerr << " the path is behind the robot !! " << endl;
      logManager_.writeToLog("the path is behind the robot !!");

      cv::Mat safetyMap;
      if (!getSafetyMap(safetyMap) || !safetyMap.data)
      {
        cerr << "failed to get getSafetyMap " << endl;
        return false;  /// TO-DO -> THIS IS ERROR, NEED TO UNDERSTAND
      }

      bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

      if (!canRotateInPlace)
      {
        cerr << " cant rotate-in place !! enable reverse !! " << endl;
        logManager_.writeToLog("cant rotate-in place !! enable reverse !!");

        /// find on the last forward path goal that the robot can rotate

        bool foundReverseGoal = false;
        if (robotHistoryPathMsg_.poses.size() == 0)
        {
          return false;
        }
        for (int i = robotHistoryPathMsg_.poses.size() - 1; i > 0; i--)
        {
          bool canRotateInPlace =
              checkIFsafeToRotate(safetyMap, robotHistoryPathMsg_.poses[i], robot_w_m_, robot_h_m_, 0, 0.2);

          if (canRotateInPlace)
          {
            // set orienation to this goal (rotate 180)
            float robotHeading = atan2(
                (2.0 *
                 (robotHistoryPathMsg_.poses[i].pose.orientation.w * robotHistoryPathMsg_.poses[i].pose.orientation.z +
                  robotHistoryPathMsg_.poses[i].pose.orientation.x * robotHistoryPathMsg_.poses[i].pose.orientation.y)),
                (1.0 - 2.0 * (robotHistoryPathMsg_.poses[i].pose.orientation.y *
                                  robotHistoryPathMsg_.poses[i].pose.orientation.y +
                              robotHistoryPathMsg_.poses[i].pose.orientation.z *
                                  robotHistoryPathMsg_.poses[i].pose.orientation.z)));

            double wanted_heading = robotHeading + angles::from_degrees(180);
            geometry_msgs::PoseStamped reverseGoal;
            reverseGoal.header.frame_id = globalFrame_;
            reverseGoal.pose.position = robotHistoryPathMsg_.poses[i].pose.position;
            reverseGoal.pose.orientation = tf::createQuaternionMsgFromYaw(wanted_heading);

            cerr << " found reverse goal !! " << reverseGoal.pose.position.x << ", " << reverseGoal.pose.position.y
                 << endl;
            logManager_.writeToLog("found reverse goal !!");

            foundReverseGoal = true;

            // set reverse mode
            setReverse();

            visualization_msgs::Marker marker;
            marker.header.frame_id = globalFrame_;
            marker.header.stamp = ros::Time::now();
            marker.id = rand();

            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position.x = reverseGoal.pose.position.x;
            marker.pose.position.y = reverseGoal.pose.position.y;
            marker.pose.position.z = 0.5;
            marker.pose.orientation.x = reverseGoal.pose.orientation.x;
            marker.pose.orientation.y = reverseGoal.pose.orientation.y;
            marker.pose.orientation.z = reverseGoal.pose.orientation.z;
            marker.pose.orientation.w = reverseGoal.pose.orientation.w;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.a = 1.0;  // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.lifetime = ros::Duration(10.0);

            backward_goal_marker_pub_.publish(marker);

            cerr << "send the reverse goal !!!!!" << endl;
            logManager_.writeToLog("send the reverse goal !!!!!");

            // seng goal backward direction
            bool result = sendGoal(reverseGoal, -1, false, false, true);

            if (result)
            {
              cerr << "reverse goal reached !!! disable reverse " << endl;
              logManager_.writeToLog("reverse goal reached !!! disable reverse");

              // reached the goal !!!
              disableReverse();

              return true;
            }
            else
            {
              cerr << "reverse goal failed !!! disable reverse " << endl;
              logManager_.writeToLog("EDGE_CASE_1: reverse goal failed !!! disable reverse");

              disableReverse();
              return true;
            }
          }
        }

        // if we didnt found reverse goal the robot will navigate with reverse to
        // the starting location
        if (!foundReverseGoal)
        {
          cerr << " didnt found safe goal to rotate, going back in reverse to starting location " << endl;
          logManager_.writeToLog(
              "EDGE_CASE_2: didnt found safe goal to rotate, going back in reverse to starting location");

          setReverse();

          visualization_msgs::Marker marker;
          marker.header.frame_id = globalFrame_;
          marker.header.stamp = ros::Time::now();
          marker.id = rand();
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.pose.position.x = startingLocation_.pose.position.x;
          marker.pose.position.y = startingLocation_.pose.position.y;
          marker.pose.position.z = 0.5;
          marker.pose.orientation.x = startingLocation_.pose.orientation.x;
          marker.pose.orientation.y = startingLocation_.pose.orientation.y;
          marker.pose.orientation.z = startingLocation_.pose.orientation.z;
          marker.pose.orientation.w = startingLocation_.pose.orientation.w;
          marker.scale.x = 0.3;
          marker.scale.y = 0.3;
          marker.scale.z = 0.3;
          marker.color.a = 1.0;  // Don't forget to set the alpha!
          marker.color.r = 0.0;
          marker.color.g = 0.5;
          marker.color.b = 0.5;
          marker.lifetime = ros::Duration(10.0);

          backward_goal_marker_pub_.publish(marker);

          bool result = sendGoal(startingLocation_, -1, true);

          cerr << " status back to starting location in reverse: " << result << endl;

          logManager_.writeToLog("EDGE_CASE_3: status back to starting location in reverse " + to_string(result));

          disableReverse();

          return true;
        }

        return false;
      }
    }
    else
    {
      cerr << " the path in-front of the robot" << endl;
      logManager_.writeToLog("the path in-front of the robot");

      return false;
    }

    return false;
  }

  float calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
  {
    double difference = secondAngle - firstAngle;
    while (difference < -180)
      difference += 360;
    while (difference > 180)
      difference -= 360;
    return difference;
  }
  cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad)
  {
    cv::Point2f outPoint;
    // CW rotation
    outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
    outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
    return outPoint;
  }

  bool checkIFsafeToRotate(const cv::Mat& sefetyMap, const geometry_msgs::PoseStamped& robotPose, float robot_w,
                           float robot_h, float delta_w_cnter, float delta_h_center)
  {
    // Mat tmp = sefetyMap.clone();
    float distFromStatingPointM = goalCalculator.distanceCalculate(
        cv::Point2d(robotPose.pose.position.x, robotPose.pose.position.y),
        cv::Point2d(startingLocation_.pose.position.x, startingLocation_.pose.position.y));

    if (distFromStatingPointM < 0.15)
    {
      cerr << " the robot is very close to the starting location, so it probably can rotate" << endl;
      return true;
    }

    cv::Point2d robotPix;
    robotPix.x = (robotPose.pose.position.x - map_origin_position_x) / mapResolution_;
    robotPix.y = (robotPose.pose.position.y - map_origin_position_y) / mapResolution_;

    float robotHeading = atan2((2.0 * (robotPose.pose.orientation.w * robotPose.pose.orientation.z +
                                       robotPose.pose.orientation.x * robotPose.pose.orientation.y)),
                               (1.0 - 2.0 * (robotPose.pose.orientation.y * robotPose.pose.orientation.y +
                                             robotPose.pose.orientation.z * robotPose.pose.orientation.z)));

    cv::Point2d robotHeadingPoint(robotPix.x + (50) * cos(robotHeading), robotPix.y + (50) * sin(robotHeading));

    cv::Point2d cneterOfRotationPix(robotPix.x + ((1.0 / mapResolution_) * (delta_h_center)) * cos(robotHeading),
                                    robotPix.y + ((1.0 / mapResolution_) * (delta_h_center)) * sin(robotHeading));

    if (cneterOfRotationPix.x < 0 || cneterOfRotationPix.y < 0 || cneterOfRotationPix.x > sefetyMap.cols ||
        cneterOfRotationPix.y > sefetyMap.rows)
    {
      return false;
    }

    float wPix = (1.0 / mapResolution_) * (robot_w);
    float hPix = (1.0 / mapResolution_) * (robot_h);

    // colloect current robot rectangle footprint)
    cv::RotatedRect r(robotPix, cv::Size2f(wPix, hPix), angles::to_degrees(robotHeading));
    Point2f vertices[4];
    r.points(vertices);

    vector<cv::Point2d> footprintPoints;
    for (int i = 0; i < 4; i++)
    {
      cv::LineIterator it_map_back(sefetyMap, vertices[i], vertices[(i + 1) % 4], 4);  // 4 more dense than 8
      for (int j = 0; j < it_map_back.count; j++, ++it_map_back)
      {
        cv::Point2d pointBeam = it_map_back.pos();

        if (pointBeam.x < 0 || pointBeam.y < 0 || pointBeam.x > sefetyMap.cols || pointBeam.y > sefetyMap.rows)
        {
          return false;
        }

        footprintPoints.push_back(pointBeam);
      }
    }

    for (int deg = 0; deg < 360; deg++)
    {
      // Mat dbg = sefetyMap.clone();
      // cvtColor(dbg, dbg, COLOR_GRAY2BGR);
      // cv::arrowedLine(dbg, robotPix, robotHeadingPoint, Scalar(80, 127, 255), 2, 8, 0, 0.3);

      float wanted_angle = deg;
      for (int i = 0; i < footprintPoints.size(); i++)
      {
        cv::Point2d rot = rotate2d(footprintPoints[i] - cneterOfRotationPix, angles::from_degrees(wanted_angle));
        rot.x = rot.x + cneterOfRotationPix.x;
        rot.y = rot.y + cneterOfRotationPix.y;

        // circle(dbg, rot, 1, Scalar(255, 0, 50), -1, 8, 0);

        if (sefetyMap.at<uchar>(rot.y, rot.x) == 0)
        {
          // circle(dbg, rot , 1, Scalar(0,0,255), -1, 8, 0);

          return false;
        }
      }

      // circle(dbg, robotPix , 1, Scalar(255,0,0), -1, 8, 0);
      // circle(dbg, cneterOfRotationPix , 1, Scalar(0,255,255), -1, 8, 0);

      // cv::resize(dbg, dbg, cv::Size(dbg.cols * 4, dbg.rows * 4));

      // imshow("dbg",dbg);
      // waitKey(0);
    }

    return true;
  }

  bool initialization()
  {
    int secondsIdle = 5;

    float mForward = 0.7; 
    startingTime_ = getCurrentTime();
    node_.setParam("/coverage/state", "IDLE");
    state_ = "IDLE";

    auto startTime = ros::WallTime::now();

    cerr << "idle for  " << secondsIdle << " seconds " << endl;

    // A robot stands still and does not move
    while (ros::ok())
    {
      ros::spinOnce();

      if (exit_)
      {
        setState("USER_CTRL_C");
        saveCoverageImg();

        return false;
      }

      if (exitPerson_)
      {
        saveCoverageImg();

        setState("PERSON_DETECTED");
        return false;
      }

      auto end = ros::WallTime::now();

      auto duration = (end - startTime).toSec();
      if (duration > secondsIdle)
      {
        break;
      }
    }

    // Robot starts the initialization phase
    cerr << " Robot starts the initialization phase " << endl;
    logManager_.writeToLog("Robot starts the initialization phase");

    node_.setParam("/coverage/state", "INITIALIZATION");
    state_ = "INITIALIZATION";

    ros::spinOnce();

    if (exit_)
    {
      setState("USER_CTRL_C");
      logManager_.writeToLog("USER_CTRL_C");

      saveCoverageImg();
      return false;
    }

    if (detectedPerson_)
    {
      saveCoverageImg();
      setState("PERSON_DETECTED");
      logManager_.writeToLog("PERSON_DETECTED");

      return false;
    }

    if (!initSlamMap_ || !initGlobalCostMap_)
    {
      cerr << "map not recieved !!" << endl;
      return false;
    }

    if (!updateRobotLocation())
    {
      cerr << "can't update robot location !!" << endl;
      return false;
    }

    // get map
    Mat initalizationMap = getCurrentMap();
    addDilationByGlobalCostMap(costMapImg_, initalizationMap, convertPoseToPix(robotPose_));
    addFreeSpaceDilation(initalizationMap);

    // get robot heading
    auto robotPix = convertPoseToPix(robotPose_);

    float robotHeading = atan2((2.0 * (robotPose_.pose.orientation.w * robotPose_.pose.orientation.z +
                                       robotPose_.pose.orientation.x * robotPose_.pose.orientation.y)),
                               (1.0 - 2.0 * (robotPose_.pose.orientation.y * robotPose_.pose.orientation.y +
                                             robotPose_.pose.orientation.z * robotPose_.pose.orientation.z)));

    // Calculates the point of forward travel on the map
    cv::Point2d nextPixLocation(robotPix.x + (mForward / mapResolution_) * cos(robotHeading),
                                robotPix.y + (mForward / mapResolution_) * sin(robotHeading));

    if (nextPixLocation.x < 0 || nextPixLocation.y < 0 || nextPixLocation.x > initalizationMap.cols ||
        nextPixLocation.y > initalizationMap.rows)
    {
      return false;
    }

    int numOfRounds = 1;

    cerr << " calculate " << mForward << " m goal " << endl;
    logManager_.writeToLog(" calculate 0.7 m goal ");

    // convert pix to pose
    auto q = tf::createQuaternionMsgFromYaw(robotHeading);
    auto one_meter_goal = convertPixToPose(nextPixLocation, q);

    visualization_msgs::Marker marker;
    marker.header.frame_id = globalFrame_;
    marker.header.stamp = ros::Time::now();
    marker.id = rand();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose.position.x = one_meter_goal.pose.position.x;
    marker.pose.position.y = one_meter_goal.pose.position.y;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = one_meter_goal.pose.orientation.x;
    marker.pose.orientation.y = one_meter_goal.pose.orientation.y;
    marker.pose.orientation.z = one_meter_goal.pose.orientation.z;
    marker.pose.orientation.w = one_meter_goal.pose.orientation.w;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(5.0);

    backward_goal_marker_pub_.publish(marker);

    bool result = sendGoal(one_meter_goal, -1, false, true);

    // If the rovbat failed to reach the goal
    if (!result)
    {
      float dist_from_1_meter_goal =
          goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                           cv::Point2d(one_meter_goal.pose.position.x, one_meter_goal.pose.position.y));

      cerr << " the dist to " << mForward << " meter goal  " << dist_from_1_meter_goal << endl;
      if (!(dist_from_1_meter_goal < 0.25))
      {
        cerr << " failed to reach " << mForward << " meter goal , too far " << endl;
        logManager_.writeToLog(" failed to reach 0.7 m goal ");

        return false;
      }
    }

    ros::spinOnce();

    updateRobotLocation();

    Mat safetyMap;
    // check if can rotate in-place there
    if (getSafetyMap(safetyMap))
    {
      cerr << " check if robot can rotate in place  " << endl;
      bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

      if (canRotateInPlace)
      {
        cerr << " robot is eable to rotate in place " << endl;
        logManager_.writeToLog("robot is eable to rotate in place ");

        // rotate the robot in-place numOfRounds times
        if (!rotateInPlace(numOfRounds))  //
        {
          saveCoverageImg();
          setState("PERSON_DETECTED");
          logManager_.writeToLog("PERSON_DETECTED");
          cerr << " person detectd !!! " << endl;
          return false;
        }

        cerr << " finish rotate-in-place, turn on the lamp" << endl;
        logManager_.writeToLog(" finished rotate-in-place, turn on the lamp ");

        // turns on the lamp
        turnOnLamp();
        initializationGood_ = true;
        return true;
      }
      else
      {
        cerr << " cant rotate in 1 meter goal , try to find other goal " << endl;
        // THE Robot moved  forward but cant rotate-in-place
        // try to find other safe location for 360 deg
        for (int trial = 0; trial < 3; trial++)
        {
          if (exit_)
          {
            setState("USER_CTRL_C");
            saveCoverageImg();
            turnOffLamp();

            return false;
          }

          if (exitPerson_)
          {
            turnOffLamp();
            saveCoverageImg();
            setState("PERSON_DETECTED");
            logManager_.writeToLog("PERSON_DETECTED");

            return false;
          }

          cerr << " trial : " << trial << endl;
          // find safe location on cost-map
          ros::spinOnce();

          updateRobotLocation();

          Mat initalizationMap = getCurrentMap();
          addDilationByGlobalCostMap(costMapImg_, initalizationMap, convertPoseToPix(robotPose_));
          addFreeSpaceDilation(initalizationMap);
          initalizationMap.at<uchar>(cv::Point(convertPoseToPix(robotPose_))) = 0;

          Mat dist;
          Mat binary = initalizationMap.clone();
          binary.setTo(255, initalizationMap >= 254);
          binary.setTo(0, binary != 255);
          cv::distanceTransform(binary, dist, DIST_L2, 3);

          float distThreshM = robot_w_m_;
          float minDist = 99999;
          cv::Point2d SafestGoalPix;
          bool foundSafest = false;

          cerr << " distThreshM " << distThreshM << endl;
          Mat distGray;
          normalize(dist, distGray, 0, 1.0, NORM_MINMAX);

          // try find a safe goal (the value of the pix in the dist map is bigger than distThreshM)
          // next to robot aand also the closest one

          for (int j = 0; j < binary.rows; j++)
          {
            for (int i = 0; i < binary.cols; i++)
            {
              if (binary.at<uchar>(j, i) != 255)
              {
                continue;
              }

              float distM = dist.at<float>(j, i) * mapResolution_;

              if (distM > distThreshM)
              {
                geometry_msgs::Quaternion q;
                auto poseT = convertPixToPose(cv::Point2d(i, j), q);

                float dist = goalCalculator.distanceCalculate(
                    cv::Point2d(poseT.pose.position.x, poseT.pose.position.y),
                    cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

                if (dist < minDist)
                {
                  foundSafest = true;
                  minDist = dist;
                  SafestGoalPix = cv::Point2d(i, j);
                }
              }
            }
          }

          if (!foundSafest)
          {
            return false;
          }

          cerr << " found other safe goal, send goal there " << endl;
          logManager_.writeToLog("found other safe goal, send goal there");

          // create goal (pose )
          auto q = tf::createQuaternionMsgFromYaw(robotHeading);
          auto nextGoal = convertPixToPose(SafestGoalPix, q);

          visualization_msgs::Marker marker;
          marker.header.frame_id = globalFrame_;
          marker.header.stamp = ros::Time::now();
          marker.id = rand();
          marker.type = visualization_msgs::Marker::ARROW;
          marker.pose.position.x = nextGoal.pose.position.x;
          marker.pose.position.y = nextGoal.pose.position.y;
          marker.pose.position.z = 0.5;
          marker.pose.orientation.x = nextGoal.pose.orientation.x;
          marker.pose.orientation.y = nextGoal.pose.orientation.y;
          marker.pose.orientation.z = nextGoal.pose.orientation.z;
          marker.pose.orientation.w = nextGoal.pose.orientation.w;
          marker.scale.x = 0.3;
          marker.scale.y = 0.3;
          marker.scale.z = 0.3;
          marker.color.a = 1.0;  // Don't forget to set the alpha!
          marker.color.r = 0.1;
          marker.color.g = 0.4;
          marker.color.b = 0.5;
          marker.lifetime = ros::Duration(5.0);

          backward_goal_marker_pub_.publish(marker);

          bool result = sendGoal(nextGoal, -1, false, true);

          updateRobotLocation();

          if (!result)
          {
            float dist =
                goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                                 cv::Point2d(nextGoal.pose.position.x, nextGoal.pose.position.y));

            cerr << " the dist to safe goal " << dist << endl;
            if (!(dist < 0.3))
            {
              cerr << " the robot failed to reach the safe goal !! "
                   << " trial : " << trial << endl;
              logManager_.writeToLog("the robot failed to reach the safe goal !! trial" + to_string(trial));

              return false;
            }
          }

          // goal reached, check if can rotate in place
          cerr << "safe goal reached, check if can rotate in place " << endl;
          logManager_.writeToLog("safe goal reached, check if can rotate in place");

          bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

          if (canRotateInPlace)
          {
            cerr << " the robot is able to rotate in the safe goal " << endl;
            logManager_.writeToLog("the robot is able to rotate in the safe goal");

            // rotate the robot in-place numOfRounds times
            if (!rotateInPlace(numOfRounds))
            {
              cerr << " person detectd !!! " << endl;
              logManager_.writeToLog(" person detectd !!!");

              turnOffLamp();
              saveCoverageImg();
              setState("PERSON_DETECTED");

              return false;
            }

            // robot was able to rotate in place !!

            cerr << " finished rotate-in-place, turn on the lamp" << endl;
            logManager_.writeToLog("finished rotate-in-place, turn on the lamp");

            // TURN-N THE LIGHT
            turnOnLamp();

            initializationGood_ = true;

            return true;
          }
          else
          {
            cerr << " the robot cant rotate in this safe goal "
                 << " trial : " << trial << endl;
            logManager_.writeToLog("the robot cant rotate in this safe goal, trial " + to_string(trial));

            continue;
          }
        }
      }
    }
    else
    {
      cerr << " error getting map " << endl;
      return false;
    }

    return false;
  }

  void coverage()
  {
    int countNumberOfTriesFinsihGoal = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      if (!initSlamMap_)
      {
        continue;
      }

      if (!updateRobotLocation())
      {
        continue;
      }

      if (exit_)
      {
        saveCoverageImg();
        turnOffLamp();
        setState("USER_CTRL_C");
        return;
      }

      if (exitPerson_)
      {
        turnOffLamp();
        saveCoverageImg();
        setState("PERSON_DETECTED");
        return;
      }

      switch (coverage_state_)
      {
        case COVERAGE_BY_STRAIGHT_LINES: {
          cerr << "COVERAGE_BY_STRAIGHT_LINES " << endl;
          logManager_.writeToLog("COVERAGE_BY_STRAIGHT_LINES");

          //   clearAllCostMaps();

          currentAlgoMap_ = getCurrentMap();

          // calculate goal-distance-transform-map
          cv::Mat distanceTransformImg;

          updateRobotLocation();

          cv::Point2d currentPosition = convertPoseToPix(startingLocation_);

          cv::Point2d goal = currentPosition;

          // // calc the distance-transform-img from current goal
          if (!distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_, currentPosition,
                                                                        distanceTransformImg, 1))
          {
            cerr << " failed to calcutate the distance transform img" << endl;
            coverage_state_ = ERROR_COVERAGE;
            break;
          }

          // calc the gird coverage
          auto pathGrid = disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition, goal,
                                                                distanceTransformImg, getDistanceBetweenGoalsPix());

          // convert the path into poses
          grid_poses_with_status_.setPixelsPath(pathGrid);
          grid_poses_with_status_.coveragePathPoses_ = covertPointsPathToPoseRout(pathGrid);
          grid_poses_with_status_.initStatusList();

          // exectute currnet navigation the blob-coverage
          cerr << "num of coverage waypoints " << grid_poses_with_status_.coveragePathPoses_.size() << " path_ size is "
               << pathGrid.size() << endl;
          logManager_.writeToLog("num of coverage waypoints " +
                                 to_string(grid_poses_with_status_.coveragePathPoses_.size()));

          bool coverage_done = false;
          float coveragePer = 0.0;

          vector<cv::Point2d> rejectedGoals;
          cerr << " Marks all the waypoints that the robot has already visited during EXPLORATION " << endl;
          // Marks all the waypoints that the robot has already visited during EXPLORATION
          markedWayPointsByExploration();

          while (!coverage_done && ros::ok())
          {
            ros::spinOnce();

            if (exit_)
            {
              setState("USER_CTRL_C");
              saveCoverageImg();
              turnOffLamp();
              return;
            }

            if (exitPerson_)
            {
              turnOffLamp();
              saveCoverageImg();
              setState("PERSON_DETECTED");
              return;
            }

            currentAlgoMap_ = getCurrentMap();
            updateRobotLocation();

            // addd the infaltion by global cost-map
            addDilationByGlobalCostMap(costMapImg_, currentAlgoMap_, convertPoseToPix(robotPose_));

            // marked waypoints near marked REJECTED goals
            for (int i = 0; i < rejectedGoals.size(); i++)
            {
              for (int j = 0; j < grid_poses_with_status_.coveragePathPoses_.size(); j++)
              {
                if (grid_poses_with_status_.status_[j] == UN_COVERED)
                {
                  auto wayPointPix = convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[j]);

                  float dist = goalCalculator.distanceCalculate(wayPointPix, rejectedGoals[i]);

                  if (dist < ((1.0 / mapResolution_) * (0.2)))
                  {
                    grid_poses_with_status_.setStatByIndex(j, REJECTED);
                  }
                }
              }
            }

            auto robotPix = convertPoseToPix(robotPose_);

            float robotHeading = atan2((2.0 * (robotPose_.pose.orientation.w * robotPose_.pose.orientation.z +
                                               robotPose_.pose.orientation.x * robotPose_.pose.orientation.y)),
                                       (1.0 - 2.0 * (robotPose_.pose.orientation.y * robotPose_.pose.orientation.y +
                                                     robotPose_.pose.orientation.z * robotPose_.pose.orientation.z)));

            cv::Point2d finalGoalToNavigate;
            int bestGoalIndexWaypoint;

            // find next goal by 8 directions algo
            auto foundGoalByDirection =
                findNextGoalByDirection(finalGoalToNavigate, &bestGoalIndexWaypoint, robotPix, robot_w_m_,
                                        mapResolution_, robotHeading, currentAlgoMap_);

            if (!foundGoalByDirection)  // find next goal by connected-components algo
            {
              cerr << " try to finld goal by findNextGoalByConnetctedComponents ... " << endl;
              logManager_.writeToLog("try to finld goal by findNextGoalByConnetctedComponents ... ");

              auto foundGoalByConnectedComponents = findNextGoalByConnetctedComponents(
                  finalGoalToNavigate, currentAlgoMap_, grid_poses_with_status_, distBetweenGoalsM_, mapResolution_);

              // found goal by connected component algo
              if (foundGoalByConnectedComponents)
              {
                cerr << " found goal by connected component !!" << endl;
                logManager_.writeToLog("found goal by connected component !!");

                /// calculate the goal
                geometry_msgs::Quaternion q;
                auto goalHeading = -1 * atan2(finalGoalToNavigate.y - robotPix.y, finalGoalToNavigate.x - robotPix.x);
                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, -1 * goalHeading);
                q.w = orientation.getW();
                q.x = orientation.getX();
                q.y = orientation.getY();
                q.z = orientation.getZ();

                auto nextGoal = convertPixToPose(finalGoalToNavigate, q);

                /// calculate wanted path
                nav_msgs::Path wanted_path;
                bool resMakePlan = makePlan(wanted_path, robotPose_, nextGoal);
                cerr << " resMakePlan " << resMakePlan << endl;
                if (resMakePlan)
                {
                  logManager_.writeToLog("trobot was able to make plan to the connected component goal");

                  bool reverseDone = reverseLogic(wanted_path, nextGoal);
 
                  logManager_.writeToLog("the reverse (backward to the safe goal) was down");
                  cerr << " reverseDone " << reverseDone << endl;

                  if (reverseDone)
                  {
                    continue;
                  }
                }
                else
                { 
                 logManager_.writeToLog("failed to make plan fo connectecd compoennet goal, mark it");

                  cerr << " failed to make plan fo connectecd compoennet goal, mark it " << endl;
                  rejectedGoals.push_back(finalGoalToNavigate);
                  continue;
                }

                // a goal can be sent and there will be no need to reverse
                bool result = sendGoal(nextGoal, -1, true);

                // goal of connected component failed, marks this area as obstacle in algo map
                if (!result)
                {
                  cerr << " failed to send goal connectecd compoennet " << endl;
                  logManager_.writeToLog("failed to send goal connectecd compoennet");

                  rejectedGoals.push_back(finalGoalToNavigate);
                }

                // if the robot in farward mode, was able to make plan, but goal aborted
                if (resMakePlan && !result)
                {
                  Mat safetyMap;
                  // check if robot cant rotate in place, if so make small reverse
                  if (getSafetyMap(safetyMap))
                  {
                    bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

                    if (!canRotateInPlace)
                    { 

                      logManager_.writeToLog("the robot was able to make plan but failed to reach goal and cant canRotateInPlace, make small reverse");

                      smallReverseAllowed_ = true;

                      makeSmallReverse();

                      smallReverseAllowed_ = false;
                    }
                  }
                }

                continue;
              }
              else
              {
                cerr << " failed to find by connected components " << endl;
                logManager_.writeToLog("coverage_done: failed to find by connected components");

                coverage_done = true;
                break;
              }
            }

            cerr << " send the goal by direction only !!! " << endl;
            logManager_.writeToLog("send the goal by direction only !!!");

            // send the goal by direction only !!!
            percentCoverage_ = getCurrentPercentCoverage();
            node_.setParam("/coverage/percentage", percentCoverage_);

            /// calculate the goal !!
            geometry_msgs::Quaternion q;
            auto goalHeading = -1 * atan2(finalGoalToNavigate.y - robotPix.y, finalGoalToNavigate.x - robotPix.x);
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, -1 * goalHeading);
            q.w = orientation.getW();
            q.x = orientation.getX();
            q.y = orientation.getY();
            q.z = orientation.getZ();

            auto nextGoal = convertPixToPose(finalGoalToNavigate, q);

            /// calculate wanted path
            nav_msgs::Path wanted_path;
            bool resMakePlan = makePlan(wanted_path, robotPose_, nextGoal);
            cerr << " resMakePlan " << resMakePlan << endl;
            if (resMakePlan)
            { 
              logManager_.writeToLog("trobot was able to make plan to the direction-goal");

              bool reverseDone = reverseLogic(wanted_path, nextGoal);
              logManager_.writeToLog("the reverse (backward to the safe goal) was down");

              cerr << " reverseDone " << reverseDone << endl;

              if (reverseDone)
              {
                continue;
              }
            }
            else
            {
              // dont send the goal beacsue plan no valid!!
              logManager_.writeToLog(" failed to make plan for direction goal, mark it");

              cerr << " failed to make plan for direction goal, mark it " << endl;
              grid_poses_with_status_.setStatByIndex(bestGoalIndexWaypoint, REJECTED);
              continue;
            }

            bool result = sendGoal(nextGoal, bestGoalIndexWaypoint);

            // if the robot in farward mode, was able to make plan, but goal aborted
            if (resMakePlan && !result)
            {
              Mat safetyMap;
              // check if robot cant rotate in place, if so make small reverse
              if (getSafetyMap(safetyMap))
              {
                bool canRotateInPlace = checkIFsafeToRotate(safetyMap, robotPose_, robot_w_m_, robot_h_m_, 0, 0.2);

                if (!canRotateInPlace)
                { 
                 logManager_.writeToLog("the robot was able to make plan but failed to reach goal and cant canRotateInPlace, make small reverse");

                  smallReverseAllowed_ = true;
                  makeSmallReverse();

                  smallReverseAllowed_ = false;
                }
              }
            }

            grid_poses_with_status_.setStatByIndex(bestGoalIndexWaypoint, REJECTED);
          }

          coverage_state_ = BACK_TO_STARTING_LOCATION;
          break;
        }

        case BACK_TO_STARTING_LOCATION: {
          cerr << " BACK_TO_STARTING_LOCATION : " << startingLocation_ << " : " << countNumberOfTriesFinsihGoal << endl;

          logManager_.writeToLog("BACK_TO_STARTING_LOCATION");

          if (countNumberOfTriesFinsihGoal > 3)
          {
            coverage_state_ = COVERAGE_DONE;

            break;
          }

          /// calculate wanted path
          nav_msgs::Path wanted_path;
          bool resMakePlan = makePlan(wanted_path, robotPose_, startingLocation_);
          cerr << " resMakePlan " << resMakePlan << endl;
          if (resMakePlan)
          {
            bool reverseDone = reverseLogic(wanted_path, startingLocation_);
            cerr << " reverseDone " << reverseDone << endl;

            if (reverseDone)
            {
              coverage_state_ = BACK_TO_STARTING_LOCATION;
              break;
            }
          }

          bool result = sendGoal(startingLocation_, -1, true);

          if (result)
          {
            cerr << "BACK_TO_STARTING_LOCATION reached!" << endl;

            coverage_state_ = COVERAGE_DONE;

            break;
          }
          else
          {
            cerr << " Failed to reach BACK_TO_STARTING_LOCATION, trying again" << endl;

            coverage_state_ = BACK_TO_STARTING_LOCATION;

            countNumberOfTriesFinsihGoal++;

            break;
          }
        }
        case COVERAGE_DONE: {
          cerr << " COVERAGE_DONE " << endl;

          lampTimer_.stop();

          setState("COVERAGE_DONE");

          saveCoverageImg();

          coverage_state_ = COVERAGE_DONE;

          break;
        }

        case ERROR_COVERAGE: {
          lampTimer_.stop();

          if (!errCoverage_)
          {
            cerr << " ERROR_COVERAGE " << endl;
            errCoverage_ = true;
          }

          node_.setParam("/coverage/state", "STOPPED");

          coverage_state_ = ERROR_COVERAGE;

          if (exit_)
          {
            saveCoverageImg();
            turnOffLamp();

            setState("USER_CTRL_C");
            return;
          }

          if (exitPerson_)
          {
            turnOffLamp();
            saveCoverageImg();
            setState("PERSON_DETECTED");

            return;
          }

          break;
        }
      }
    }
  }

  static void mySigintHandler(int sig, void* ptr)
  {
    cerr << " user pressed CTRL+C " << endl;
    exit_ = true;
  }

private:
  void updateTimerCallback(const ros::TimerEvent&)
  {
    // if coverage done

    if (coverage_state_ == COVERAGE_DONE)
    {
      turnOffLamp();

      return;
    }

    // if we are still in  INITIALIZATION, do nothing

    if ((state_ == "INITIALIZATION" || state_ == "IDLE" || !initializationGood_))
    {
      return;
    }

    // in coverage/ exploration

    if (detectedPerson_)
    {
      cerr << " PERSON_DETECTED " << endl;

      setState("PERSON_DETECTED");

      moveBaseController_.moveBaseClient_.cancelAllGoals();

      turnOffLamp();

      exitPerson_ = true;

      return;
    }
    else if (initializationGood_)
    {
      turnOnLamp();

      return;
    }

    turnOffLamp();
  }
  void clearAllCostMaps()
  {
    cerr << " clearAllCostMaps " << endl;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    std_srvs::Empty srv;
    if (client.call(srv))
    {
      cerr << " clearing costmaps!!!!!!" << endl;
      ros::Duration(2).sleep();
    }
    else
    {
      cerr << "errrror ->>>> clearing costmaps!!!!!!" << endl;
    }
  }

  bool makePlan(nav_msgs::Path& path, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;

    if (client.call(srv))
    {
      cerr << " plan size " << srv.response.plan.poses.size() << endl;

      path = srv.response.plan;

      if (srv.response.plan.poses.size() == 0)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    else
    {
      return false;
    }

    return true;
  }

  void markedWayPointsByExploration()
  {
    for (int j = 0; j < robotHistoryPathMsg_.poses.size(); j++)
    {
      for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
      {
        if (grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH ||
            grid_poses_with_status_.status_[i] == REJECTED)
        {
          continue;
        }

        if (goalCalculator.distanceCalculate(
                cv::Point2d(robotHistoryPathMsg_.poses[j].pose.position.x,
                            robotHistoryPathMsg_.poses[j].pose.position.y),
                cv::Point2d(grid_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                            grid_poses_with_status_.coveragePathPoses_[i].pose.position.y)) < sanitization_radius_)
        {
          grid_poses_with_status_.status_[i] = COVERED_BY_ROBOT_PATH;
        }
      }
    }
  }
  void publishSanitizationRadius()
  {
    std::srand(std::time(nullptr));  // use current time as seed for random generator

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = baseFrame_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 6000;
    line_strip.lifetime = ros::Duration(1.0);

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    if (smallReverseAllowed_)
    {
      line_strip.scale.x = 0.1;
      line_strip.color.b = 0.0;
      line_strip.color.g = 0.1;
      line_strip.color.r = 0.5;
      line_strip.color.a = 1.0;
    }
    else
    {
      if (reversAllowed_)
      {
        line_strip.scale.x = 0.1;
        line_strip.color.b = 0.0;
        line_strip.color.g = 0.0;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
      }
      else
      {
        line_strip.scale.x = 0.1;
        line_strip.color.b = 0.5;
        line_strip.color.g = 1.5;
        line_strip.color.r = 0.0;
        line_strip.color.a = 1.0;
      }
    }

    for (uint32_t i = 0; i < 360; ++i)
    {
      geometry_msgs::Point p;

      p.y = (sanitization_radius_)*sin(angles::from_degrees(i));
      p.x = (sanitization_radius_)*cos(angles::from_degrees(i));
      p.z = 0.5;

      line_strip.points.push_back(p);
    }

    sanitization_radius_marker_pub_.publish(line_strip);
  }

  float getScoreByDirection(float rotattionAngle, const cv::Point2d& refPoint1, const cv::Point2d& refPoint2,
                            cv::Point2d& bestGoal, int* goalIndex, const cv::Point2d& robotPix, float robotHeading,
                            float robot_w_m, float mapResolution_, const Mat& imgMap, vector<int>& waypointsInPathDebug,
                            string direc, float minAreaM = 0.0, float maxDistanceM = 4.0)
  {
    // cerr << direc << endl;

    cv::Mat workMap = imgMap.clone();

    // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

    float robot_W_pix = robot_w_m / mapResolution_;

    float len = robot_W_pix;

    float size = (robot_W_pix / 2);

    int unCovered = 0;

    cv::Point2d middle;

    cv::Point2d robotHeadingPoint(robotPix.x + (10) * cos(robotHeading), robotPix.y + (10) * sin(robotHeading));
    // cv::arrowedLine(dbg, robotPix, robotHeadingPoint, Scalar(80, 127, 255), 2,
    //                 8, 0, 0.3);

    // for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
    // {

    //     switch (grid_poses_with_status_.status_[i])
    //     {
    //         case UN_COVERED:
    //         {

    //             circle(dbg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]) , 1, Scalar(0, 255, 0),
    //             -1, 8, 0); break;
    //         }
    //         case COVERED:
    //         {
    //             circle(dbg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]) , 1, Scalar(255, 0, 0),
    //             -1, 8, 0); break;
    //         }
    //         case COVERED_BY_ROBOT_PATH:
    //         {
    //             circle(dbg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]) ,1, Scalar(255, 0, 0),
    //             -1, 8, 0); break;
    //         }
    //         case COVERED_BY_OBSTACLE:
    //         {
    //             circle(dbg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 0, 255),
    //             -1, 8, 0); break;
    //         }
    //     }
    // }

    while (ros::ok())
    {
      cv::Point2d leftSide(refPoint1.x + (size)*cos(rotattionAngle), refPoint1.y + (size)*sin(rotattionAngle));

      cv::Point2d rightSide(refPoint2.x + (size)*cos(rotattionAngle), refPoint2.y + (size)*sin(rotattionAngle));

      if (leftSide.x < 0 || leftSide.x > workMap.cols || leftSide.y < 0 || leftSide.y > workMap.rows)
      {
        break;
      }

      // contour represent the area of searching uncovered goals
      vector<cv::Point> contour{ refPoint1, refPoint2, rightSide, leftSide };

      // cv::polylines(dbg, contour, true, Scalar(255, 0, 0), 1);

      middle = cv::Point2d((leftSide.x + rightSide.x) / 2, (leftSide.y + rightSide.y) / 2);

      float minDistFromMiddle = 9999.0;

      float distFromRobot = goalCalculator.distanceCalculate(middle, robotPix);

      // we finsih this direction (obstacle or max dist)
      if (workMap.at<uchar>(leftSide.y, leftSide.x) != 254 || workMap.at<uchar>(rightSide.y, rightSide.x) != 254 ||
          distFromRobot > (maxDistanceM / mapResolution_))
      {
        for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
        {
          // IF this is uncovered goal and inside the polygon

          if (grid_poses_with_status_.status_[i] == UN_COVERED)
          {
            if (pointPolygonTest(contour, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), false) > 0)
            {
              unCovered++;
              float distFromMiddle = goalCalculator.distanceCalculate(
                  convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), middle);

              if (distFromMiddle < minDistFromMiddle)
              {
                bestGoal = convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]);
                *goalIndex = i;
                minDistFromMiddle = distFromMiddle;
              }
            }
          }
        }

        break;
      }

      size += 1 / mapResolution_;

      // imshow(direc,dbg);
      // waitKey(0);
    }

    // circle(dbg, bestGoal,  5, Scalar(255,0,255), 1, 8, 0);

    // cv::resize(dbg, dbg, cv::Size(dbg.cols * 2, dbg.rows * 2 ));
    // cv::putText(dbg, "unCovered : "+to_string(int(unCovered)), cv::Point(20, 20), 1, 1, Scalar(0));

    // imwrite("/home/yakir/distance_transform_coverage_ws/imgs/" +direc+".png", dbg);

    return unCovered;
  }

  bool updateRobotLocation()
  {
    tf::StampedTransform transform;

    try
    {
      // get current robot pose
      tfListener_.waitForTransform(globalFrame_, baseFrame_, ros::Time(0), ros::Duration(0.01));

      tfListener_.lookupTransform(globalFrame_, baseFrame_, ros::Time(0), transform);

      robotPose_.header.frame_id = globalFrame_;
      robotPose_.header.stamp = ros::Time::now();
      robotPose_.pose.position.x = transform.getOrigin().x();
      robotPose_.pose.position.y = transform.getOrigin().y();
      robotPose_.pose.position.z = 0;
      robotPose_.pose.orientation.x = transform.getRotation().x();
      robotPose_.pose.orientation.y = transform.getRotation().y();
      robotPose_.pose.orientation.z = transform.getRotation().z();
      robotPose_.pose.orientation.w = transform.getRotation().w();

      if (!(state_ == "INITIALIZATION" || state_ == "IDLE"))
      {
        if (robotHistoryPathMsg_.poses.size() > 0)
        {
          int last_index = robotHistoryPathMsg_.poses.size() - 1;

          float diffLocationsM =
              goalCalculator.distanceCalculate(cv::Point2d(robotHistoryPathMsg_.poses[last_index].pose.position.x,
                                                           robotHistoryPathMsg_.poses[last_index].pose.position.y),
                                               cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

          if (diffLocationsM > 0.05)
          {
            robotHistoryPathMsg_.poses.push_back(robotPose_);
            publishSanitizationRadius();
          }
        }
      }

      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());

      cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;

      return false;
    }
  }

  void setOrientationByNextUnCoveredGoal(int index, geometry_msgs::PoseStamped& originalGoal)
  {
    if (index == grid_poses_with_status_.coveragePathPoses_.size() - 1)
    {
      return;
    }

    for (int i = index + 1; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
    {
      if (grid_poses_with_status_.status_[i] == UN_COVERED)
      {
        // this is the next un-covered goals, calculate direction from this goal
        auto nextGoal = grid_poses_with_status_.coveragePathPoses_[i];

        float angle = atan2(originalGoal.pose.position.y - nextGoal.pose.position.y,
                            originalGoal.pose.position.x - nextGoal.pose.position.x);

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, -1 * angle);
        geometry_msgs::Quaternion q;
        q.w = orientation.getW();
        q.x = orientation.getX();
        q.y = orientation.getY();
        q.z = orientation.getZ();

        originalGoal.pose.orientation = q;

        return;
      }
    }
  }

  float getCurrentPercentCoverage()
  {
    if (grid_poses_with_status_.coveragePathPoses_.size() == 0)
    {
      return 0.0;
    }

    float countCovered = 0.0;

    float countNeedToBeCovered = 0.0;

    if (coverage_state_ == COVERAGE_DONE)
    {
      for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
      {
        switch (grid_poses_with_status_.status_[i])
        {
          case REJECTED: {
            // probably this is the aborted goals
            countNeedToBeCovered += 1.0;
          }
          case COVERED_BY_ROBOT_PATH: {
            countNeedToBeCovered += 1.0;
            countCovered += 1.0;
          }
        }
      }
    }
    else
    {
      for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
      {
        switch (grid_poses_with_status_.status_[i])
        {
          case UN_COVERED: {
            countNeedToBeCovered += 1.0;
          }
          case REJECTED: {
            // probably this is the aborted goals
            countNeedToBeCovered += 1.0;
          }
          case COVERED_BY_ROBOT_PATH: {
            countNeedToBeCovered += 1.0;
            countCovered += 1.0;
          }
        }
      }
    }

    if (countNeedToBeCovered == 0.0)
    {
      return 0.0;
    }

    float percentCoverage = (countCovered / countNeedToBeCovered) * 100.0;

    if (percentCoverage > 100)
    {
      percentCoverage = 100.0;
    }

    return percentCoverage;
  }

  void publishRobotHistoryPath()
  {
    robotHistoryPathMsg_.header.stamp = ros::Time::now();
    robotHistoryPathMsg_.header.frame_id = globalFrame_;

    robot_history_path_pub_.publish(robotHistoryPathMsg_);
  }

  void personsCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (msg->data == true)
    {
      detectedPerson_ = true;
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    currOdom_.child_frame_id = msg->child_frame_id;
    currOdom_.header = msg->header;
    currOdom_.pose = msg->pose;
    currOdom_.twist = msg->twist;
  }
  void cameraScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    updateRobotLocation();

    for (double i = 0; i < scan->ranges.size(); i++)
    {
      if (isinf(scan->ranges[i]) == false)
      {
        double ray_angle = scan->angle_min + (i * scan->angle_increment);

        cv::Point2d rayPoint((scan->ranges[i] * cos(ray_angle)), (scan->ranges[i] * sin(ray_angle)));

        // transform to map frame
        cv::Point3d p = cv::Point3d(rayPoint.x, rayPoint.y, 0);

        auto transformedRayPoint = transformFrames(p, globalFrame_, scan->header.frame_id, scan->header.stamp);

        float dist =
            goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                             cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));

        if (dist < 3.0)
        {  // M{

          cameraScanObstacles.push_back(transformedRayPoint);
        }

        // int n_digit = 1;
        // string robot_location_camera_beam =
        //     prd(robotPose_.pose.position.x, n_digit) + "_" + prd(robotPose_.pose.position.y, n_digit) + "_" +
        //     prd(transformedRayPoint.point.x, n_digit) + "_" + prd(transformedRayPoint.point.y, n_digit);

        // float dist =
        //     goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
        //                                      cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));

        // if (dist < 3.0)
        // {  // M{

        //   map<string, int>::iterator i = camera_laser_beams_.find(robot_location_camera_beam);

        //   if (i == camera_laser_beams_.end())
        //   {
        //     camera_laser_beams_[robot_location_camera_beam] = 1;
        //   }
        // }
      }
    }
  }

  bool getSafetyMap(Mat& saftetyMap)
  {
    auto robotBaseFootPrint = convertPoseToPix(robotPose_);

    if (!costMapImg_.data || robotBaseFootPrint.x < 0 || robotBaseFootPrint.y < 0 || !currentGlobalMap_.data ||
        !initSlamMap_)
    {
      cerr << " failed to get safey map " << endl;
      return false;
    }

    Mat globalCostMap = costMapImg_.clone();
    saftetyMap = cv::Mat(currentGlobalMap_.rows, currentGlobalMap_.cols, CV_8UC1, cv::Scalar(205));

    cv::Point2d mapCenter(currentGlobalMap_.cols / 2, currentGlobalMap_.rows / 2);
    cv::Point2d costMapCenter(globalCostMap.cols / 2, globalCostMap.rows / 2);

    int deltaRobotX = mapCenter.x - robotBaseFootPrint.x;
    int deltaRobotY = mapCenter.y - robotBaseFootPrint.y;

    try
    {
      for (int y = 0; y < globalCostMap.rows; y++)
      {
        for (int x = 0; x < globalCostMap.cols; x++)
        {
          int value = globalCostMap.at<uchar>(y, x);

          if (value == 255)  // not infalted
          {
            cv::Point2d nP(x + (mapCenter.x - costMapCenter.x) - deltaRobotX,
                           y + (mapCenter.y - costMapCenter.y) - deltaRobotY);

            if (nP.x > 0 && nP.y > 0 && nP.x < saftetyMap.cols && nP.y < saftetyMap.rows)
            {
              saftetyMap.at<uchar>(nP.y, nP.x) = 0;
            }
          }
        }
      }

      // imwrite("/home/yakir/distance_transform_coverage_ws/saftetyMap.png", saftetyMap);

      return true;
    }
    catch (cv::Exception& e)
    {
      const char* err_msg = e.what();
      std::cerr << "exception caught: " << err_msg << std::endl;
      return false;
    }

    // try
    // {

    //   if (initSlamMap_ && initGlobalCostMap_ && mappingMap_.data && currentGlobalMap_.data)
    //   {
    //     saftetyMap = cv::Mat(currentGlobalMap_.rows, currentGlobalMap_.cols, CV_8UC1, cv::Scalar(205));

    //     return true;
    //   }
    //   else {

    //     cerr<<" initSlamMap_ "<<initSlamMap_<<" initGlobalCostMap_ "<<initGlobalCostMap_<<
    //       " mappingMap_.data "<<mappingMap_.data<<" currentGlobalMap_.data "<<currentGlobalMap_.data<<endl;
    //     return false;
    //   }
    // }
    // catch (cv::Exception& e)
    // {
    //   const char* err_msg = e.what();
    //   std::cerr << "exception caught: " << err_msg << std::endl;

    //   return false;
    // }
  }

  string prd(const double x, const int decDigits)
  {
    stringstream ss;
    ss << fixed;
    ss.precision(decDigits);  // set # places after decimal
    ss << x;
    return ss.str();
  }

  bool findNextGoalByDirection(cv::Point2d& finalGoalToNavigate, int* bestGoalIndex, const cv::Point2d& robotPix,
                               float robot_w_m, float mapResolution_, float robotHeading, const cv::Mat& imgMap)
  {
    float robot_W_pix = robot_w_m / mapResolution_;

    float len = robot_W_pix;

    cv::Point2d robotFootprintFrontLeft(robotPix.x + (len)*cos(robotHeading - angles::from_degrees(45)),
                                        robotPix.y + (len)*sin(robotHeading - angles::from_degrees(45)));

    cv::Point2d robotFootprintFrontRight(robotPix.x + (len)*cos(robotHeading + angles::from_degrees(45)),
                                         robotPix.y + (len)*sin(robotHeading + angles::from_degrees(45)));

    cv::Point2d robotFootprintBackRIGHT(robotPix.x + (len)*cos(robotHeading + angles::from_degrees(45 + 90)),
                                        robotPix.y + (len)*sin(robotHeading + angles::from_degrees(45 + 90)));

    cv::Point2d robotFootprintBackLeft(robotPix.x + (len)*cos(robotHeading - angles::from_degrees(45 + 90)),
                                       robotPix.y + (len)*sin(robotHeading - angles::from_degrees(45 + 90)));

    vector<float> scores;
    vector<int> bestIndexesFromWaypoints;

    // front
    cv::Point2d bestGoalPixFront;
    vector<int> deubugIndexFront;
    int goalINdexFront = -1;
    float rotattionAngleFront = robotHeading + angles::from_degrees(0.0);
    scores.push_back(getScoreByDirection(rotattionAngleFront, robotFootprintFrontLeft, robotFootprintFrontRight,
                                         bestGoalPixFront, &goalINdexFront, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexFront, "front"));
    bestIndexesFromWaypoints.push_back(goalINdexFront);

    // front-left
    cv::Point2d bestGoalPixFrontLeft;
    vector<int> deubugIndexFrontLeft;
    int goalINdexFrontLeft = -1;
    float rotattionAngleFrontLeft = robotHeading + angles::from_degrees(-45.0);
    scores.push_back(getScoreByDirection(rotattionAngleFrontLeft, robotFootprintBackLeft, robotFootprintFrontRight,
                                         bestGoalPixFrontLeft, &goalINdexFrontLeft, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexFrontLeft, "front-left"));
    bestIndexesFromWaypoints.push_back(goalINdexFrontLeft);

    // front-right
    cv::Point2d bestGoalPixFrontRight;
    vector<int> deubugIndexFrontRight;
    int goalINdexFrontRight = -1;
    float rotattionAngleFrontRight = robotHeading + angles::from_degrees(45.0);
    scores.push_back(getScoreByDirection(rotattionAngleFrontRight, robotFootprintBackRIGHT, robotFootprintFrontLeft,
                                         bestGoalPixFrontRight, &goalINdexFrontRight, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexFrontRight, "front-right"));
    bestIndexesFromWaypoints.push_back(goalINdexFrontRight);

    // back
    cv::Point2d bestGoalPixBack;
    vector<int> deubugIndexBack;
    int goalINdexBack = -1;
    float rotattionAngleBack = robotHeading + angles::from_degrees(180);
    scores.push_back(getScoreByDirection(rotattionAngleBack, robotFootprintBackLeft, robotFootprintBackRIGHT,
                                         bestGoalPixBack, &goalINdexBack, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexBack, "back"));
    bestIndexesFromWaypoints.push_back(goalINdexBack);

    // back-left
    cv::Point2d bestGoalPixBackLeft;
    vector<int> deubugIndexBackLeft;
    int goalINdexBackLeft = -1;
    float rotattionAngleBackLeft = robotHeading + angles::from_degrees(180 + 45);
    scores.push_back(getScoreByDirection(rotattionAngleBackLeft, robotFootprintBackRIGHT, robotFootprintFrontLeft,
                                         bestGoalPixBackLeft, &goalINdexBackLeft, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexBackLeft, "back-left"));
    bestIndexesFromWaypoints.push_back(goalINdexBackLeft);

    // back-left
    cv::Point2d bestGoalPixBackRight;
    vector<int> deubugIndexBackRight;
    int goalINdexBackRight = -1;
    float rotattionAngleBackRight = robotHeading + angles::from_degrees(180 - 45);
    scores.push_back(getScoreByDirection(rotattionAngleBackRight, robotFootprintBackLeft, robotFootprintFrontRight,
                                         bestGoalPixBackRight, &goalINdexBackRight, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexBackRight, "back-right"));
    bestIndexesFromWaypoints.push_back(goalINdexBackRight);

    // left
    cv::Point2d bestGoalPixLeft;
    vector<int> deubugIndexLeft;
    int goalINdexLeft = -1;
    float rotattionAngleLeft = robotHeading + angles::from_degrees(-90.0);
    scores.push_back(getScoreByDirection(rotattionAngleLeft, robotFootprintFrontLeft, robotFootprintBackLeft,
                                         bestGoalPixLeft, &goalINdexLeft, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexLeft, "left"));
    bestIndexesFromWaypoints.push_back(goalINdexLeft);

    // right
    cv::Point2d bestGoalPixRight;
    vector<int> deubugIndexRight;
    int goalINdexRight = -1;
    float rotattionAngleRight = robotHeading + angles::from_degrees(90.0);
    scores.push_back(getScoreByDirection(rotattionAngleRight, robotFootprintFrontRight, robotFootprintBackRIGHT,
                                         bestGoalPixRight, &goalINdexRight, robotPix, robotHeading, robot_w_m,
                                         mapResolution_, imgMap, deubugIndexRight, "right"));
    bestIndexesFromWaypoints.push_back(goalINdexRight);

    vector<cv::Point2d> goals{ bestGoalPixFront,    bestGoalPixFrontLeft, bestGoalPixFrontRight, bestGoalPixBack,
                               bestGoalPixBackLeft, bestGoalPixBackRight, bestGoalPixLeft,       bestGoalPixRight };

    vector<vector<int>> waypointsIndexses{ deubugIndexFront, deubugIndexFrontLeft, deubugIndexFrontRight,
                                           deubugIndexBack,  deubugIndexBackLeft,  deubugIndexBackRight,
                                           deubugIndexLeft,  deubugIndexRight };

    visualization_msgs::MarkerArray Markerarr;

    for (int i = 0; i < goals.size(); i++)
    {
      geometry_msgs::Quaternion q;

      float angle = atan2(goals[i].y - robotPix.y, goals[i].x - robotPix.x);
      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, angle);
      q.w = orientation.getW();
      q.x = orientation.getX();
      q.y = orientation.getY();
      q.z = orientation.getZ();

      geometry_msgs::PoseStamped p = convertPixToPose(goals[i], q);

      visualization_msgs::Marker marker;
      marker.header.frame_id = globalFrame_;
      marker.header.stamp = ros::Time::now();
      marker.id = i + 1;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.pose.position.x = p.pose.position.x;
      marker.pose.position.y = p.pose.position.y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = q.x;
      marker.pose.orientation.y = q.y;
      marker.pose.orientation.z = q.z;
      marker.pose.orientation.w = q.w;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.text = to_string(int(scores[i]));
      marker.lifetime = ros::Duration(10.0);

      Markerarr.markers.push_back(marker);
    }

    directions_marker_array_pub_.publish(Markerarr);

    // for(int i = 0; i < goals.size(); i++){
    //     if ( goals[i].x != 0 && goals[i].y != 0)
    //     cv::arrowedLine(dbg, robotPix, goals[i], Scalar(0), 1);

    // }

    float maxScore = 0.0;
    float index = -1;
    int score_threshold = 2;

    for (int i = 0; i < goals.size(); i++)
    {
      if (scores[i] > maxScore && scores[i] > score_threshold)
      {
        index = i;
        maxScore = scores[i];
      }
    }

    if (index != -1)
    {
      finalGoalToNavigate = goals[index];
      *bestGoalIndex = bestIndexesFromWaypoints[index];
    }
    else
    {
      return false;
    }

    cerr << "maxScore " << maxScore << endl;

    if (maxScore > score_threshold)
    {
      return true;
    }

    return false;
  }

  bool findSafestGoalFromUncoveredGoals(cv::Point2d& nextGoal, const cv::Mat& imgMap,
                                        const Path_with_Status& waypointsWithStatus, float goals_m_resolution,
                                        float map_resolution, float miAreaForComponentM = 0.2)
  {
    Mat binary = cv::Mat(imgMap.rows, imgMap.cols, CV_8UC1, cv::Scalar(0));

    float pixRes = goals_m_resolution / map_resolution;
    for (int i = 0; i < waypointsWithStatus.status_.size(); i++)
    {
      if (waypointsWithStatus.status_[i] == UN_COVERED)
      {
        circle(binary, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), pixRes, Scalar(255), -1, 8, 0);
      }
    }

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

    if (contours.size() == 0)
    {
      return false;
    }

    auto robotPix = convertPoseToPix(robotPose_);

    float maxArea = 0.0;
    int index = -1;
    float minDist = 99999.9;

    // find the closest connected component
    for (int i = 0; i < contours.size(); i++)
    {
      Rect r = cv::boundingRect(contours[i]);
      float areaM = (r.width * map_resolution) * (r.height * map_resolution);

      Point center_of_rect = (r.br() + r.tl()) * 0.5;
      float dist = goalCalculator.distanceCalculate(center_of_rect, robotPix);

      if (areaM > miAreaForComponentM)
      {
        if (dist < minDist)
        {
          minDist = dist;
          index = i;
        }
        else
        {
          drawContours(binary, contours, i, Scalar(0), -1);
        }
      }
      else
      {
        drawContours(binary, contours, i, Scalar(0), -1);
      }
    }

    if (index == -1)
    {
      return false;
    }

    try
    {
      cv::Rect r = cv::boundingRect(contours[index]);
      cv::Mat cropped = binary(r);
      Mat dist;
      cv::distanceTransform(cropped, dist, DIST_L2, 3);
      // Normalize the distance image for range = {0.0, 1.0}
      // so we can visualize and threshold it
      normalize(dist, dist, 0, 1.0, NORM_MINMAX);

      double min, max;
      cv::Point minLoc;
      cv::Point maxLoc;
      minMaxLoc(dist, &min, &max, &minLoc, &maxLoc);

      if (!(maxLoc.x > 0 && maxLoc.y > 0))
      {
        cerr << " failed to find safe goal";
        return false;
      }
      else
      {
        nextGoal = cv::Point2d(maxLoc.x + r.x, maxLoc.y + r.y);
        cerr << " found " << nextGoal << endl;
        return true;
      }
    }
    catch (cv::Exception& e)
    {
      const char* err_msg = e.what();
      std::cerr << "exception caught: " << err_msg << std::endl;

      return false;
    }

    return false;
  }

  bool findNextGoalByConnetctedComponents(cv::Point2d& finalGoalToNavigate, const Mat& imgMap,
                                          const Path_with_Status& waypointsWithStatus, float goals_m_resolution,
                                          float mapResolution_)
  {
    if (findSafestGoalFromUncoveredGoals(finalGoalToNavigate, imgMap, waypointsWithStatus, goals_m_resolution,
                                         mapResolution_))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  geometry_msgs::PointStamped transformFrames(Point3d objectPoint3d, string target_frame, string source_Frame,
                                              ros::Time t)
  {
    geometry_msgs::PointStamped pointStampedIn;
    geometry_msgs::PointStamped pointStampedOut;

    pointStampedIn.header.frame_id = source_Frame;
    pointStampedIn.header.stamp = t;
    pointStampedIn.point.x = objectPoint3d.x;
    pointStampedIn.point.y = objectPoint3d.y;
    pointStampedIn.point.z = objectPoint3d.z;

    try
    {
      tf::StampedTransform transform;

      tfListener_.waitForTransform(target_frame, source_Frame, ros::Time(0), ros::Duration(0.01));

      tfListener_.lookupTransform(target_frame, source_Frame, ros::Time(0), transform);

      tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

      return pointStampedOut;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());

      return pointStampedOut;
    }
  }

  Mat occupancyGridMatToGrayScale(const Mat& map)
  {
    // CV_8UC1 - UINT image type
    Mat output(map.rows, map.cols, CV_8UC1, Scalar(205));

    for (int j = 0; j < map.rows; j++)
    {
      for (int i = 0; i < map.cols; i++)
      {
        auto value = map.at<int8_t>(cv::Point(i, j));
        uint8_t newValue = 0;

        if (value == UNKNOWN)  // unknown
          newValue = 205;
        else if (value == FREE)  // clear
          newValue = 254;
        else if (value == BLOCKED)  // occupay
          newValue = 0;

        output.at<uchar>(cv::Point(i, j)) = newValue;
      }
    }

    return output;
  }

  void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    globalMapOriginPositionX_ = msg->info.origin.position.x;
    globalMapOriginPositionY_ = msg->info.origin.position.y;
    globalMapResolution_ = msg->info.resolution;

    auto endLocalCostMap = high_resolution_clock::now();
    auto durationFromLastCalc = duration_cast<seconds>(endLocalCostMap - startLocalCostMap_).count();

    /// do this every 2 seconds
    if (initSlamMap_ && durationFromLastCalc > 0.5)
    {
      costMapImg_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1, Scalar(0));
      memcpy(costMapImg_.data, msg->data.data(), msg->info.height * msg->info.width);

      for (int j = 0; j < costMapImg_.rows; j++)
      {
        for (int i = 0; i < costMapImg_.cols; i++)
        {
          int value = costMapImg_.at<uchar>(j, i);

          if (value == 100)
          {
            costMapImg_.at<uchar>(j, i) = 255;  ///
          }
          else if (value > 0 && value < 100)
          {
            costMapImg_.at<uchar>(j, i) = 100;  // inflation
          }
        }
      }

      initGlobalCostMap_ = true;

      string global_costmap_frame = msg->header.frame_id;

      // Mat dbg = costMapImg.clone();
      // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

      for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
      {
        // if this goal cant be n obstacle
        if (grid_poses_with_status_.status_[i] == REJECTED ||
            grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
        {
          continue;
        }

        // transform to odom frame (global costmap framme)
        cv::Point3d p = cv::Point3d(grid_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                                    grid_poses_with_status_.coveragePathPoses_[i].pose.position.y, 0);

        auto poseInOdomFrame = transformFrames(p, global_costmap_frame, globalFrame_, msg->header.stamp);

        // convert odom pose to odom pix
        float xPix = (poseInOdomFrame.point.x - globalMapOriginPositionX_) / globalMapResolution_;
        float yPix = (poseInOdomFrame.point.y - globalMapOriginPositionY_) / globalMapResolution_;

        cv::Point pOnImg = cv::Point(xPix, yPix);

        // get the cost value
        int costVal = costMapImg_.at<uchar>(pOnImg.y, pOnImg.x);

        float distRobotFromGoal = goalCalculator.distanceCalculate(
            cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
            cv::Point2d(grid_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                        grid_poses_with_status_.coveragePathPoses_[i].pose.position.y));

        // GOAL ON obstacle
        if (costVal != 0)
        {
          // the goal inside the wanted radius
          grid_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);

          // circle(dbg, pOnImg,  1, Scalar(0,255,0), -1, 8, 0);
        }
        /// goal not inside obstacle
        else
        {
          // if inside radius but last time int was inside obstacle,
          // keep it as obstacle
          if (grid_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
          {
            grid_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);
          }
          else
          {
            grid_poses_with_status_.setStatByIndex(i, UN_COVERED);
          }
        }
      }

      // imwrite("/home/algo-kobuki/imgs/dbg.png", dbg);
      // imwrite("/home/algo-kobuki/imgs/gmapping.png", currentGlobalMap_);

      startLocalCostMap_ = endLocalCostMap;
    }
  }

  void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    // Check if the global map size changed
    if (width_ != -1 && height_ != -1)
    {
      if (msg->info.width != width_ || msg->info.height != height_)
      {
        double old_origin_x = map_origin_position_x;
        double old_origin_y = map_origin_position_y;

        double new_origin_x = msg->info.origin.position.x;
        double new_origin_y = msg->info.origin.position.y;

        double deltaW = msg->info.width - width_;
        double deltaH = msg->info.height - height_;

        updateCurrentBlobsFrontiers(old_origin_x, old_origin_y, new_origin_x, new_origin_y, deltaW, deltaH);
      }
    }

    cv::Mat tmp = cv::Mat(msg->info.height, msg->info.width, CV_8SC1, Scalar(UNKNOWN));
    memcpy(tmp.data, msg->data.data(), msg->info.height * msg->info.width);

    globalMapWidth_ = msg->info.width;

    globalMapHeight_ = msg->info.height;

    mapResolution_ = msg->info.resolution;

    map_origin_position_x = msg->info.origin.position.x;

    map_origin_position_y = msg->info.origin.position.y;

    distanceTransformGoalCalculator.setResolution(getDistanceBetweenGoalsPix());

    initSlamMap_ = true;

    currentGlobalMap_ = occupancyGridMatToGrayScale(tmp.clone());

    mappingMap_ = currentGlobalMap_.clone();

    addDilationForGlobalMap(currentGlobalMap_, 0.05, mapResolution_);

    addFreeSpaceDilation(currentGlobalMap_);
  }

  void updateCurrentBlobsFrontiers(double old_origin_x, double old_origin_y, double new_origin_x, double new_origin_y,
                                   double deltaW, double deltaH)
  {
    for (int i = 0; i < currentBlobsFrontiers_.size(); i++)
    {
      for (int j = 0; j < currentBlobsFrontiers_[i].contour.size(); j++)
      {
        cv::Point oldGoalPix = currentBlobsFrontiers_[i].contour[j];

        double x_start = -((new_origin_x - old_origin_x) / mapResolution_);
        double y_start = -((new_origin_y - old_origin_y) / mapResolution_);
        cv::Point newGoalPix(oldGoalPix.x + x_start - deltaW, oldGoalPix.y + y_start - deltaH);

        currentBlobsFrontiers_[i].contour[j] = newGoalPix;

        if (j == 0)
        {
          currentBlobsFrontiers_[i].center = newGoalPix;
        }
      }
    }
  }

  void publishEdgesFrontiers(const std::vector<Frontier>& currentEdgesFrontiers)
  {
    visualization_msgs::MarkerArray Markerarr;

    for (int i = 0; i < currentEdgesFrontiers.size(); i++)
    {
      cv::Point pixP = currentEdgesFrontiers[i].center;
      geometry_msgs::Quaternion q;
      q.w = 1;
      geometry_msgs::PoseStamped p = convertPixToPose(pixP, q);

      visualization_msgs::Marker pOnEgge;
      pOnEgge.header.frame_id = globalFrame_;
      pOnEgge.header.stamp = ros::Time::now();
      pOnEgge.ns = "points_and_lines";
      pOnEgge.id = rand();
      pOnEgge.action = visualization_msgs::Marker::ADD;
      pOnEgge.type = visualization_msgs::Marker::SPHERE;
      pOnEgge.pose.position.x = p.pose.position.x;
      pOnEgge.pose.position.y = p.pose.position.y;
      pOnEgge.pose.position.z = 0;
      pOnEgge.pose.orientation.x = 0;
      pOnEgge.pose.orientation.y = 0;
      pOnEgge.pose.orientation.z = 0;
      pOnEgge.pose.orientation.w = 1.0;
      pOnEgge.scale.x = 0.3;
      pOnEgge.scale.y = 0.3;
      pOnEgge.scale.z = 0.3;
      pOnEgge.color.a = 1.0;
      pOnEgge.color.r = 0.0;
      pOnEgge.color.g = 0.2;
      pOnEgge.color.b = 1.0;
      pOnEgge.lifetime = ros::Duration(5.0);

      Markerarr.markers.push_back(pOnEgge);
    }

    edges_frontires_marker_array_Pub.publish(Markerarr);
  }

  void publishWaypointsWithStatus()
  {
    visualization_msgs::MarkerArray Markerarr;
    int count = 1;

    for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = globalFrame_;
      m.header.stamp = ros::Time::now();
      m.ns = "points_and_lines";
      m.id = count + i;
      m.action = visualization_msgs::Marker::ADD;
      m.type = visualization_msgs::Marker::SPHERE;
      m.pose.position.x = grid_poses_with_status_.coveragePathPoses_[i].pose.position.x;
      m.pose.position.y = grid_poses_with_status_.coveragePathPoses_[i].pose.position.y;
      m.pose.position.z = 0;
      m.pose.orientation.x = 0;
      m.pose.orientation.y = 0;
      m.pose.orientation.z = -0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.color.a = 1.0;

      // checked

      /// REJECTED = ORANGE
      if (grid_poses_with_status_.status_[i] == REJECTED)
      {
        int r, g, b;
        sscanf(REJECTED_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
        m.color.r = r / 255.0;
        m.color.g = g / 255.0;
        m.color.b = b / 255.0;

      }  /// UN_COVERED = GREEN
      else if (grid_poses_with_status_.status_[i] == UN_COVERED)
      {
        int r, g, b;
        sscanf(UN_COVERED_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
        m.color.r = r / 255.0;
        m.color.g = g / 255.0;
        m.color.b = b / 255.0;

      }  /// COVERED_BY_ROBOT_PATH = LIGHT BLUE
      else if (grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
      {
        int r, g, b;
        sscanf(COVERED_BY_ROBOT_PATH_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
        m.color.r = r / 255.0;
        m.color.g = g / 255.0;
        m.color.b = b / 255.0;
      }  /// COVERED_BY_OBSTACLE = RED

      else if (grid_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
      {
        int r, g, b;
        sscanf(COVERED_BY_OBSTACLE_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
        m.color.r = r / 255.0;
        m.color.g = g / 255.0;
        m.color.b = b / 255.0;
      }

      Markerarr.markers.push_back(m);
    }

    waypoints_with_status_pub_.publish(Markerarr);
  }

  cv::Point convertPoseToPix(const geometry_msgs::PoseStamped& pose)
  {
    float xPix = (pose.pose.position.x - map_origin_position_x) / mapResolution_;
    float yPix = (pose.pose.position.y - map_origin_position_y) / mapResolution_;

    cv::Point p = cv::Point(xPix, yPix);

    return p;
  }

  geometry_msgs::PoseStamped convertPixToPose(const cv::Point& pixel, geometry_msgs::Quaternion q)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = globalFrame_;

    pose.pose.position.x = (pixel.x * mapResolution_) + map_origin_position_x;
    pose.pose.position.y = (pixel.y * mapResolution_) + map_origin_position_y;
    pose.pose.position.z = 0.0;

    if (q.w != 0.0)
    {
      pose.pose.orientation.w = q.w;
      pose.pose.orientation.x = q.x;
      pose.pose.orientation.y = q.y;
      pose.pose.orientation.z = q.z;
    }
    else
    {
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
    }

    return pose;
  }

  cv::Mat getCurrentMap()
  {
    Mat tmp = currentGlobalMap_.clone();
    return tmp;
  }

  void setStartLocation()
  {
    startCoveragePoint_ = convertPoseToPix(robotPose_);
  }

  int getDistanceBetweenGoalsPix()
  {
    float pixDist = (1.0 / mapResolution_) * distBetweenGoalsM_;

    return (int)pixDist;
  }

  vector<geometry_msgs::PoseStamped> covertPointsPathToPoseRout(const vector<cv::Point>& pointsPath)
  {
    vector<geometry_msgs::PoseStamped> posesPath;

    float prevAngle = 0.0;
    for (int i = 0; i < pointsPath.size(); i++)
    {
      float angle = atan2(pointsPath[i + 1].y - pointsPath[i].y, pointsPath[i + 1].x - pointsPath[i].x);
      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, angle);
      geometry_msgs::Quaternion q;
      q.w = orientation.getW();
      q.x = orientation.getX();
      q.y = orientation.getY();
      q.z = orientation.getZ();

      geometry_msgs::PoseStamped pose;
      pose = convertPixToPose(pointsPath[i], q);
      pose.header.frame_id = globalFrame_;
      pose.header.stamp = ros::Time::now();

      /*
       * if this is the first waypoint or the last we added it,
       * else we add only the uniqu direction of waypoints
       */
      if (i == 0 || i == (pointsPath.size() - 1))
      {
        prevAngle = angle;
        posesPath.push_back(pose);
      }
      else
      {
        if (reducing_goals_)
        {
          if (angle != prevAngle)
          {
            prevAngle = angle;
            posesPath.push_back(pose);
          }
        }
        else
        {
          posesPath.push_back(pose);
        }
      }
    }

    return posesPath;
  }

  void publishCoveragePath(const vector<geometry_msgs::PoseStamped>& coveragePathPoses)
  {
    nav_msgs::Path msgMsg;
    msgMsg.header.frame_id = globalFrame_;
    msgMsg.header.stamp = ros::Time::now();
    msgMsg.poses = coveragePathPoses;

    cuurentCoveragePathPub_.publish(msgMsg);

    ros::Duration(1).sleep();
  }

  cv::Point fixLocationOnGrid(const cv::Point& goal, const cv::Point& ref)
  {
    int detlaY = abs(goal.y + ref.y) % getDistanceBetweenGoalsPix();
    int detlaX = abs(goal.x + ref.x) % getDistanceBetweenGoalsPix();

    cv::Point p(goal.x - detlaX, goal.y - detlaY);
    int val = currentAlgoMap_.at<uchar>(p.y, p.x);
    if (val == free_space)
    {
      detlaY = abs(p.y + ref.y) % getDistanceBetweenGoalsPix();
      detlaX = abs(p.x + ref.x) % getDistanceBetweenGoalsPix();

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
      cv::Point topLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)),
                        p.y - ((getDistanceBetweenGoalsPix() * scalar)));
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
      if (downRight.y > 0 && downRight.x > 0 && downRight.y < currentAlgoMap_.rows &&
          downRight.x < currentAlgoMap_.cols)
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

  cv::Point fixStartLocation(cv::Point p)
  {
    int scalar = 1;

    cv::Point pError(-1, -1);
    for (int i = 0; i < 20; i++)
    {
      cv::Point left(p.x - (getDistanceBetweenGoalsPix() * scalar), p.y);
      cv::Point right(p.x + (getDistanceBetweenGoalsPix() * scalar), p.y);
      cv::Point up(p.x, p.y - (getDistanceBetweenGoalsPix() * scalar));
      cv::Point down(p.x, p.y + (getDistanceBetweenGoalsPix() * scalar));
      cv::Point downLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)),
                         p.y + ((getDistanceBetweenGoalsPix() * scalar)));
      cv::Point downRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)),
                          p.y + ((getDistanceBetweenGoalsPix() * scalar)));
      cv::Point topLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)),
                        p.y - ((getDistanceBetweenGoalsPix() * scalar)));
      cv::Point topRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)),
                         p.y - ((getDistanceBetweenGoalsPix() * scalar)));

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
      if (downRight.y > 0 && downRight.x > 0 && downRight.y < currentAlgoMap_.rows &&
          downRight.x < currentAlgoMap_.cols)
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

    return pError;
  }

  void removeGoalsByRobotRout()
  {
    for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
    {
      /// if the goal already covered
      if (grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
      {
        continue;
      }

      float distRobotFromGoal =
          goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                           cv::Point2d(grid_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                                                       grid_poses_with_status_.coveragePathPoses_[i].pose.position.y));

      if (distRobotFromGoal < sanitization_radius_)
      {
        grid_poses_with_status_.setStatByIndex(i, COVERED_BY_ROBOT_PATH);
      }
    }
  }

  string getMoveBaseState(actionlib::SimpleClientGoalState state)
  {
    switch (state.state_)
    {
      case actionlib::SimpleClientGoalState::ACTIVE: {
        return "ACTIVE";
      }
      case actionlib::SimpleClientGoalState::PENDING: {
        return "PENDING";
      }
      case actionlib::SimpleClientGoalState::RECALLED: {
        return "RECALLED";
      }
      case actionlib::SimpleClientGoalState::REJECTED: {
        return "REJECTED";
      }
      case actionlib::SimpleClientGoalState::PREEMPTED: {
        return "PREEMPTED";
      }
      case actionlib::SimpleClientGoalState::ABORTED: {
        return "ABORTED";
      }
      case actionlib::SimpleClientGoalState::SUCCEEDED: {
        return "SUCCEEDED";
      }
      case actionlib::SimpleClientGoalState::LOST: {
        return "LOST";
      }
    }

    return "";
  }

  bool sendGoal(const geometry_msgs::PoseStamped& goalMsg, int goalIndex = -1, bool in_explore = false,
                bool in_initialization = false, bool in_reverse = false)
  {
    // navigate to the point
    moveBaseController_.navigate(goalMsg);

    bool result = true;

    while (ros::ok())
    {
      ros::spinOnce();

      if (exit_)
      {
        setState("USER_CTRL_C");
        return false;
      }

      if (exitPerson_)
      {
        turnOffLamp();
        setState("PERSON_DETECTED");
        return false;
      }

      updateRobotLocation();

      removeGoalsByRobotRout();

      publishRobotHistoryPath();

      publishWaypointsWithStatus();

      if (show_live_video_)
      {
        showLiveVideo(goalMsg, in_explore);
      }

      // if we already very close to the goal
      if (goalIndex != -1)
      {
        if (grid_poses_with_status_.status_[goalIndex] != UN_COVERED)
        {
          moveBaseController_.moveBaseClient_.cancelGoal();
          return true;
        }
      }

      if (in_explore)
      {
        float distDromRobot =
            goalCalculator.distanceCalculate(cv::Point2d(goalMsg.pose.position.x, goalMsg.pose.position.y),
                                             cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

        if (distDromRobot < 0.35)
        {
          cerr << " cancel the goal !! " << endl;
          moveBaseController_.moveBaseClient_.cancelGoal();
          return true;
        }
      }

      if (in_initialization)
      {
        float distDromRobot =
            goalCalculator.distanceCalculate(cv::Point2d(goalMsg.pose.position.x, goalMsg.pose.position.y),
                                             cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

        if (distDromRobot < 0.1)
        {
          cerr << "in_initialization  cancel the goal !! " << endl;
          moveBaseController_.moveBaseClient_.cancelGoal();
          return true;
        }
      }

      if (in_reverse)
      {
        float distDromRobot =
            goalCalculator.distanceCalculate(cv::Point2d(goalMsg.pose.position.x, goalMsg.pose.position.y),
                                             cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

        if (distDromRobot < 0.1)
        {
          cerr << "in_reverse  cancel the goal !! " << endl;
          moveBaseController_.moveBaseClient_.cancelGoal();
          return true;
        }
      }

      moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
      auto move_base_state = moveBaseController_.moveBaseClient_.getState();

      string strState = getMoveBaseState(move_base_state);

      // cerr << "strState:  " << strState << endl;

      if (move_base_state == actionlib::SimpleClientGoalState::ACTIVE ||
          move_base_state == actionlib::SimpleClientGoalState::PENDING)
      {
        continue;
      }

      if (move_base_state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        result = true;
        break;
      }
      else
      {
        cerr << "faield - > strState:  " << strState << endl;

        result = false;
        break;
      }

      ros::spinOnce();
    }

    return result;
  }

  void showLiveVideo(const geometry_msgs::PoseStamped& goalMsg, bool in_explore = false)
  {
    // try {

    //   if ( (state_ ==  "INITIALIZATION" || state_ == "IDLE") ) {

    //     return;
    //   }

    //   if (mappingMap_.data && initSlamMap_)
    //   {
    //     // Mat patternImg = mappingMap_.clone();
    //     Mat robotTreaceImg = mappingMap_.clone();

    //     // cvtColor(patternImg, patternImg, COLOR_GRAY2BGR);
    //     cvtColor(robotTreaceImg, robotTreaceImg, COLOR_GRAY2BGR);

    //     // draw the grid
    //     for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
    //     {
    //       if (grid_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
    //       {
    //         circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 0,
    //         255),
    //               -1, 8, 0);
    //       }
    //       else if (grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH ||
    //               grid_poses_with_status_.status_[i] == REJECTED)
    //       {
    //         circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(255, 0,
    //         0),
    //               -1, 8, 0);
    //       }
    //       else if (grid_poses_with_status_.status_[i] == UN_COVERED)
    //       {
    //         circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 255,
    //         0),
    //               -1, 8, 0);
    //       }
    //     }

    //     // draw the trace only
    //     for (int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++)
    //     {
    //       cv::Point p1 = convertPoseToPix(robotHistoryPathMsg_.poses[i]);
    //       cv::Point p2 = convertPoseToPix(robotHistoryPathMsg_.poses[i + 1]);

    //       if( in_explore){
    //                   cv::line(robotTreaceImg, p1, p2, Scalar(0, 0, 0), 1);
    //       }
    //       else
    //       {
    //         cv::line(robotTreaceImg, p1, p2, Scalar(0, 255, 255), 1);

    //       }
    //     }

    //     //draw robot pose
    //     circle(robotTreaceImg,
    //      convertPoseToPix(robotHistoryPathMsg_.poses[robotHistoryPathMsg_.poses.size()-1]),
    //       3, Scalar(0, 0, 0),
    //               -1, 8, 0);

    //     // draw next goal
    //     circle(robotTreaceImg, convertPoseToPix(goalMsg), 3, Scalar(255, 0, 255),
    //               -1, 8, 0);

    //     // imshow("robotTreaceImg",robotTreaceImg);
    //     // waitKey(1);

    //     sensor_msgs::ImagePtr msg =
    //     cv_bridge::CvImage(std_msgs::Header(), "bgr8", robotTreaceImg).toImageMsg();

    //     liveMapPub_.publish(msg);
    //   }

    // }
    //  catch (cv::Exception& e)
    // {
    //   const char* err_msg = e.what();
    //   std::cerr << "exception caught: " << err_msg << std::endl;
    // }
  }
  void saveCoverageImg()
  {
    if (!imgSaved_ && mappingMap_.data)
    { 

      logManager_.writeToLog("saveCoverageImg");

      Mat robotTreaceImg = mappingMap_.clone();

      cvtColor(robotTreaceImg, robotTreaceImg, COLOR_GRAY2BGR);

      // put back the black and the gray color to the map
      for (int j = 0; j < mappingMap_.rows; j++)
      {
        for (int i = 0; i < mappingMap_.cols; i++)
        {
          int value = mappingMap_.at<uchar>(j, i);

          if (value == 0)
          {
            robotTreaceImg.at<cv::Vec3b>(j, i)[0] = 0;
            robotTreaceImg.at<cv::Vec3b>(j, i)[1] = 0;
            robotTreaceImg.at<cv::Vec3b>(j, i)[2] = 0;
          }
          else if (value == 205)
          {
            robotTreaceImg.at<cv::Vec3b>(j, i)[0] = 205;
            robotTreaceImg.at<cv::Vec3b>(j, i)[1] = 205;
            robotTreaceImg.at<cv::Vec3b>(j, i)[2] = 205;
          }
        }
      }

      // draw the grid
      for (int i = 0; i < grid_poses_with_status_.coveragePathPoses_.size(); i++)
      {
        if (grid_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
        {
          int r, g, b;
          sscanf(COVERED_BY_OBSTACLE_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
          circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1,
                 Scalar(b, g, r) /*Scalar(0, 0, 255)*/, -1, 8, 0);
        }
        else if (grid_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
        {
          int r, g, b;
          sscanf(COVERED_BY_ROBOT_PATH_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
          circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1,
                 Scalar(b, g, r) /*Scalar(255, 0, 0)*/, -1, 8, 0);
        }
        else if (grid_poses_with_status_.status_[i] == UN_COVERED)
        {
          int r, g, b;
          sscanf(UN_COVERED_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
          circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1,
                 Scalar(b, g, r) /*Scalar(0, 255, 0)*/, -1, 8, 0);
        }
        else if (grid_poses_with_status_.status_[i] == REJECTED)
        {
          int r, g, b;
          sscanf(REJECTED_COLOR.c_str(), "%02x%02x%02x", &r, &g, &b);
          circle(robotTreaceImg, convertPoseToPix(grid_poses_with_status_.coveragePathPoses_[i]), 1,
                 Scalar(b, g, r) /*Scalar(0, 165, 255)*/, -1, 8, 0);
        }
      }
      std::cerr << "111111" << std::endl;
      // draw the trace only
      if (!(state_ == "INITIALIZATION" || state_ == "IDLE"))
      {
      for (int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++)
      {
        cv::Point p1 = convertPoseToPix(robotHistoryPathMsg_.poses[i]);
        cv::Point p2 = convertPoseToPix(robotHistoryPathMsg_.poses[i + 1]);

        cv::line(robotTreaceImg, p1, p2, Scalar(0, 255, 255), 1);
      }
      }
      std::cerr << "222222" << std::endl;
      auto end = high_resolution_clock::now();
      auto durationCoverage = duration_cast<seconds>(end - startingCoverageTime_);

      int durationMinutes = durationCoverage.count() / 60;

      percentCoverage_ = getCurrentPercentCoverage();
      string image_name_format =
          startingTime_ + '_' + to_string(durationMinutes) + '_' + to_string(int(percentCoverage_));
      string full_img_name = coverage_img_path_ + image_name_format + ".png";
      node_.setParam("/coverage/image_name", image_name_format);

      cerr << "full_img_name: " << full_img_name << endl;

      cv::imwrite(full_img_name, robotTreaceImg);
      // save to nimbus-cloud
      cv::imwrite("/var/lib/nimbus/records/" + image_name_format + ".png", robotTreaceImg);

      imgSaved_ = true;
    }
  }

  void addDilationByGlobalCostMap(const Mat& globalCostMap, Mat& algoMap, const cv::Point2d& robotBaseFootPrint)
  {
    if (!globalCostMap.data || !algoMap.data || robotBaseFootPrint.x < 0 || robotBaseFootPrint.y < 0 ||
        robotBaseFootPrint.x > algoMap.cols || robotBaseFootPrint.y > algoMap.rows)
    {
      cerr << " failed to addDilationByGlobalCostMap " << endl;
      return;
    }
    cv::Point2d mapCenter(algoMap.cols / 2, algoMap.rows / 2);
    cv::Point2d costMapCenter(globalCostMap.cols / 2, globalCostMap.rows / 2);

    // cv::Point robotBaseFootPrint;

    // robotBaseFootPrint.x = (1.1 + 10) /  0.05;
    // robotBaseFootPrint.y = (-1.43  + 10) / 0.05;
    // cerr<<" robotBaseFootPrint x "<<robotBaseFootPrint.x<<" robotBaseFootPrint y "<<robotBaseFootPrint.y<<endl;

    int deltaRobotX = mapCenter.x - robotBaseFootPrint.x;
    int deltaRobotY = mapCenter.y - robotBaseFootPrint.y;

    try
    {
      for (int y = 0; y < globalCostMap.rows; y++)
      {
        for (int x = 0; x < globalCostMap.cols; x++)
        {
          int value = globalCostMap.at<uchar>(y, x);

          if (value > 0)
          {
            cv::Point2d nP(x + (mapCenter.x - costMapCenter.x) - deltaRobotX,
                           y + (mapCenter.y - costMapCenter.y) - deltaRobotY);

            //  cv::Point2d nP(x + (mapCenter.x - costMapCenter.x),
            //     y + (mapCenter.y - costMapCenter.y)  );

            if (nP.x > 0 && nP.y > 0 && nP.x < algoMap.cols && nP.y < algoMap.rows)
            {
              algoMap.at<uchar>(nP.y, nP.x) = 0;
            }
          }
        }
      }
    }
    catch (cv::Exception& e)
    {
      const char* err_msg = e.what();
      std::cerr << "exception caught: " << err_msg << std::endl;
    }
  }

  void setReverse()
  {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter float_param;
    dynamic_reconfigure::Config conf;

    float_param.name = "min_vel_x";
    float_param.value = -0.1;
    conf.doubles.push_back(float_param);

    srv_req.config = conf;
    if (ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp))
    {
      cerr << " after set reverse " << endl;
    }
    else
    {
      cerr << "errrrrrrrrrrrrrrrrrrrrr " << endl;
    }

    ros::Duration(1).sleep();

    reversAllowed_ = true;
  }

  void disableReverse()
  {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter float_param;
    dynamic_reconfigure::Config conf;

    float_param.name = "min_vel_x";
    float_param.value = 0.0;
    conf.doubles.push_back(float_param);

    srv_req.config = conf;
    if (ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp))
    {
      cerr << " after disable reverse " << endl;
    }
    else
    {
      cerr << "errrrrrrrrrrrrrrrrrrrrr " << endl;
    }

    ros::Duration(1).sleep();

    reversAllowed_ = false;
  }

  bool rotateInPlace(int numOfRounds = 2)
  {
    geometry_msgs::Twist rotationCmd;
    rotationCmd.linear.x = 0;
    rotationCmd.linear.y = 0;
    rotationCmd.linear.z = 0;
    rotationCmd.angular.x = 0;
    rotationCmd.angular.y = 0;
    // clockwise
    rotationCmd.angular.z = -abs(0.2);

    float sumOfAngles = 0.0;

    int iteration = 0;

    float prevAngle = 0.0;

    cerr << " start to rotate " << numOfRounds << " rounds " << endl;

    logManager_.writeToLog("start to rotate  1 round");


    while (ros::ok())
    {
      updateRobotLocation();

      if (exit_)
      {
        setState("USER_CTRL_C");
        logManager_.writeToLog("USER_CTRL_C");

        return false;
      }

      if (exitPerson_)
      {
        turnOffLamp();
        setState("PERSON_DETECTED");
        logManager_.writeToLog("rotateInPlace func: PERSON_DETECTED");

        return false;
      }

      float currDeg = angles::to_degrees(
          atan2((2.0 * (currOdom_.pose.pose.orientation.w * currOdom_.pose.pose.orientation.z +
                        currOdom_.pose.pose.orientation.x * currOdom_.pose.pose.orientation.y)),
                (1.0 - 2.0 * (currOdom_.pose.pose.orientation.y * currOdom_.pose.pose.orientation.y +
                              currOdom_.pose.pose.orientation.z * currOdom_.pose.pose.orientation.z))));

      // init
      if (iteration == 0)
      {
        prevAngle = currDeg;
        iteration++;

        continue;
      }

      if (!(prevAngle < 0 && currDeg > 0))
      {
        sumOfAngles = sumOfAngles + (prevAngle - currDeg);

        prevAngle = currDeg;
      }
      else
      {
        sumOfAngles = sumOfAngles + (180 + (180 + prevAngle)) - currDeg;

        prevAngle = currDeg;
      }

      // cerr<<"sumOfAngles "<<sumOfAngles<<endl;

      int num_of_rounds = (int(sumOfAngles) / 360);

      if (num_of_rounds >= numOfRounds)
      {
        geometry_msgs::Twist rotationCmd;
        rotationCmd.linear.x = 0;
        rotationCmd.linear.y = 0;
        rotationCmd.linear.z = 0;
        rotationCmd.angular.x = 0;
        rotationCmd.angular.y = 0;
        rotationCmd.angular.z = 0;

        reverse_cmd_vel_pub_.publish(rotationCmd);
        ros::Duration(0.1).sleep();

        return true;
      }

      if (detectedPerson_)
      {
        geometry_msgs::Twist rotationCmd;
        rotationCmd.linear.x = 0;
        rotationCmd.linear.y = 0;
        rotationCmd.linear.z = 0;
        rotationCmd.angular.x = 0;
        rotationCmd.angular.y = 0;
        rotationCmd.angular.z = 0;

        reverse_cmd_vel_pub_.publish(rotationCmd);
        ros::Duration(0.1).sleep();

        return false;
      }

      iteration++;

      reverse_cmd_vel_pub_.publish(rotationCmd);

      ros::Duration(0.1).sleep();

      ros::spinOnce();
    }

    return true;
  }

  void turnOnLamp()
  {
    std_msgs::Bool msg;
    msg.data = true;
    uv_lamp_set_state_pub_.publish(msg);
  }

public:
  bool initializationGood_ = false;
  string state_ = "INITIALIZING";

private:
  COVERAGE_STATE coverage_state_ = COVERAGE_STATE::COVERAGE_BY_STRAIGHT_LINES;

  EXPLORE_STATE explore_state_ = EXPLORE_STATE::IDLE;

  // move-base
  MoveBaseController moveBaseController_;

  // subs
  ros::Subscriber global_map_sub_;

  ros::Subscriber global_cost_map_sub_;

  ros::Subscriber camera_scan_sub_;

  ros::Subscriber odom_sub_;

  ros::Subscriber is_person_detected_sub_;

  // timer

  ros::Timer lampTimer_;

  // pubs

  ros::Publisher reverse_cmd_vel_pub_;

  ros::Publisher cuurentCoveragePathPub_;

  ros::Publisher edges_frontires_marker_array_Pub;

  ros::Publisher waypoints_with_status_pub_;

  ros::Publisher directions_marker_array_pub_;

  ros::Publisher uv_lamp_set_state_pub_;

  image_transport::Publisher coverage_map_pub_;

  ros::Publisher sanitization_radius_marker_pub_;

  ros::Publisher robot_history_path_pub_;
  nav_msgs::Path robotHistoryPathMsg_;

  ros::Publisher backward_goal_marker_pub_;

  image_transport::Publisher liveMapPub_;

  // classes

  DistanceTransformGoalCalculator distanceTransformGoalCalculator;
  DisantanceMapCoverage disantanceMapCoverage;
  GoalCalculator goalCalculator;

  // ALGO-PARAMS

  std::vector<Frontier> currentBlobsFrontiers_;

  std::vector<Frontier> currentEdgesFrontiers_;

  cv::Point startCoveragePoint_;

  cv::Point globalStart_;

  cv::Mat currentAlgoMap_;

  Path_with_Status grid_poses_with_status_;

  ros::NodeHandle node_;

  // params

  geometry_msgs::PoseStamped robotPose_;

  nav_msgs::Odometry currOdom_;

  geometry_msgs::PoseStamped robotStartLocation_;

  vector<cv::Point2d> currentCameraScanMapPointsM_;

  cv::Mat currentGlobalMap_;

  Mat mappingMap_;

  float map_origin_position_x = -1;

  float map_origin_position_y = -1;

  float globalMapWidth_;

  float globalMapHeight_;

  float mapResolution_ = 0.05;

  // global cost map params
  float globalMapOriginPositionX_ = -1;
  float globalMapOriginPositionY_ = -1;
  float globalMapResolution_ = -1;
  cv::Mat costMapImg_;

  bool initSlamMap_ = false;

  // reverse logic
  bool reversAllowed_ = false;
  bool smallReverseAllowed_ = false;

  bool initGlobalCostMap_ = false;

  float width_;

  float height_;

  double distBetweenGoalsM_ = 0.5;

  tf::TransformListener tfListener_;

  string globalFrame_ = "map";

  string baseFrame_ = "base_footprint";

  string odomFrame_ = "odom";

  bool reducing_goals_ = true;

  bool show_live_video_ = false;

  double robot_w_m_ = 0.53;
  double robot_h_m_ = 0.53;

  double path_deg_angle_threshold_ = 100.0;

  double walls_inflation_m_ = 0.3;

  double sanitization_radius_ = 1.0;

  double radius_for_cleaning_route_goals_ = 0.2;

  double duration_wait_for_move_base_response_ = 15.0;

  double percentCoverage_ = 0.0;

  string coverage_img_path_ = "";

  string image_name_ = "";

 // string state_ = "INITIALIZING";

  bool detectedPerson_ = false;

  bool imgSaved_ = false;

  // robot footprint
  bool gotFootPrint_ = false;
  geometry_msgs::PolygonStamped currentRobotFootPrintLocationOodm_;

  bool errCoverage_ = false;

  high_resolution_clock::time_point startingCoverageTime_;

  high_resolution_clock::time_point startLocalCostMap_;

  std::map<std::string, int> camera_laser_beams_;

  vector<geometry_msgs::PointStamped> cameraScanObstacles;

  // log-manager
  LogManager logManager_;

  string COVERED_BY_OBSTACLE_COLOR;
  string COVERED_BY_ROBOT_PATH_COLOR;
  string UN_COVERED_COLOR;
  string REJECTED_COLOR;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_coverage_exploration_node", ros::init_options::NoSigintHandler);

  MapCoverageManager mapCoverageManager;

  signal(SIGINT, (void (*)(int))MapCoverageManager::mySigintHandler);

  if (mapCoverageManager.initialization())
  {
    if (mapCoverageManager.explore())
    {
      mapCoverageManager.coverage();
    }
  }
  else
  {
	  if (mapCoverageManager.state_ != "PERSON_DETECTED" && mapCoverageManager.state_ != "USER_CTRL_C"){

    mapCoverageManager.turnOffLamp();
    mapCoverageManager.setState("INITIALIZATION_ERROR");
    cerr << " initialization failed  " << endl;
	  }
  }

  //ros::spin();

  return 0;
}
