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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Empty.h>
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

using namespace cv;
using namespace std::chrono;
using namespace std;

#include "../include/DisantanceMapCoverage.h"
#include "../include/GoalCalculator.h"
#include "../include/MoveBaseController.h"

geometry_msgs::PoseStamped startingLocation_;
string startingTime_;

bool exit_ = false;

enum GoalState
{
    UN_COVERED = 0,
    COVERED = 1,
    COVERED_BY_ROBOT_PATH = 2,
    COVERED_BY_OBSTACLE = 3
};

struct Path_with_Status
{
    vector<geometry_msgs::PoseStamped> coveragePathPoses_;

    vector<GoalState> status_;

    vector<cv::Point> path_; // for display on image

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

    void setPixelsPath(const vector<cv::Point> &path)
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
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%F/%H_%M", timeinfo);
    string curr_time_Str = strdup(buffer);
    std::replace(curr_time_Str.begin(), curr_time_Str.end(), '/', '_');
    std::replace(curr_time_Str.begin(), curr_time_Str.end(), '-', '_');

    return curr_time_Str;
}

void addDilationForGlobalMap(Mat &imgMap, float walls_inflation_m, float mapResolution)
{

    try
    {
        int dilationPix = (1.0 / mapResolution) * (walls_inflation_m);

        cv::Mat binary = imgMap.clone();
        binary.setTo(0, imgMap != 0);
        binary.setTo(255, imgMap == 0);
        dilate(binary, binary, Mat(), Point(-1, -1), dilationPix, 1, 1);

        imgMap.setTo(0, binary == 255);
    }
    catch (cv::Exception &e)
    {
        const char *err_msg = e.what();
        std::cerr << "exception caught: " << err_msg << std::endl;
    }
}

void addFreeSpaceDilation(Mat &grayscaleImg)
{

    Mat binary = cv::Mat(grayscaleImg.rows, grayscaleImg.cols,
                         CV_8UC1, cv::Scalar(0));

    binary.setTo(255, grayscaleImg >= 254);
    dilate(binary, binary, Mat(), Point(-1, -1), 1, 1, 1);

    Mat newImg = cv::Mat(grayscaleImg.rows, grayscaleImg.cols,
                         CV_8UC1, cv::Scalar(205));

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
        }

        // rosparam
        ros::NodeHandle nodePrivate("~");
        nodePrivate.param("distance_between_goals_m", distBetweenGoalsM_, 0.5);
        nodePrivate.param("walls_inflation_m", walls_inflation_m_, 0.3);
        nodePrivate.param("robot_w_m", robot_w_m_, 0.53);
        nodePrivate.param("robot_h_m", robot_h_m_, 0.53);
        nodePrivate.param("sanitization_radius", sanitization_radius_, 1.0);

        

        nodePrivate.param("duration_wait_for_move_base_response", duration_wait_for_move_base_response_, 15.0);
        nodePrivate.param<string>("coverage_image_path", coverage_img_path_, string(""));
        nodePrivate.param<string>("base_frame", baseFrame_, string("base_link"));
        nodePrivate.param<string>("global_frame", globalFrame_, string("map"));

        nodePrivate.param("reducing_goals", reducing_goals_, true);

        nodePrivate.param("/coverage/percentage", percentCoverage_, 0.0);
        nodePrivate.param("/coverage/state", state_, string("INITIALIZING"));
        nodePrivate.param("/coverage/image_name", image_name_, string(""));

        // subs
        global_map_sub_ =
            node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                     &MapCoverageManager::globalMapCallback, this);

        // subs
        global_cost_map_sub_ =
            node_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1,
                                                     &MapCoverageManager::globalCostMapCallback, this);

        // pubs

        reverse_cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>(
            "/cmd_vel", 1, false);

        cuurentCoveragePathPub_ = node_.advertise<nav_msgs::Path>(
            "/coverage_path", 1, false);

        robot_history_path_pub_ = node_.advertise<nav_msgs::Path>(
            "/robot_history_path", 1, false);

        waypoints_with_status_pub_ =
            node_.advertise<visualization_msgs::MarkerArray>("/waypoints_with_status_marker_arr", 10);

        edges_frontires_marker_array_Pub =
            node_.advertise<visualization_msgs::MarkerArray>("/edges_frontiers_marker_arr", 10);

        directions_marker_array_pub_ =
            node_.advertise<visualization_msgs::MarkerArray>("/directions_marker_arr", 10);

        sanitization_radius_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/sanitization_radius", 10);



        init_ = false;

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

        cerr << "MapCoverageManager distructor " << endl;
        saveCoverageImg();

        ros::shutdown();
    }

    bool exlpore()
    {
        string m = "";

        vector<cv::Point2d> unreachedPointFromFronitiers;

        while (ros::ok())
        {
            ros::spinOnce();

            if (exit_)
            {

                return false;
            }

            if (!init_ || !initGlobalCostMap_)
            {
                cerr << "map not recieved !!" << endl;

                continue;
                
            }

            if (!updateRobotLocation())
            {
                cerr << "can't update robot location !!" << endl;

                continue;
            }

            node_.setParam("/coverage/state", "RUNNING");

            currentAlgoMap_ = getCurrentMap();

            clearAllCostMaps();


            Mat explorationImgMap = currentAlgoMap_.clone();
            addDilationByGlobalCostMap(costMapImg_, explorationImgMap,
                        convertPoseToPix(robotPose_));
            // imwrite("/home/yakir/distance_transform_coverage_ws/costMapImg_.png", costMapImg_);
            // imwrite("/home/yakir/distance_transform_coverage_ws/explorationImgMap.png", explorationImgMap);

            // addDilationForGlobalMap(explorationImgMap, 0.1, mapResolution_);
            // addFreeSpaceDilation(explorationImgMap);

            // set the unreached goals on the exploration map

            for (int i = 0; i < unreachedPointFromFronitiers.size(); i++)
            {

                int radiusPix = (1.0 / mapResolution_) * (walls_inflation_m_);
                circle(explorationImgMap, unreachedPointFromFronitiers[i], radiusPix, Scalar(0));
            }

            switch (explore_state_)
            {

            case IDLE:
            {
                cerr << "IDLE " << endl;

                startingTime_ = getCurrentTime();

                startingLocation_ = robotPose_;

                explore_state_ = NAV_TO_NEXT_FRONTIER;


                break;
            }
            case NAV_TO_NEXT_FRONTIER:
            {

                cerr << "NAV_TO_NEXT_FRONTIER " << endl;

                float mapScore = 0.0;

                updateRobotLocation();

                auto robotPix = convertPoseToPix(robotPose_);

                std::vector<Frontier> currentEdgesFrontiers;
                mapScore = goalCalculator.calcEdgesFrontiers(explorationImgMap,
                                                             currentEdgesFrontiers, robotPix, mapResolution_);

                cerr << "map exploration score: " << mapScore << endl;

                if (currentEdgesFrontiers.size() == 0)
                {

                    explore_state_ = FINISH_EXPLORE;
                    break;
                }

                geometry_msgs::Quaternion q;
                q.w = 1;
                auto nextFrontierGoal = convertPixToPose(currentEdgesFrontiers[0].center, q);

                publishEdgesFrontiers(currentEdgesFrontiers);

                makeReverseIfNeeded();

                cerr << " sending goal to the next froniter pix " << currentEdgesFrontiers[0].center << endl;


                bool result = sendGoal(nextFrontierGoal, -1, true);

                unreachedPointFromFronitiers.push_back(currentEdgesFrontiers[0].center);


                cerr << "move_base result for NAV_TO_NEXT_FRONTIER " << result << endl;

                explore_state_ = NAV_TO_NEXT_FRONTIER;

                break;
            }
            case ERROR_EXPLORE:
            {
                node_.setParam("/coverage/state", "STOPPED");

                cerr << "ERROR_EXPLORE " << endl;
                break;
            }
            case FINISH_EXPLORE:
            {
                cerr << "FINISH_EXPLORE " << endl;

                return true;
            }
            }
        }

        return false;
    }

    void coverage()
    {

        int countNumberOfTriesFinsihGoal = 0;

        while (ros::ok())
        {
            ros::spinOnce();

            if (!init_)
            {

                continue;
            }

            if (!updateRobotLocation())
            {

                continue;
            }

            switch (coverage_state_)
            {
            case COVERAGE_BY_STRAIGHT_LINES:
            {
                cerr << " COVERAGE_BY_STRAIGHT_LINES " << endl;

                currentAlgoMap_ = getCurrentMap();
                addDilationByGlobalCostMap(costMapImg_, currentAlgoMap_,
                        convertPoseToPix(robotPose_));

                // calculate goal-distance-transform-map
                cv::Mat distanceTransformImg;

                updateRobotLocation();

                cv::Point2d currentPosition = convertPoseToPix(startingLocation_);

                cerr << " currentPosition " << currentPosition << endl;

                cv::Point2d goal = currentPosition;

                // // calc the distance-transform-img from current goal
                if (!distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_,
                            currentPosition, distanceTransformImg, 1))
                {

                    cerr << " failed to calcutate the distance transform img" << endl;
                    coverage_state_ = ERROR_COVERAGE;
                    break;
                }

                // Mat dbg = mapping_map_.clone();
                // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

                // calc the path-coverage of the current blob

                auto path =
                    disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition,
                                                          goal, distanceTransformImg, getDistanceBetweenGoalsPix());

                // convert the path into poses
                path_poses_with_status_.setPixelsPath(path);
                path_poses_with_status_.coveragePathPoses_ = covertPointsPathToPoseRout(path);
                path_poses_with_status_.initStatusList();

                // exectute currnet navigation the blob-coverage
                cerr << "num of coverage waypoints " << path_poses_with_status_.coveragePathPoses_.size() << " path_ size is " << path.size() << endl;

                bool coverage_done = false;
                int iteration = 0;
                float coveragePer = 0.0;


                vector<cv::Point2d> markedGoalsOnMap;

                markedWayPointsByExploration();


                while (!coverage_done && ros::ok())
                {   
                    ros::spinOnce();

                    if (exit_)
                    {

                        saveCoverageImg();
                        return;
                    }

                    currentAlgoMap_ = getCurrentMap();

                    updateRobotLocation();


                    // marked checked goals
                    for(int i = 0; i < markedGoalsOnMap.size(); i++ ){
                        currentAlgoMap_.at<uchar>(markedGoalsOnMap[i].y, markedGoalsOnMap[i].x) = 0;
                    }

                    auto robotPix = convertPoseToPix(robotPose_);
                    
                    float robotHeading = atan2((2.0 *
                        (robotPose_.pose.orientation.w * robotPose_.pose.orientation.z +
                            robotPose_.pose.orientation.x * robotPose_.pose.orientation.y)),
                        (1.0 - 2.0 * (robotPose_.pose.orientation.y * robotPose_.pose.orientation.y +
                                        robotPose_.pose.orientation.z * robotPose_.pose.orientation.z)));
                   

                    cv::Point2d finalGoalToNavigate;
                    int bestGoalIndexWaypoint;


                    // find next goal by 8 direction method
                    auto foundGoalByDirection = findNextGoalByDirection(finalGoalToNavigate, &bestGoalIndexWaypoint,
                        robotPix, robot_w_m_,
                        mapResolution_, robotHeading,
                        currentAlgoMap_);


                    
                    if( !foundGoalByDirection)  // find next goal by connected-components
                    {
                        cerr<<" trting to finld goal by findNextGoalByConnetctedComponents ... "<<endl;    
                        auto foundGoalByConnectedComponents = 
                            findNextGoalByConnetctedComponents(finalGoalToNavigate, currentAlgoMap_,
                            path_poses_with_status_, distBetweenGoalsM_, mapResolution_);


                        if (foundGoalByConnectedComponents ) {
                            
                            cerr<<" found goal by connected component !!"<<endl;
                            
                            clearAllCostMaps();
                            
                            makeReverseIfNeeded();

                            /// send the goal !!
                            geometry_msgs::Quaternion q;
                            auto goalHeading = -1*  atan2(finalGoalToNavigate.y - robotPix.y,
                                finalGoalToNavigate.x - robotPix.x);
                            tf2::Quaternion orientation;
                            orientation.setRPY(0, 0, -1 * goalHeading);
                            q.w = orientation.getW();
                            q.x = orientation.getX();
                            q.y = orientation.getY();
                            q.z = orientation.getZ();
                            
                            markedGoalsOnMap.push_back(finalGoalToNavigate);
                            
                            auto nextGoal = convertPixToPose(finalGoalToNavigate, q);
                            bool result = sendGoal(nextGoal, -1);      

                            continue;

                        } else {
                            
                            cerr<<" failed to find by connected components "<<endl;
                            coverage_done = true;
                            break;;
                        }
                    }

                    cerr<<" publish goal by direction pix goal : "<<finalGoalToNavigate<<endl;

                    // send the goal by direction only !!! 
                    percentCoverage_ = getCurrentPercentCoverage();
                    node_.setParam("/coverage/percentage", percentCoverage_);                   


                    makeReverseIfNeeded();


                     /// send the goal !!
                    geometry_msgs::Quaternion q;
                    auto goalHeading = -1*  atan2(finalGoalToNavigate.y - robotPix.y,
                        finalGoalToNavigate.x - robotPix.x);
                    tf2::Quaternion orientation;
                    orientation.setRPY(0, 0, -1 * goalHeading);
                    q.w = orientation.getW();
                    q.x = orientation.getX();
                    q.y = orientation.getY();
                    q.z = orientation.getZ();
                    
                    
                    auto nextGoal = convertPixToPose(finalGoalToNavigate, q);
                    bool result = sendGoal(nextGoal, bestGoalIndexWaypoint);               

                    iteration++;

                   
                }

                coverage_state_ = BACK_TO_STARTING_LOCATION;
                break;
            }        
            case COVERAGE:
            {
                cerr << " COVERAGE " << endl;

                currentAlgoMap_ = getCurrentMap();

                // calculate goal-distance-transform-map
                cv::Mat distanceTransformImg;

                updateRobotLocation();

                cv::Point2d currentPosition = convertPoseToPix(startingLocation_);

                cerr << " currentPosition " << currentPosition << endl;

                cv::Point2d goal = currentPosition;

                // // calc the distance-transform-img from current goal
                if (!distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_,
                                                                              currentPosition, distanceTransformImg, 1))
                {

                    cerr << " failed to calcutate the disntace transform img" << endl;
                    coverage_state_ = ERROR_COVERAGE;
                    break;
                }

                Mat grayDistImg;
                distanceTransformGoalCalculator.normalizeDistanceTransform(distanceTransformImg, grayDistImg);

                // Mat dbg = mapping_map_.clone();
                // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

                // calc the path-coverage of the current blob

                auto path =
                    disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition,
                                                          goal, distanceTransformImg, getDistanceBetweenGoalsPix());

                // convert the path into poses
                path_poses_with_status_.setPixelsPath(path);
                path_poses_with_status_.coveragePathPoses_ = covertPointsPathToPoseRout(path);
                path_poses_with_status_.initStatusList();

                // exectute currnet navigation the blob-coverage
                cerr << "cccccccccccc num of coverage waypoints " << path_poses_with_status_.coveragePathPoses_.size() << " path_ size is " << path.size() << endl;

                for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
                {

                    ros::spinOnce();

                    updateRobotLocation();

                    removeGoalsByRobotRout();

                    percentCoverage_ = getCurrentPercentCoverage();
                    node_.setParam("/coverage/percentage", percentCoverage_);

                    publishCoveragePath(path_poses_with_status_.coveragePathPoses_);

                    publishRobotHistoryPath();

                    // the waypoint is checked (black)
                    if (path_poses_with_status_.status_[i] != UN_COVERED)
                    {
                        continue;
                    }

                    if (!makeReverseIfNeeded())
                    {

                        moveFarwardIfRobotStuck();
                    }

                    ros::spinOnce();

                    cerr << " sending goal coverage index " << i << endl;
                    bool result = sendGoal(path_poses_with_status_.coveragePathPoses_[i], i);

                    // set way[oint as checked
                    path_poses_with_status_.setStatByIndex(i, COVERED);

                    if (exit_)
                    {

                        saveCoverageImg();
                        return;
                    }

                    if (result)
                    {
                        cerr << i << ": Waypoint reached!" << endl;
                    }
                    else
                    {
                        cerr << i << ": Failed to reach waypoint" << endl;
                    }
                }

                coverage_state_ = BACK_TO_STARTING_LOCATION;
                break;
            }
            case BACK_TO_STARTING_LOCATION:
            {
                clearAllCostMaps();

                if (countNumberOfTriesFinsihGoal > 3)
                {

                    coverage_state_ = COVERAGE_DONE;

                    break;
                }

                cerr << "startingLocation_ : " << startingLocation_.pose.position.x << ", " << startingLocation_.pose.position.y << endl;
                cerr << "startingLocation_  frame: " << startingLocation_.header.frame_id << endl;

                makeReverseIfNeeded();

                bool result = sendGoal(startingLocation_);

                if (result)
                {
                    cerr << "BACK_TO_STARTING_LOCATION reached!" << endl;

                    coverage_state_ = COVERAGE_DONE;

                    break;
                }
                else
                {
                    cerr << " Failed to reach BACK_TO_STARTING_LOCATION, try again" << endl;

                    coverage_state_ = BACK_TO_STARTING_LOCATION;

                    countNumberOfTriesFinsihGoal++;

                    break;
                }
            }
            case COVERAGE_DONE:
            {
               cerr << " COVERAGE_DONE " << endl;

                node_.setParam("/coverage/state", "STOPPED");

                saveCoverageImg();

                coverage_state_ = COVERAGE_DONE;
                break;
            }

            case ERROR_COVERAGE:
            {

                if (!errCoverage_)
                {
                    cerr << " ERROR_COVERAGE " << endl;
                    errCoverage_ = true;
                }

                node_.setParam("/coverage/state", "STOPPED");

                coverage_state_ = ERROR_COVERAGE;
                break;
            }
            }
        }
    }
    

    void clearAllCostMaps(){ 

        cerr << " clearAllCostMaps " << endl;
                
        ros::NodeHandle n;
        ros::ServiceClient client
                = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        
        std_srvs::Empty srv;               
        if (client.call(srv))
        {   
            cerr<<" clearing costmaps!!!!!!";
            ros::Duration(1).sleep();

        }
        else
        {
            cerr<<"errrror clearing costmaps!!!!!!";
            
        }

    }
    void markedWayPointsByExploration() {

        for(int j = 0; j < robotHistoryPathMsg_.poses.size(); j++){

            for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++){
                
                if ( path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH || 
                    path_poses_with_status_.status_[i] == COVERED  ){
                        continue;
                }

                if (goalCalculator.distanceCalculate(
                    cv::Point2d(robotHistoryPathMsg_.poses[j].pose.position.x,
                    robotHistoryPathMsg_.poses[j].pose.position.y), 
                        cv::Point2d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                            path_poses_with_status_.coveragePathPoses_[i].pose.position.y)) < sanitization_radius_){

                   path_poses_with_status_.status_[i] = COVERED_BY_ROBOT_PATH;             
                }
            }
        }


    }
    void publishSanitizationRadius(){
        
        
        std::srand(std::time(nullptr)); // use current time as seed for random generator

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = baseFrame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 6000;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.1;
        line_strip.color.b = 0.5;
        line_strip.color.g = 1.0;
        line_strip.color.r = 0.2;
        line_strip.color.a = 1.0;

        for (uint32_t i = 0; i < 360; ++i)
        {
            geometry_msgs::Point p;

            p.y = (sanitization_radius_)*sin(angles::from_degrees(i));
            p.x = (sanitization_radius_)*cos(angles::from_degrees(i));
            p.z = 0;

            line_strip.points.push_back(p);
        }


        sanitization_radius_marker_pub_.publish(line_strip);
    }


    float getScoreByDirection(float rotattionAngle,
                            const cv::Point2d &refPoint1, const cv::Point2d &refPoint2,
                            cv::Point2d &bestGoal, int* goalIndex, const cv::Point2d &robotPix, float robotHeading,
                            float robot_w_m, float mapResolution_, const Mat &imgMap,
                            vector<int> &waypointsInPathDebug, string direc,
                            float minAreaM = 0.0,
                            float maxDistanceM = 4.0)
    {

        // cerr << direc << endl;

        cv::Mat dbg = imgMap.clone();
        cvtColor(dbg, dbg, COLOR_GRAY2BGR);

        float robot_W_pix = robot_w_m / mapResolution_;

        float len = robot_W_pix;

        float size = (robot_W_pix / 2);

        int unCovered = 0;

        cv::Point2d middle;

        cv::Point2d robotHeadingPoint(robotPix.x + (10) * cos(robotHeading),
                                    robotPix.y + (10) * sin(robotHeading));
        // cv::arrowedLine(dbg, robotPix, robotHeadingPoint, Scalar(80, 127, 255), 2,
        //                 8, 0, 0.3);

        // for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
        // {

        //     switch (path_poses_with_status_.status_[i])
        //     {
        //         case UN_COVERED:
        //         {  
                    
        //             circle(dbg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]) , 1, Scalar(0, 255, 0), -1, 8, 0);
        //             break;
        //         }
        //         case COVERED:
        //         {
        //             circle(dbg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]) , 1, Scalar(255, 0, 0), -1, 8, 0);
        //             break;
        //         }
        //         case COVERED_BY_ROBOT_PATH:
        //         {
        //             circle(dbg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]) ,1, Scalar(255, 0, 0), -1, 8, 0);
        //             break;
        //         }
        //         case COVERED_BY_OBSTACLE:
        //         {
        //             circle(dbg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 0, 255), -1, 8, 0);
        //             break;
        //         }
        //     }
        // }                

        while (ros::ok())
        {
            cv::Point2d leftSide(refPoint1.x + (size)*cos(rotattionAngle),
                                refPoint1.y + (size)*sin(rotattionAngle));

            cv::Point2d rightSide(refPoint2.x + (size)*cos(rotattionAngle),
                                refPoint2.y + (size)*sin(rotattionAngle));

            if (leftSide.x < 0 || leftSide.x > imgMap.cols ||
                leftSide.y < 0 || leftSide.y > imgMap.rows)
            {

                break;
            }

            // contour represent the area of searching uncovered goals
            vector<cv::Point> contour{refPoint1, refPoint2, rightSide, leftSide};

            cv::polylines(dbg, contour, true, Scalar(255, 0, 0), 1);

            middle = cv::Point2d((leftSide.x + rightSide.x) / 2,
                                (leftSide.y + rightSide.y) / 2);

            float minDistFromMiddle = 9999.0;

            float distFromRobot = goalCalculator.distanceCalculate(middle, robotPix);

            // we finsih this direction (obstacle or max dist)
            if (imgMap.at<uchar>(leftSide.y, leftSide.x) != 254 ||
                imgMap.at<uchar>(rightSide.y, rightSide.x) != 254 ||
                distFromRobot > (maxDistanceM / mapResolution_) )
            {

                for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
                {                    

                    // IF this is uncovered goal and inside the polygon
                    if (path_poses_with_status_.status_[i] == UN_COVERED) {
                        
                        if (pointPolygonTest(contour, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), false) > 0)
                        {                       

                            unCovered++;
                            float distFromMiddle = 
                                goalCalculator.distanceCalculate(convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]),
                                     middle);

                            if (distFromMiddle < minDistFromMiddle)
                            {
                                bestGoal = convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]);
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
            tfListener_.lookupTransform(globalFrame_, baseFrame_,
                                        ros::Time(0), transform);

            robotPose_.header.frame_id = globalFrame_;
            robotPose_.header.stamp = ros::Time::now();
            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();

            // if( robotHistoryPathMsg_.poses.size() > 0 ){

            //     if(robotHistoryPathMsg_.poses[robotHistoryPathMsg_.poses.size() -1].pose.position.x !=  
            //         robotPose_.pose.position.x && robotHistoryPathMsg_.poses[robotHistoryPathMsg_.poses.size() -1].pose.position.y !=  
            //         robotPose_.pose.position.y) {

            //             robotHistoryPathMsg_.poses.push_back(robotPose_);

            //             publishSanitizationRadius();

            //         }
            // }

            robotHistoryPathMsg_.poses.push_back(robotPose_);

            publishSanitizationRadius();
        

            return true;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

    void setOrientationByNextUnCoveredGoal(int index, geometry_msgs::PoseStamped &originalGoal)
    {

        if (index == path_poses_with_status_.coveragePathPoses_.size() - 1)
        {

            return;
        }

        for (int i = index + 1; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
        {

            if (path_poses_with_status_.status_[i] == UN_COVERED)
            {

                // this is the next un-covered goals, calculate direction from this goal
                auto nextGoal = path_poses_with_status_.coveragePathPoses_[i];

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

        if (path_poses_with_status_.coveragePathPoses_.size() == 0)
        {

            return 0.0;
        }

        float countCovered = 0.0;

        float countNeedToBeCovered = 0.0;

        if (coverage_state_ == COVERAGE_DONE)
        {

            for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
            {
                switch (path_poses_with_status_.status_[i])
                {

                case COVERED:
                {
                    // probably this is the aborted goals
                    countNeedToBeCovered += 1.0;
                }
                case COVERED_BY_ROBOT_PATH:
                {
                    countNeedToBeCovered += 1.0;
                    countCovered += 1.0;
                }
                }
            }
        }
        else
        {

            for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
            {
                switch (path_poses_with_status_.status_[i])
                {
                case UN_COVERED:
                {
                    countNeedToBeCovered += 1.0;
                }
                case COVERED:
                {
                    // probably this is the aborted goals
                    countNeedToBeCovered += 1.0;
                }
                case COVERED_BY_ROBOT_PATH:
                {
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

    bool moveFarwardIfRobotStuck()
    {

        if (!updateRobotLocation())
        {
            return false;
        }

        if (!costMapImg_.data)
        {

            return false;
        }

        tf::StampedTransform transform;

        geometry_msgs::PoseStamped robotOdomPose;

        try
        {

            // get current robot pose ODOM frame !!
            tfListener_.lookupTransform(odomFrame_, baseFrame_,
                                        ros::Time(0), transform);

            robotOdomPose.header.frame_id = odomFrame_;
            robotOdomPose.header.stamp = ros::Time::now();
            robotOdomPose.pose.position.x = transform.getOrigin().x();
            robotOdomPose.pose.position.y = transform.getOrigin().y();
            robotOdomPose.pose.position.z = 0;
            robotOdomPose.pose.orientation.x = transform.getRotation().x();
            robotOdomPose.pose.orientation.y = transform.getRotation().y();
            robotOdomPose.pose.orientation.z = transform.getRotation().z();
            robotOdomPose.pose.orientation.w = transform.getRotation().w();

            float robotHeading = atan2((2.0 *
                                        (robotOdomPose.pose.orientation.w * robotOdomPose.pose.orientation.z +
                                         robotOdomPose.pose.orientation.x * robotOdomPose.pose.orientation.y)),
                                       (1.0 - 2.0 * (robotOdomPose.pose.orientation.y * robotOdomPose.pose.orientation.y +
                                                     robotOdomPose.pose.orientation.z * robotOdomPose.pose.orientation.z)));

            // convert odom pose to odom pix
            cv::Point2d robotPix = cv::Point2d((robotOdomPose.pose.position.x - globalMapOriginPositionX_) / globalMapResolution_,
                                               (robotOdomPose.pose.position.y - globalMapOriginPositionY_) / globalMapResolution_);

            // Mat dbg = costMapImg_.clone();
            // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

            float robotRPix = (1.0 / globalMapResolution_) * (robot_w_m_ / 2);
            // circle(dbg, robotPix, robotRPix , Scalar(0,255,255), 1, 8, 0);

            float shiftFromCenter = robot_w_m_;
            float zoneRPix = robotRPix + (1.0 / globalMapResolution_) * (shiftFromCenter / 2);

            cv::Point2d robotHeadingPointBack(robotPix.x + (zoneRPix) * -1 * cos(robotHeading),
                                              robotPix.y + (zoneRPix) * -1 * sin(robotHeading));

            cv::Point2d robotHeadingPointBackLeft(robotPix.x + (zoneRPix)*cos(robotHeading - angles::from_degrees(90)),
                                                  robotPix.y + (zoneRPix)*sin(robotHeading - angles::from_degrees(90)));

            cv::Point2d robotHeadingPointBackRight(robotPix.x + (zoneRPix)*cos(robotHeading + angles::from_degrees(90)),
                                                   robotPix.y + (zoneRPix)*sin(robotHeading + angles::from_degrees(90)));
            bool allClear = true;

            vector<cv::Point2d> directions;
            directions.push_back(robotHeadingPointBack);
            directions.push_back(robotHeadingPointBackLeft);
            directions.push_back(robotHeadingPointBackRight);

            for (int i = 0; i < directions.size(); i++)
            {

                auto directionP = directions[i];
                cv::LineIterator it_map_back(costMapImg_, robotPix, directionP, 4); // 4 more dense than 8

                // check behind
                for (int j = 0; j < it_map_back.count; j++, ++it_map_back)
                {
                    cv::Point2d pointBeam = it_map_back.pos();
                    int valueTemp = costMapImg_.at<uchar>(pointBeam.y, pointBeam.x);
                    if (valueTemp == 255)
                    {
                        allClear = false;
                        break;
                    }
                }

                if (!allClear)
                {
                    cerr << " the index was " << i << endl;
                    break;
                }
            }

            if (!allClear)
            {

                // we can publish farward

                ros::Rate rate(1);
                float maxReversDuration = 1.0;

                geometry_msgs::Twist velocity;
                velocity.linear.x = 0.2;

                auto startTime = ros::WallTime::now();

                // Mat dbg = costMapImg_.clone();
                // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

                // cv::arrowedLine(dbg, robotPix, robotHeadingPointBack, Scalar(0, 255, 0), 2);

                // cv::arrowedLine(dbg, robotPix, robotHeadingPointBackLeft, Scalar(255, 0, 255), 2);

                // cv::arrowedLine(dbg, robotPix, robotHeadingPointBackRight, Scalar(255,0, 255), 2);

                // circle(dbg, robotPix, zoneRPix , Scalar(255,0,255), 1, 8, 0);

                // imshow(" dbg ",dbg);
                // waitKey(1);

                while (ros::ok())
                {

                    cerr << " FORRRWARD !!!!!!!!!! " << endl;
                    reverse_cmd_vel_pub_.publish(velocity);
                    rate.sleep();

                    auto end = ros::WallTime::now();

                    auto duration = (end - startTime).toSec();
                    if (duration > maxReversDuration)
                    {
                        break;
                    }

                    ros::spinOnce();

                    updateRobotLocation();
                }

                ros::Duration(0.1).sleep();

                return true;
            }

            return false;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

    bool makeReverseIfNeeded()
    {

        if (!updateRobotLocation())
        {
            return false;
        }

        if (!costMapImg_.data)
        {

            return false;
        }

        tf::StampedTransform transform;

        geometry_msgs::PoseStamped robotOdomPose;

        try
        {

            // get current robot pose ODOM frame !!
            tfListener_.lookupTransform(odomFrame_, baseFrame_,
                                        ros::Time(0), transform);

            robotOdomPose.header.frame_id = odomFrame_;
            robotOdomPose.header.stamp = ros::Time::now();
            robotOdomPose.pose.position.x = transform.getOrigin().x();
            robotOdomPose.pose.position.y = transform.getOrigin().y();
            robotOdomPose.pose.position.z = 0;
            robotOdomPose.pose.orientation.x = transform.getRotation().x();
            robotOdomPose.pose.orientation.y = transform.getRotation().y();
            robotOdomPose.pose.orientation.z = transform.getRotation().z();
            robotOdomPose.pose.orientation.w = transform.getRotation().w();

            float robotHeading = atan2((2.0 *
                                        (robotOdomPose.pose.orientation.w * robotOdomPose.pose.orientation.z +
                                         robotOdomPose.pose.orientation.x * robotOdomPose.pose.orientation.y)),
                                       (1.0 - 2.0 * (robotOdomPose.pose.orientation.y * robotOdomPose.pose.orientation.y +
                                                     robotOdomPose.pose.orientation.z * robotOdomPose.pose.orientation.z)));

            // convert odom pose to odom pix
            cv::Point2d robotPix = cv::Point2d((robotOdomPose.pose.position.x - globalMapOriginPositionX_) / globalMapResolution_,
                                               (robotOdomPose.pose.position.y - globalMapOriginPositionY_) / globalMapResolution_);

            // Mat dbg = costMapImg_.clone();
            // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

            float robotRPix = (1.0 / globalMapResolution_) * (robot_w_m_ / 2);
            // circle(dbg, robotPix, robotRPix , Scalar(0,255,255), 1, 8, 0);

            float shiftFromCenter = 0.8;
            float zoneRPix = robotRPix + (1.0 / globalMapResolution_) * (shiftFromCenter / 2);

            cv::Point2d robotHeadingPointFront(robotPix.x + (zoneRPix)*cos(robotHeading),
                                               robotPix.y + (zoneRPix)*sin(robotHeading));

            cv::Point2d robotHeadingPointBack(robotPix.x + (zoneRPix) * -1 * cos(robotHeading),
                                              robotPix.y + (zoneRPix) * -1 * sin(robotHeading));

            // cv::arrowedLine(dbg, robotPix, robotHeadingPointFront, Scalar(0, 255, 0), 2);

            // cv::arrowedLine(dbg, robotPix, robotHeadingPointBack, Scalar(0, 0, 255), 2);

            // circle(dbg, robotPix, zoneRPix , Scalar(255,0,255), 1, 8, 0);

            // check if robot blocked in front
            cv::LineIterator it_map_front(costMapImg_, robotPix, robotHeadingPointFront, 4); // 4 more dense than 8

            bool blockedInFront = false;
            for (int j = 0; j < it_map_front.count; j++, ++it_map_front)
            {
                cv::Point2d pointBeam = it_map_front.pos();
                int valueTemp = costMapImg_.at<uchar>(pointBeam.y, pointBeam.x);
                if (valueTemp != 0)
                {
                    blockedInFront = true;
                    break;
                }
            }

            if (blockedInFront)
            {

                // check if clear behind the robot

                cv::LineIterator it_map_back(costMapImg_, robotPix, robotHeadingPointBack, 4); // 4 more dense than 8

                bool clearInBack = true;
                for (int j = 0; j < it_map_back.count; j++, ++it_map_back)
                {
                    cv::Point2d pointBeam = it_map_back.pos();
                    int valueTemp = costMapImg_.at<uchar>(pointBeam.y, pointBeam.x);
                    if (valueTemp != 0)
                    {
                        clearInBack = false;
                        break;
                    }
                }

                if (clearInBack)
                {

                    // we can publish reverse

                    ros::Rate rate(1);
                    float maxReversDuration = 2.0;

                    geometry_msgs::Twist velocity;
                    velocity.linear.x = -0.2;

                    auto startTime = ros::WallTime::now();

                    while (ros::ok())
                    {

                        cerr << " REEEEEEVERSE !!!!!!!!!! " << endl;
                        reverse_cmd_vel_pub_.publish(velocity);
                        rate.sleep();

                        auto end = ros::WallTime::now();

                        auto duration = (end - startTime).toSec();
                        if (duration > maxReversDuration)
                        {
                            break;
                        }

                        ros::spinOnce();

                        updateRobotLocation();

                        removeGoalsByRobotRout();
                    }

                    ros::Duration(0.5).sleep();

                    return true;
                }
            }

            // imshow("dbg", dbg);
            // waitKey(1);

            return false;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

    void setCoverageState(bool coverageState)
    {

        coverageStateStarts_ = coverageState;
    }

    static void mySigintHandler(int sig, void *ptr)
    {

        cerr << " user pressed CTRL+C " << endl;
        exit_ = true;
    }

private:
    void publishRobotHistoryPath()
    {

        robotHistoryPathMsg_.header.stamp = ros::Time::now();
        robotHistoryPathMsg_.header.frame_id = globalFrame_;

        robot_history_path_pub_.publish(robotHistoryPathMsg_);
    }

    void cameraScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {

        currentCameraScanMapPointsM_.clear();

        for (double i = 0; i < scan->ranges.size(); i++)
        {

            if (isinf(scan->ranges[i]) == false)
            {

                double ray_angle = scan->angle_min + (i * scan->angle_increment);

                cv::Point2d rayPoint((scan->ranges[i] * cos(ray_angle)),
                                     (scan->ranges[i] * sin(ray_angle)));

                // transform to map frame
                cv::Point3d p = cv::Point3d(rayPoint.x, rayPoint.y, 0);

                auto transformedRayPoint = transformFrames(p, globalFrame_, scan->header.frame_id, scan->header.stamp);

                currentCameraScanMapPointsM_.push_back(cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));
            }
        }
    }


    bool findNextGoalByDirection(cv::Point2d& finalGoalToNavigate,
        int* bestGoalIndex,
        const cv::Point2d& robotPix, float robot_w_m,
        float mapResolution_, float robotHeading,
        const cv::Mat& imgMap) {

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
        scores.push_back(getScoreByDirection(rotattionAngleFront,
                                                robotFootprintFrontLeft, robotFootprintFrontRight,
                                                bestGoalPixFront, &goalINdexFront, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexFront, "front"));
        bestIndexesFromWaypoints.push_back(goalINdexFront);

        // front-left
        cv::Point2d bestGoalPixFrontLeft;
        vector<int> deubugIndexFrontLeft;
        int goalINdexFrontLeft = -1;
        float rotattionAngleFrontLeft = robotHeading + angles::from_degrees(-45.0);
        scores.push_back(getScoreByDirection(rotattionAngleFrontLeft,
                                                robotFootprintBackLeft, robotFootprintFrontRight,
                                                bestGoalPixFrontLeft, &goalINdexFrontLeft, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexFrontLeft, "front-left"));
        bestIndexesFromWaypoints.push_back(goalINdexFrontLeft);

        // front-right
        cv::Point2d bestGoalPixFrontRight;
        vector<int> deubugIndexFrontRight;
        int goalINdexFrontRight = -1;
        float rotattionAngleFrontRight = robotHeading + angles::from_degrees(45.0);
        scores.push_back(getScoreByDirection(rotattionAngleFrontRight,
                                                robotFootprintBackRIGHT, robotFootprintFrontLeft,
                                                bestGoalPixFrontRight, &goalINdexFrontRight, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexFrontRight, "front-right"));
        bestIndexesFromWaypoints.push_back(goalINdexFrontRight);

        // back
        cv::Point2d bestGoalPixBack;
        vector<int> deubugIndexBack;
        int goalINdexBack = -1;
        float rotattionAngleBack = robotHeading + angles::from_degrees(180);
        scores.push_back(getScoreByDirection(rotattionAngleBack,
                                                robotFootprintBackLeft, robotFootprintBackRIGHT,
                                                bestGoalPixBack, &goalINdexBack, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexBack, "back"));
        bestIndexesFromWaypoints.push_back(goalINdexBack);

        // back-left
        cv::Point2d bestGoalPixBackLeft;
        vector<int> deubugIndexBackLeft;
        int goalINdexBackLeft = -1;
        float rotattionAngleBackLeft = robotHeading + angles::from_degrees(180 + 45);
        scores.push_back(getScoreByDirection(rotattionAngleBackLeft,
                                                robotFootprintBackRIGHT, robotFootprintFrontLeft,
                                                bestGoalPixBackLeft, &goalINdexBackLeft, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexBackLeft, "back-left"));
        bestIndexesFromWaypoints.push_back(goalINdexBackLeft);

        // back-left
        cv::Point2d bestGoalPixBackRight;
        vector<int> deubugIndexBackRight;
        int goalINdexBackRight = -1;
        float rotattionAngleBackRight = robotHeading + angles::from_degrees(180 - 45);
        scores.push_back(getScoreByDirection(rotattionAngleBackRight,
                                                robotFootprintBackLeft, robotFootprintFrontRight,
                                                bestGoalPixBackRight, &goalINdexBackRight, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexBackRight, "back-right"));
        bestIndexesFromWaypoints.push_back(goalINdexBackRight);

        // left
        cv::Point2d bestGoalPixLeft;
        vector<int> deubugIndexLeft;
        int goalINdexLeft = -1;
        float rotattionAngleLeft = robotHeading + angles::from_degrees(-90.0);
        scores.push_back(getScoreByDirection(rotattionAngleLeft,
                                                robotFootprintFrontLeft, robotFootprintBackLeft,
                                                bestGoalPixLeft, &goalINdexLeft, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexLeft, "left"));
        bestIndexesFromWaypoints.push_back(goalINdexLeft);

        // right
        cv::Point2d bestGoalPixRight;
        vector<int> deubugIndexRight;
        int goalINdexRight = -1;
        float rotattionAngleRight = robotHeading + angles::from_degrees(90.0);
        scores.push_back(getScoreByDirection(rotattionAngleRight,
                                                robotFootprintFrontRight, robotFootprintBackRIGHT,
                                                bestGoalPixRight, &goalINdexRight, robotPix, robotHeading, robot_w_m,
                                                mapResolution_, imgMap, deubugIndexRight, "right"));
        bestIndexesFromWaypoints.push_back(goalINdexRight);


        vector<cv::Point2d> goals{bestGoalPixFront,
                                    bestGoalPixFrontLeft, bestGoalPixFrontRight,
                                    bestGoalPixBack,
                                    bestGoalPixBackLeft, bestGoalPixBackRight,
                                    bestGoalPixLeft, bestGoalPixRight};

        vector<vector<int>> waypointsIndexses{deubugIndexFront,
                                                deubugIndexFrontLeft, deubugIndexFrontRight,
                                                deubugIndexBack,
                                                deubugIndexBackLeft, deubugIndexBackRight,
                                                deubugIndexLeft, deubugIndexRight};

        visualization_msgs::MarkerArray Markerarr;

        for (int i = 0; i < goals.size(); i++) {

            geometry_msgs::Quaternion q;
            
            float angle = atan2(goals[i].y - robotPix.y,
                                goals[i].x - robotPix.x);
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
            marker.color.a = 1.0; // Don't forget to set the alpha!
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
        int score_threshold = 0;

        for (int i = 0; i < goals.size(); i++)
        {

            if (scores[i] > maxScore && scores[i] > score_threshold)
            {
                index = i;
                maxScore = scores[i];
            }
        }

        if( index != -1 )
        {
            finalGoalToNavigate = goals[index];
            *bestGoalIndex = bestIndexesFromWaypoints[index];

        } 
        else 
        {
            return false;
        }

        cerr<<"maxScore "<<maxScore<<endl;


        if (maxScore > score_threshold )
        {          

            return true;               
        } 
        

        return false;
                


    }


    bool findSafestGoalFromUncoveredGoals(cv::Point2d &nextGoal, 
        const cv::Mat &imgMap, const Path_with_Status &waypointsWithStatus, 
        float goals_m_resolution, float map_resolution,
        float miAreaForComponentM = 0.5)
    {

        Mat binary = cv::Mat(imgMap.rows, imgMap.cols,
                            CV_8UC1, cv::Scalar(0));


        

        float pixRes = goals_m_resolution / map_resolution;
        for (int i = 0; i < waypointsWithStatus.status_.size(); i++)
        {

            if (waypointsWithStatus.status_[i] == UN_COVERED)
            {

                circle(binary, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), pixRes, Scalar(255), -1, 8, 0);
            }
        }

        

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

        if (contours.size() == 0)
        {
        return false;
        }

        float maxArea = 0.0;
        int index = -1;
        for( int i =0; i < contours.size(); i++ ){

            Rect r = cv::boundingRect(contours[i]);
            float areaM = (r.width * map_resolution) * (r.height * map_resolution);

            if ( areaM > miAreaForComponentM){

                if ( areaM > maxArea ){
                    maxArea = areaM;
                    index = i;
                }
                else {
                    drawContours(binary, contours, i, Scalar(0), -1);

                }
            } else {
                drawContours(binary, contours, i, Scalar(0), -1);

            }
        }

        
        if (index == -1 ){
            return false;
        }

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
            cerr<<" found "<<nextGoal<<endl;
            return true;
        }

        return false;

    }

    bool findNextGoalByConnetctedComponents(cv::Point2d& finalGoalToNavigate, const Mat& imgMap,
        const Path_with_Status& waypointsWithStatus, float goals_m_resolution, float mapResolution_)
    {



        if (findSafestGoalFromUncoveredGoals(finalGoalToNavigate,
            imgMap, waypointsWithStatus, goals_m_resolution, mapResolution_))
        {
            // circle(imgMap, nextGoal, goals_m_resolution / mapResolution_, Scalar(0), -1, 8, 0);


            return true;
        }
        else
        {   
            return false;
        }
        
    }

    geometry_msgs::PointStamped transformFrames(
        Point3d objectPoint3d, string target_frame, string source_Frame, ros::Time t)
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
            tf::StampedTransform transform_;

            tfListener_.lookupTransform(target_frame, source_Frame,
                                        ros::Time(0), transform_);

            tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

            return pointStampedOut;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());

            return pointStampedOut;
        }
    }

    Mat occupancyGridMatToGrayScale(const Mat &map)
    {
        // CV_8UC1 - UINT image type
        Mat output(map.rows, map.cols, CV_8UC1, Scalar(205));

        for (int j = 0; j < map.rows; j++)
        {
            for (int i = 0; i < map.cols; i++)
            {

                auto value = map.at<int8_t>(cv::Point(i, j));
                uint8_t newValue = 0;

                if (value == UNKNOWN) // unknown
                    newValue = 205;
                else if (value == FREE) // clear
                    newValue = 254;
                else if (value == BLOCKED) // occupay
                    newValue = 0;

                output.at<uchar>(cv::Point(i, j)) = newValue;
            }
        }

        return output;
    }

    void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {

        globalMapOriginPositionX_ = msg->info.origin.position.x;
        globalMapOriginPositionY_ = msg->info.origin.position.y;
        globalMapResolution_ = msg->info.resolution;

        auto endLocalCostMap = high_resolution_clock::now();
        auto durationFromLastCalc = duration_cast<seconds>(endLocalCostMap - startLocalCostMap_).count();

        /// do this every 2 seconds
        if (init_ && durationFromLastCalc > 0.3 )
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
                        costMapImg_.at<uchar>(j, i) = 255;
                    }
                    else if (value > 0 && value < 100)
                    {
                        costMapImg_.at<uchar>(j, i) = 100;
                    }
                }
            }
            
            initGlobalCostMap_ = true;

            string global_costmap_frame = msg->header.frame_id;

            // Mat dbg = costMapImg.clone();
            // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

            for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
            {

                // if this goal cant be n obstacle
                if (path_poses_with_status_.status_[i] == COVERED ||
                    path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
                {

                    continue;
                }

                // transform to odom frame (global costmap framme)
                cv::Point3d p = cv::Point3d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                                            path_poses_with_status_.coveragePathPoses_[i].pose.position.y, 0);

                auto poseInOdomFrame = transformFrames(p, global_costmap_frame, globalFrame_, msg->header.stamp);

                // convert odom pose to odom pix
                float xPix = (poseInOdomFrame.point.x - globalMapOriginPositionX_) / globalMapResolution_;
                float yPix = (poseInOdomFrame.point.y - globalMapOriginPositionY_) / globalMapResolution_;

                cv::Point pOnImg = cv::Point(xPix, yPix);

                // get the cost value
                int costVal = costMapImg_.at<uchar>(pOnImg.y, pOnImg.x);

                float distRobotFromGoal =
                    goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                                     cv::Point2d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                                                                 path_poses_with_status_.coveragePathPoses_[i].pose.position.y));

                // GOAL ON obstacle
                if (costVal != 0)
                {

                    // the goal inside the wanted radius
                    if (distRobotFromGoal < 8.0)
                    {

                        path_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);
                    }
                    else
                    {

                        /// goal outside the raius, make it uncovered
                        path_poses_with_status_.setStatByIndex(i, UN_COVERED);
                    }

                    // circle(dbg, pOnImg,  1, Scalar(0,255,0), -1, 8, 0);

                } /// goal not inside obstacle
                else
                {

                    // if inside radius but last time int was inside obstacle,
                    // keep it as obstacle
                    if (distRobotFromGoal < 2.0 &&
                        path_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
                    {

                        path_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);
                    }
                    else
                    {

                        path_poses_with_status_.setStatByIndex(i, UN_COVERED);
                    }
                }
            }

            // imwrite("/home/algo-kobuki/imgs/dbg.png", dbg);
            // imwrite("/home/algo-kobuki/imgs/gmapping.png", currentGlobalMap_);

            startLocalCostMap_ = endLocalCostMap;
        }

    }

    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
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

                updateCurrentBlobsFrontiers(old_origin_x, old_origin_y,
                                            new_origin_x, new_origin_y,
                                            deltaW, deltaH);
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

        init_ = true;

        currentGlobalMap_ = occupancyGridMatToGrayScale(tmp.clone());

        mappingMap_ = currentGlobalMap_.clone();

        // addDilationForGlobalMap(currentGlobalMap_, /*walls_inflation_m_*/0.1, mapResolution_);

        addFreeSpaceDilation(currentGlobalMap_);
    }

    void updateCurrentBlobsFrontiers(double old_origin_x, double old_origin_y,
                                     double new_origin_x, double new_origin_y,
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

    void publishEdgesFrontiers(const std::vector<Frontier> &currentEdgesFrontiers)
    {

        visualization_msgs::MarkerArray Markerarr;
        int count = 1;

        RNG rng(12345);

        for (int i = 0; i < currentEdgesFrontiers.size(); i++)
        {

            Scalar color = Scalar(rng.uniform(0, 0), rng.uniform(0, 0), rng.uniform(0, 255));

            for (int j = 0; j < currentEdgesFrontiers[i].contour.size(); j++)
            {

                cv::Point pixP = currentEdgesFrontiers[i].contour[j];
                geometry_msgs::Quaternion q;
                q.w = 1;
                geometry_msgs::PoseStamped p = convertPixToPose(pixP, q);

                visualization_msgs::Marker pOnEgge;
                pOnEgge.header.frame_id = globalFrame_;
                pOnEgge.header.stamp = ros::Time::now();
                pOnEgge.ns = "points_and_lines";
                pOnEgge.id = count;
                pOnEgge.action = visualization_msgs::Marker::ADD;
                pOnEgge.type = visualization_msgs::Marker::SPHERE;
                pOnEgge.pose.position.x = p.pose.position.x;
                pOnEgge.pose.position.y = p.pose.position.y;
                pOnEgge.pose.position.z = 0;
                pOnEgge.pose.orientation.x = 0;
                pOnEgge.pose.orientation.y = 0;
                pOnEgge.pose.orientation.z = 0;
                pOnEgge.pose.orientation.w = 1.0;
                pOnEgge.scale.x = 0.1;
                pOnEgge.scale.y = 0.1;
                pOnEgge.scale.z = 0.1;
                pOnEgge.color.a = 1.0;
                pOnEgge.color.r = float(color[0]) / 255.0;
                pOnEgge.color.g = float(color[1]) / 255.0;
                pOnEgge.color.b = float(color[2]) / 255.0;

                Markerarr.markers.push_back(pOnEgge);

                count++;
            }
        }

        edges_frontires_marker_array_Pub.publish(Markerarr);
    }

    void publishWaypointsWithStatus()
    {

        visualization_msgs::MarkerArray Markerarr;
        int count = 1;

        for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
        {
            visualization_msgs::Marker m;
            m.header.frame_id = globalFrame_;
            m.header.stamp = ros::Time::now();
            m.ns = "points_and_lines";
            m.id = count + i;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::SPHERE;
            m.pose.position.x = path_poses_with_status_.coveragePathPoses_[i].pose.position.x;
            m.pose.position.y = path_poses_with_status_.coveragePathPoses_[i].pose.position.y;
            m.pose.position.z = 0;
            m.pose.orientation.x = 0;
            m.pose.orientation.y = 0;
            m.pose.orientation.z = 0;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.color.a = 1.0;

            // checked

            /// COVERED = BLACK
            if (path_poses_with_status_.status_[i] == COVERED)
            {
                m.color.r = 253.0 / 255.0;
                m.color.g = 218.0 / 255.0;
                m.color.b = 13.0 / 255.0;

            } /// UN_COVERED = GREEN
            else if (path_poses_with_status_.status_[i] == UN_COVERED)
            {

                m.color.r = 0.0;
                m.color.g = 1.0;
                m.color.b = 0.0;

            } /// COVERED_BY_ROBOT_PATH = LIGHT BLUE
            else if (path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
            {

                m.color.r = 0.0;
                m.color.g = 0.1;
                m.color.b = 1.0;
            } /// COVERED_BY_OBSTACLE = RED

            else if (path_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
            {

                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
            }

            Markerarr.markers.push_back(m);
        }

        waypoints_with_status_pub_.publish(Markerarr);
    }

    cv::Point convertPoseToPix(const geometry_msgs::PoseStamped &pose)
    {

        float xPix = (pose.pose.position.x - map_origin_position_x) / mapResolution_;
        float yPix = (pose.pose.position.y - map_origin_position_y) / mapResolution_;

        cv::Point p = cv::Point(xPix, yPix);

        return p;
    }

    geometry_msgs::PoseStamped convertPixToPose(const cv::Point &pixel,
                                                geometry_msgs::Quaternion q)
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

    vector<geometry_msgs::PoseStamped> covertPointsPathToPoseRout(
        const vector<cv::Point> &pointsPath)
    {

        vector<geometry_msgs::PoseStamped> posesPath;

        float prevAngle = 0.0;
        for (int i = 0; i < pointsPath.size(); i++)
        {

            float angle = atan2(pointsPath[i + 1].y - pointsPath[i].y,
                                pointsPath[i + 1].x - pointsPath[i].x);
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

    void publishCoveragePath(const vector<geometry_msgs::PoseStamped> &coveragePathPoses)
    {

        nav_msgs::Path msgMsg;
        msgMsg.header.frame_id = globalFrame_;
        msgMsg.header.stamp = ros::Time::now();
        msgMsg.poses = coveragePathPoses;

        cuurentCoveragePathPub_.publish(msgMsg);

        ros::Duration(1).sleep();
    }

    cv::Point fixLocationOnGrid(const cv::Point &goal, const cv::Point &ref)
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
            cv::Point downLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)), p.y + ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point downRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)), p.y + ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point topLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)), p.y - ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point topRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)), p.y - ((getDistanceBetweenGoalsPix() * scalar)));

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
            cv::Point downLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)), p.y + ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point downRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)), p.y + ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point topLeft(p.x - ((getDistanceBetweenGoalsPix() * scalar)), p.y - ((getDistanceBetweenGoalsPix() * scalar)));
            cv::Point topRight(p.x + ((getDistanceBetweenGoalsPix() * scalar)), p.y - ((getDistanceBetweenGoalsPix() * scalar)));

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

        return pError;
    }

    void removeGoalsByRobotRout()
    {

        for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
        {

            /// if the goal already covered
            if (path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH)
            {
                continue;
            }

            float distRobotFromGoal =
                goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                                                 cv::Point2d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                                                             path_poses_with_status_.coveragePathPoses_[i].pose.position.y));

            if (distRobotFromGoal < sanitization_radius_)
            {

                path_poses_with_status_.setStatByIndex(i, COVERED_BY_ROBOT_PATH);
            }
        }
    }

    string getMoveBaseState(actionlib::SimpleClientGoalState state)
    {

        switch (state.state_)
        {
        case actionlib::SimpleClientGoalState::ACTIVE:
        {
            return "ACTIVE";
        }
        case actionlib::SimpleClientGoalState::PENDING:
        {
            return "PENDING";
        }
        case actionlib::SimpleClientGoalState::RECALLED:
        {
            return "RECALLED";
        }
        case actionlib::SimpleClientGoalState::REJECTED:
        {
            return "REJECTED";
        }
        case actionlib::SimpleClientGoalState::PREEMPTED:
        {
            return "PREEMPTED";
        }
        case actionlib::SimpleClientGoalState::ABORTED:
        {
            return "ABORTED";
        }
        case actionlib::SimpleClientGoalState::SUCCEEDED:
        {
            return "SUCCEEDED";
        }
        case actionlib::SimpleClientGoalState::LOST:
        {
            return "LOST";
        }
        }

        return "";
    }

    bool sendGoal(const geometry_msgs::PoseStamped &goalMsg, int goalIndex = -1, bool in_explore = false)
    {

        // navigate to the point
        moveBaseController_.navigate(goalMsg);

        bool result = true;

        while (ros::ok())
        {

            ros::spinOnce();

            updateRobotLocation();

            removeGoalsByRobotRout();

            publishRobotHistoryPath();

            publishWaypointsWithStatus();

            // if we already reached the goal, no need to rotate
            if (goalIndex != -1)
            {
                if (path_poses_with_status_.status_[goalIndex] != UN_COVERED)
                {
                    moveBaseController_.moveBaseClient_.cancelGoal();
                    return true;
                }
            }

            if ( in_explore){

                if (goalCalculator.distanceCalculate(
                    cv::Point2d(goalMsg.pose.position.x, goalMsg.pose.position.y), 
                        cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y)) < 0.2){
                
                    moveBaseController_.moveBaseClient_.cancelGoal();
                    return true;
                
                }

            }

            if (exit_)
            {

                return true;
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
                cerr << "strState:  " << strState << endl;

                result = false;
                break;
            }

            ros::spinOnce();
        }

        return result;
    }

    void saveCoverageImg()
    {

        if (!imgSaved_ && mappingMap_.data)
        {

            // Mat patternImg = mappingMap_.clone();
            Mat robotTreaceImg = mappingMap_.clone();

            // cvtColor(patternImg, patternImg, COLOR_GRAY2BGR);
            cvtColor(robotTreaceImg, robotTreaceImg, COLOR_GRAY2BGR);

            // // draw the pattern
            // for (int i = 0; i < path_poses_with_status_.path_.size() - 1; i++)
            // {

            //     cv::line(patternImg, path_poses_with_status_.path_[i],
            //              path_poses_with_status_.path_[i + 1], Scalar(34, 139, 139), 1);
            // }

            // draw the robot trace (INCLUDE DIMS)
            for (int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++)
            {
                circle(robotTreaceImg, convertPoseToPix(robotHistoryPathMsg_.poses[i]), sanitization_radius_  / mapResolution_,
                     Scalar(250, 206, 135), -1, 8, 0);
                 
            }

            // put back the black and the gray color to the map 
            for (int j = 0; j < mappingMap_.rows; j++)
            {
                for (int i = 0; i < mappingMap_.cols; i++)
                {

                    int value = mappingMap_.at<uchar>(j, i);

                    if (value == 0 )
                    {
                        robotTreaceImg.at<cv::Vec3b>(j, i)[0] = 0;
                        robotTreaceImg.at<cv::Vec3b>(j, i)[1] = 0;
                        robotTreaceImg.at<cv::Vec3b>(j, i)[2] = 0;

                        
                    }
                    else if (value  == 205)
                    {
                        robotTreaceImg.at<cv::Vec3b>(j, i)[0] = 205;
                        robotTreaceImg.at<cv::Vec3b>(j, i)[1] = 205;
                        robotTreaceImg.at<cv::Vec3b>(j, i)[2] = 205;
                    }
                }
            }
            // draw the trace only
            for (int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++)
            {

                cv::Point p1 = convertPoseToPix(robotHistoryPathMsg_.poses[i]);
                cv::Point p2 = convertPoseToPix(robotHistoryPathMsg_.poses[i + 1]);

                cv::line(robotTreaceImg, p1, p2, Scalar(238, 104, 123), 1);
            }

            // draw the grid
            for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
            {

                if (path_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE)
                {   
                    cerr<<" COVERED_BY_OBSTACLE "<<i<<endl;
                    circle(robotTreaceImg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 0, 255), -1, 8, 0);
                }
                else if (path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH ||  
                    path_poses_with_status_.status_[i] == COVERED)
                {
                    circle(robotTreaceImg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(255, 0, 0), -1, 8, 0);
                }
                else if (path_poses_with_status_.status_[i] == UN_COVERED)
                {
                    circle(robotTreaceImg, convertPoseToPix(path_poses_with_status_.coveragePathPoses_[i]), 1, Scalar(0, 255, 0), -1, 8, 0);
                }

                
            }

            /*
                Hi yakir,  can you change the file format  to these:
                YYYY_MM_DD_HH_MM_(Duration_in_mins)_percentage.png for sorting purpose.
                For "rosparam set /disinfect_report" use the same format YYYY_MM_DD_HH_MM_(Duration_in_mins)_percentage without .png extension.
            */

            auto end = high_resolution_clock::now();
            auto durationCoverage = duration_cast<seconds>(end - startingCoverageTime_);

            int durationMinutes = durationCoverage.count() / 60;

            percentCoverage_ = getCurrentPercentCoverage();
            string image_name_format = startingTime_ + '_' + to_string(durationMinutes) + '_' + to_string(int(percentCoverage_));
            string full_img_name = coverage_img_path_ + image_name_format + ".png";
            node_.setParam("/coverage/image_name", image_name_format);

            cerr << "full_img_name: " << full_img_name << endl;

            cv::imwrite(full_img_name, robotTreaceImg);
            // cv::imwrite(coverage_img_path_ + "patthern.png", patternImg);

            imgSaved_ = true;
        }
    }

    void addDilationByGlobalCostMap(const Mat& globalCostMap, Mat& algoMap,
         const cv::Point2d& robotBaseFootPrint ) {

        if(!costMapImg_.data || !algoMap.data || robotBaseFootPrint.x < 0 || robotBaseFootPrint.y < 0){
            return;
        }        
        cv::Point2d mapCenter(algoMap.cols /2 , algoMap.rows / 2);
        cv::Point2d costMapCenter(globalCostMap.cols /2 , globalCostMap.rows / 2);


        // cv::Point robotBaseFootPrint;
        
        // robotBaseFootPrint.x = (1.1 + 10) /  0.05;
        // robotBaseFootPrint.y = (-1.43  + 10) / 0.05;
        // cerr<<" robotBaseFootPrint x "<<robotBaseFootPrint.x<<" robotBaseFootPrint y "<<robotBaseFootPrint.y<<endl;

        int deltaRobotX = mapCenter.x - robotBaseFootPrint.x;
        int deltaRobotY = mapCenter.y - robotBaseFootPrint.y;


        for (int y = 0; y < globalCostMap.rows; y++)
        {
            for (int x = 0; x < globalCostMap.cols; x++)
            {

                int value = globalCostMap.at<uchar>(y, x);

                if (value > 0)
                {
                    cv::Point2d nP(x + (mapCenter.x - costMapCenter.x) - deltaRobotX , 
                        y + (mapCenter.y - costMapCenter.y)  - deltaRobotY);

                    //  cv::Point2d nP(x + (mapCenter.x - costMapCenter.x), 
                    //     y + (mapCenter.y - costMapCenter.y)  );
                

                    algoMap.at<uchar>(nP.y , nP.x) = 0;
                   
                }
            
            }
        }

    }

private:
    COVERAGE_STATE coverage_state_ = COVERAGE_STATE::COVERAGE_BY_STRAIGHT_LINES;
    bool coverageStateStarts_ = false;

    EXPLORE_STATE explore_state_ = EXPLORE_STATE::IDLE;

    // move-base
    MoveBaseController moveBaseController_;

    // subs
    ros::Subscriber global_map_sub_;

    ros::Subscriber global_cost_map_sub_;

    // pubs

    ros::Publisher reverse_cmd_vel_pub_;

    ros::Publisher cuurentCoveragePathPub_;

    ros::Publisher edges_frontires_marker_array_Pub;

    ros::Publisher waypoints_with_status_pub_;

    ros::Publisher directions_marker_array_pub_;

    image_transport::Publisher coverage_map_pub_;

    ros::Publisher sanitization_radius_marker_pub_;

    ros::Publisher robot_history_path_pub_;
    nav_msgs::Path robotHistoryPathMsg_;

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

    Path_with_Status path_poses_with_status_;

    ros::NodeHandle node_;

    // params

    geometry_msgs::PoseStamped robotPose_;

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

    bool init_ = false;

    bool initGlobalCostMap_ = false;

    float width_;

    float height_;

    double distBetweenGoalsM_ = 0.5;

    tf::TransformListener tfListener_;

    string globalFrame_ = "map";

    string baseFrame_ = "base_footprint";

    string odomFrame_ = "odom";

    bool reducing_goals_ = true;

    double robot_w_m_ = 0.53;
    double robot_h_m_ = 0.53;
    double walls_inflation_m_ = 0.3;

    double sanitization_radius_ = 1.0;

    double radius_for_cleaning_route_goals_ = 0.2;

    double duration_wait_for_move_base_response_ = 15.0;

    double percentCoverage_ = 0.0;

    string coverage_img_path_ = "";

    string image_name_ = "";

    string state_ = "INITIALIZING";

    bool imgSaved_ = false;

    // robot footprint
    bool gotFootPrint_ = false;
    geometry_msgs::PolygonStamped currentRobotFootPrintLocationOodm_;

    bool errCoverage_ = false;

    high_resolution_clock::time_point startingCoverageTime_;

    high_resolution_clock::time_point startLocalCostMap_;
};

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "map_coverage_exploration_node" , ros::init_options::NoSigintHandler);

    MapCoverageManager mapCoverageManager;
    mapCoverageManager.setCoverageState(false);

    signal(SIGINT, (void (*)(int))MapCoverageManager::mySigintHandler); 


    if ( mapCoverageManager.exlpore()) {
        
        mapCoverageManager.setCoverageState(true);

        mapCoverageManager.coverage();

    }

    ros::spin();

    return 0;
}


// int main(){


//     Mat mappingMap = imread("/home/yakir/distance_transform_coverage_ws/ddd/currentGlobalMap_.pgm",1);
//     Mat globalCostMap = imread("/home/yakir/distance_transform_coverage_ws/ddd/costmap.pgm",0);

//     cv::Point2d mapCenter(mappingMap.cols /2 , mappingMap.rows / 2);
//     cv::Point2d costMapCenter(globalCostMap.cols /2 , globalCostMap.rows / 2);


//     cv::Point robotBaseFootPrint;
    
//     robotBaseFootPrint.x = (1.1 + 10) /  0.05;
//     robotBaseFootPrint.y = (-1.43  + 10) / 0.05;
//     cerr<<" robotBaseFootPrint x "<<robotBaseFootPrint.x<<" robotBaseFootPrint y "<<robotBaseFootPrint.y<<endl;

//     int deltaRobotX = mapCenter.x - robotBaseFootPrint.x;
//     int deltaRobotY = mapCenter.y - robotBaseFootPrint.y;


//     for (int y = 0; y < globalCostMap.rows; y++)
//     {
//         for (int x = 0; x < globalCostMap.cols; x++)
//         {

//             int value = globalCostMap.at<uchar>(y, x);

//             if (value > 0)
//             {
//                 cv::Point2d nP(x + (mapCenter.x - costMapCenter.x) - deltaRobotX , 
//                     y + (mapCenter.y - costMapCenter.y)  - deltaRobotY);

//                 //  cv::Point2d nP(x + (mapCenter.x - costMapCenter.x), 
//                 //     y + (mapCenter.y - costMapCenter.y)  );
               

//                 mappingMap.at<cv::Vec3b>(nP.y , nP.x)[0] = 0;
//                 mappingMap.at<cv::Vec3b>(nP.y , nP.x)[1] = 0;
//                 mappingMap.at<cv::Vec3b>(nP.y , nP.x)[2] = 255;
    
//             }
           
//         }
//     }

//     circle(mappingMap, robotBaseFootPrint, 3, Scalar(255,0 , 255));

//     imshow("mappingMap",mappingMap);
//     waitKey(0);
    
// }