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


#include <tf/tf.h>
#include <tf/transform_listener.h>


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
    COVERED  = 1,
    COVERED_BY_ROBOT_PATH = 2,
    COVERED_BY_OBSTACLE = 3    
};    

struct Path_with_Status
{
    vector<geometry_msgs::PoseStamped> coveragePathPoses_;

    vector<GoalState> status_;

    vector<cv::Point> path_; // for display on image


    void initStatusList( ){

        status_.resize(coveragePathPoses_.size());

        for( int i = 0; i < status_.size(); i++){

            setStatByIndex(i, UN_COVERED);
        }

    }

    void setStatByIndex(int index, GoalState status) {

        status_[index] = status;
    }

    void setPixelsPath(const  vector<cv::Point>& path){

        path_.resize(path.size());

        for(int i = 0; i < path.size(); i++ ){

            path_.push_back(cv::Point2d(path[i].x, path[i].x));
        }
    }

};

    

string getCurrentTime(){ 


    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (buffer,80,"%F/%H_%M",timeinfo);
    string curr_time_Str = strdup(buffer);
    std::replace( curr_time_Str.begin(), curr_time_Str.end(), '/', '_');
    std::replace( curr_time_Str.begin(), curr_time_Str.end(), '-', '_');


    return curr_time_Str;    
}


void addDilationForGlobalMap(Mat &imgMap, float walls_inflation_m, float mapResolution)
{

    try
    {
        int dilationPix = (1.0 / mapResolution) * (walls_inflation_m );        

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

 void addFreeSpaceDilation(Mat& grayscaleImg) {

        Mat binary  = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, 
            CV_8UC1, cv::Scalar(0));

        binary.setTo(255, grayscaleImg >= 254); 
        dilate(binary, binary, Mat(), Point(-1, -1), 1, 1, 1);     

        Mat newImg  = cv::Mat(grayscaleImg.rows, grayscaleImg.cols, 
            CV_8UC1, cv::Scalar(205));

        newImg.setTo(254, binary >= 254); 
        newImg.setTo(0, grayscaleImg == 0); 

        grayscaleImg = newImg;

    }


class MapCoverageManager
{

public:
    MapCoverageManager()
    {

        // cerr<<" wait for move-base server "<<endl;
        // moveBaseController_.waitForServer(ros::Duration(10.0));
        // ros::Duration(1).sleep();
        // cerr << " exploration is now connecting with move-base !! " << endl;

        // rosparam
        ros::NodeHandle nodePrivate("~");
        nodePrivate.param("distance_between_goals_m", distBetweenGoalsM_, 0.5);
        nodePrivate.param("walls_inflation_m", walls_inflation_m_, 0.3);
        nodePrivate.param("robot_w_m", robot_w_m_, 0.53);
        nodePrivate.param("robot_h_m", robot_h_m_, 0.53);



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
        robot_footprint_sub_ =
             node_.subscribe<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint", 1,
                                                      &MapCoverageManager::footprintCallback, this); 
                                                 

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


        // image_transport::ImageTransport it(node_);
        // coverage_map_pub_ = it.advertise("/coverage_map_img", 1);
        // nodePrivate.param("/coverage_map_img/compressed/jpeg_quality", 20);

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

    ~MapCoverageManager() {


        moveBaseController_.cancelNavigation();
        ros::Duration(1).sleep();

        cerr<<"MapCoverageManager distructor "<<endl;
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


            if (exit_){

                return false;
            }


            if (!init_)
            {
                cerr << "map not recieved !!" << endl;

                continue;;
            }

            if ( !updateRobotLocation())
            {
                cerr << "can't update robot location !!" << endl;

                continue;
            }     

            node_.setParam("/coverage/state", "RUNNING");      

            currentAlgoMap_ = getCurrentMap();

            Mat explorationImgMap = currentAlgoMap_.clone();
            
            // set the unreached goals on the exploration map
            
            for(int i = 0; i < unreachedPointFromFronitiers.size(); i++ ){
                
                int radiusPix = (1.0 / mapResolution_) * (walls_inflation_m_ );    
                circle(explorationImgMap, unreachedPointFromFronitiers[i], radiusPix, Scalar(0));        
            }

            switch (explore_state_){

                
                case NAV_TO_NEXT_FRONTIER:
                {   

                    cerr<<"NAV_TO_NEXT_FRONTIER "<<endl;

                    float mapScore = 0.0;

                    auto robotPix = convertPoseToPix(robotPose_);

                    std::vector<Frontier> currentEdgesFrontiers;
                    mapScore = goalCalculator.calcEdgesFrontiers(explorationImgMap,
                                          currentEdgesFrontiers, robotPix, mapResolution_);

                    cerr<<"map exploration score: "<<mapScore<<endl;
                    

                    if( currentEdgesFrontiers.size() == 0){

                        explore_state_ = FINISH_EXPLORE;
                        break;
                    }


                    geometry_msgs::Quaternion q;
                    q.w = 1;
                    auto nextFrontierGoal = convertPixToPose(currentEdgesFrontiers[0].center, q);                               

                    
                    publishEdgesFrontiers(currentEdgesFrontiers);
                    
                    makeReverseIfNeeded();

                    bool result = sendGoal(nextFrontierGoal);

                    if( !result) {

                        unreachedPointFromFronitiers.push_back(currentEdgesFrontiers[0].center);
                    }


                    cerr<<"move_base result for NAV_TO_NEXT_FRONTIER "<<result<<endl;

                    
                    explore_state_ = NAV_TO_NEXT_FRONTIER;

                    break;
                }
                case ERROR_EXPLORE:
                {
                    node_.setParam("/coverage/state", "STOPPED");

                    cerr<<"ERROR_EXPLORE "<<endl;
                    break;
                }
                case FINISH_EXPLORE:
                {                  
     
                    
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
                
                case COVERAGE:
                {
                    cerr << " COVERAGE " << endl;

                    currentAlgoMap_ = getCurrentMap();

                    // calculate goal-distance-transform-map
                    cv::Mat distanceTransformImg;

                    auto currentPosition = convertPoseToPix(startingLocation_);

                    cerr<<" currentPosition "<<currentPosition<<endl;


                    cv::Point2d goal = currentPosition;


                    // // calc the distance-transform-img from current goal
                    if( !distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_,
                        currentPosition, distanceTransformImg, 1)){

                        cerr<<" failed to calcutate the disntace transform img"<<endl;   
                        coverage_state_ = ERROR_COVERAGE;
                        break;
 
                    }

                    Mat grayDistImg;
                    distanceTransformGoalCalculator.normalizeDistanceTransform(distanceTransformImg, grayDistImg);


                    // Mat dbg = mapping_map_.clone();
                    // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

                    // calc the path-coverage of the current blob
                    
                    cerr<<" cccccccccccccccccc currentPosition for patthern "<<currentPosition<<endl;
                    auto path =
                        disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition,
                                                            goal, distanceTransformImg, getDistanceBetweenGoalsPix());


                    // convert the path into poses
                    path_poses_with_status_.setPixelsPath(path);
                    path_poses_with_status_.coveragePathPoses_ = covertPointsPathToPoseRout(path);
                    path_poses_with_status_.initStatusList();

                    // exectute currnet navigation the blob-coverage
                    cerr <<"cccccccccccc num of coverage waypoints " << 
                        path_poses_with_status_.coveragePathPoses_.size() <<" path_ size is "<<path.size()<<endl;
                    
                    for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
                    {   

                        ros::spinOnce();

                        updateRobotLocation();

                        percentCoverage_ = getCurrentPercentCoverage();
                        node_.setParam("/coverage/percentage", percentCoverage_);    
                        
                        publishCoveragePath(path_poses_with_status_.coveragePathPoses_);


                        publishRobotHistoryPath();

                        // the waypoint is checked (black)
                        if( path_poses_with_status_.status_[i] != UN_COVERED ){
                            continue;
                        }

                        // set the orientation of the goal dynamically
                        
                        makeReverseIfNeeded();
                        
                        ros::spinOnce();
                        

                        bool result = sendGoal(path_poses_with_status_.coveragePathPoses_[i]);

                        // set way[oint as checked
                        path_poses_with_status_.setStatByIndex(i, COVERED );


                        if( exit_){

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
                    cerr<<" BACK_TO_STARTING_LOCATION "<<endl; 

                    if( countNumberOfTriesFinsihGoal > 3) {

                        coverage_state_ = COVERAGE_DONE;  

                        break; 
                    }  
            
                    cerr<<"startingLocation_ : "<<startingLocation_.pose.position.x<<", "<<startingLocation_.pose.position.y<<endl;
                    cerr<<"startingLocation_  frame: "<<startingLocation_.header.frame_id<<endl;


                    bool result = sendGoal(startingLocation_);

                    if (result)
                    {
                        cerr <<"BACK_TO_STARTING_LOCATION reached!" << endl;

                        coverage_state_ = COVERAGE_DONE;  

                        break;

                    }
                    else
                    {
                        cerr <<" Failed to reach BACK_TO_STARTING_LOCATION, try again" << endl;

                        coverage_state_ = BACK_TO_STARTING_LOCATION; 
                        
                        countNumberOfTriesFinsihGoal++;


                        break;
                    }

                    
                }
                case COVERAGE_DONE:
                {
                    cerr<<" COVERAGE_DONE "<<endl;  

                    node_.setParam("/coverage/state", "STOPPED");
                    
                    saveCoverageImg();

                    coverage_state_ = COVERAGE_DONE;
                    break;
                }
                
                case ERROR_COVERAGE:
                {   

                    if(!errCoverage_)
                    {
                        cerr<<" ERROR_COVERAGE "<<endl; 
                        errCoverage_ = true; 

                    }
                    
                    node_.setParam("/coverage/state", "STOPPED");

                    coverage_state_ = ERROR_COVERAGE;
                    break;
                }
            }


        }

       
    }   

    float getCurrentPercentCoverage() {

        if ( path_poses_with_status_.coveragePathPoses_.size() == 0 ){

            return 0.0;
        }

        float countCovered = 0.0;

        float countNeedToBeCovered = 0.0;
 
        for(int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++ ) {  
            switch (path_poses_with_status_.status_[i])
            {
                case UN_COVERED:
                {
                    countNeedToBeCovered += 1.0;
                }
                case COVERED:
                {
                    countNeedToBeCovered += 1.0;
                    countCovered += 1.0;

                }
                case COVERED_BY_ROBOT_PATH:
                {
                    countNeedToBeCovered += 1.0;
                    countCovered += 1.0; 

                }  
                case COVERED_BY_OBSTACLE:
                {
                    

                } 
                
            }
            

        }

        float percentCoverage = ( countCovered / countNeedToBeCovered)  * 100.0;

        if( percentCoverage > 100){

            percentCoverage = 100.0;
        }

        return percentCoverage;

    }

    bool makeReverseIfNeeded() {


        if( !updateRobotLocation()){
            return false;
        }

        if( !costMapImg_.data){

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

            float robotHeading  = atan2((2.0 *
                (robotOdomPose.pose.orientation.w * robotOdomPose.pose.orientation.z + 
                robotOdomPose.pose.orientation.x * robotOdomPose.pose.orientation.y)),
                (1.0 - 2.0 * (robotOdomPose.pose.orientation.y * robotOdomPose.pose.orientation.y +
                 robotOdomPose.pose.orientation.z * robotOdomPose.pose.orientation.z)));
            
            // convert odom pose to odom pix
            cv::Point2d robotPix = cv::Point2d( (robotOdomPose.pose.position.x - globalMapOriginPositionX_) / globalMapResolution_,
                (robotOdomPose.pose.position.y - globalMapOriginPositionY_) / globalMapResolution_ );


            // Mat dbg = costMapImg_.clone();
            // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

            float robotRPix = (1.0 / globalMapResolution_) * (robot_w_m_ / 2 );
            // circle(dbg, robotPix, robotRPix , Scalar(0,255,255), 1, 8, 0);

            float shiftFromCenter = 0.6;
            float zoneRPix = robotRPix + (1.0 / globalMapResolution_) * (shiftFromCenter / 2 );

            cv::Point2d robotHeadingPointFront(robotPix.x + ( zoneRPix ) * cos(robotHeading),
                                                    robotPix.y + ( zoneRPix ) * sin(robotHeading));

            cv::Point2d robotHeadingPointBack(robotPix.x + (zoneRPix) * -1* cos(robotHeading),
                                                    robotPix.y + (zoneRPix) * -1* sin(robotHeading));

            // cv::arrowedLine(dbg, robotPix, robotHeadingPointFront, Scalar(0, 255, 0), 2);

            // cv::arrowedLine(dbg, robotPix, robotHeadingPointBack, Scalar(0, 0, 255), 2);

            // circle(dbg, robotPix, zoneRPix , Scalar(255,0,255), 1, 8, 0);



            // check if robot blocked in front
            cv::LineIterator it_map_front(costMapImg_, robotPix, robotHeadingPointFront, 4);  // 4 more dense than 8
            
            bool blockedInFront = false;
            for (int j = 0; j < it_map_front.count; j++, ++it_map_front)
            {
                cv::Point2d pointBeam = it_map_front.pos();
                int valueTemp = costMapImg_.at<uchar>(pointBeam.y, pointBeam.x);
                if (valueTemp == 255)
                {
                    blockedInFront = true;
                    break;
                }
            }

            if( blockedInFront ) {

                // check if clear behind the robot

                cv::LineIterator it_map_back(costMapImg_, robotPix, robotHeadingPointBack, 4);  // 4 more dense than 8

                bool clearInBack = true;
                for (int j = 0; j < it_map_back.count; j++, ++it_map_back)
                {
                    cv::Point2d pointBeam = it_map_back.pos();
                    int valueTemp = costMapImg_.at<uchar>(pointBeam.y, pointBeam.x);
                    if (valueTemp == 255)
                    {
                        clearInBack = false;
                        break;
                    }
                }

                if ( clearInBack ){

                    //we can publish reverse

                    ros::Rate rate(1);
                    float maxReversDuration = 2.0;

                    geometry_msgs::Twist velocity;
                    velocity.linear.x = -0.2;

                    auto startTime = ros::WallTime::now();

                    while (ros::ok()) {

                        cerr<<" REEEEEEVERSE !!!!!!!!!! "<<endl;
                        reverse_cmd_vel_pub_.publish(velocity);
                        rate.sleep();

                        auto end = ros::WallTime::now();

                        auto duration = (end - startTime).toSec();
                        if( duration > maxReversDuration){
                            break;
                        }
                    
                        ros::spinOnce();
                    }

                    ros::Duration(0.1).sleep();

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

        // try
        // {
        //     auto point1 = currentRobotFootPrintLocationOodm_.polygon.points[0];
        //     auto point2 = currentRobotFootPrintLocationOodm_.polygon.points[1];
        //     auto point3 = currentRobotFootPrintLocationOodm_.polygon.points[2];


        //     // convert odom pose to odom pix
        //     cv::Point2d pix1 = cv::Point2d( (point1.x - globalMapOriginPositionX_) / globalMapResolution_,
        //         (point1.y - globalMapOriginPositionY_) / globalMapResolution_ );

        //     cv::Point2d pix2 = cv::Point2d( (point2.x - globalMapOriginPositionX_) / globalMapResolution_,
        //         (point2.y - globalMapOriginPositionY_) / globalMapResolution_ );

        //     cv::Point2d pix3 = cv::Point2d( (point3.x - globalMapOriginPositionX_) / globalMapResolution_,
        //         (point3.y - globalMapOriginPositionY_) / globalMapResolution_ );
            
        //     cerr<<" pix1 "<<pix1<<endl;
        //     cerr<<" pix2 "<<pix2<<endl;
        //     cerr<<" pix3 "<<pix3<<endl;

        //     cerr<<"------------------------------ "<<endl;


        //     cv::RotatedRect robotRotateRect;
            
        //     try
        //     {
        //         robotRotateRect = cv::RotatedRect(pix1, pix2 , pix3); 
        //     }
        //     catch( cv::Exception& e )
        //     {
        //         const char* err_msg = e.what();
        //         std::cout << "11111111 exception caught: " << err_msg << std::endl;

        //         return false;
        //     }

        //     // We take the edges that OpenCV calculated for us
        //     // cv::Point2f vertices2f[4];
        //     // robotRotateRect.points(vertices2f);

        //     // Convert them so we can use them in a fillConvexPoly
        //     // cv::Point vertices[4];    
        //     // for(int i = 0; i < 4; ++i){
        //     //     vertices[i] = vertices2f[i];
        //     // }

        //     // Mat dbg = costMapImg_.clone();
        //     // cvtColor(dbg, dbg, COLOR_GRAY2BGR);


        //     // for (int i = 0; i < 4; i++)
        //     //     line(dbg, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 1);

        //     // float angle = robotRotateRect.angle * M_PI / 180.0;
        //     // // angle += M_PI; // you may want rotate it upsidedown
        //     // float sinA = sin(angle), cosA = cos(angle);
        //     // float data[6] = {
        //     //     cosA, sinA, robotRotateRect.size.width/2.0f - cosA * robotRotateRect.center.x - sinA * robotRotateRect.center.y,
        //     //     -sinA, cosA, robotRotateRect.size.height/2.0f - cosA * robotRotateRect.center.y + sinA * robotRotateRect.center.x};
        //     // Mat rot_mat(2, 3, CV_32FC1, data);
        //     // Mat result;

        //     Mat roiImg =  costMapImg_(robotRotateRect.boundingRect()).clone();

        //     // warpAffine(roiImg, result, rot_mat, robotRotateRect.size, INTER_CUBIC);

        //     bool needToDoRevers = false;
        //     for (int j = 0; j < roiImg.rows; j++)
        //     {
        //         for (int i = 0; i < roiImg.cols; i++)
        //         {

        //             auto value = roiImg.at<uchar>(j , i);

        //             if( value == 255){

        //                 needToDoRevers = true;
        //                 break;
        //             }
                    
        //         }

        //         if( needToDoRevers){
        //             break;
        //         }
        //     }

        //     if ( needToDoRevers ){

        //         ros::Rate rate(1);
        //         float maxReversDuration = 2.0;

        //         geometry_msgs::Twist velocity;
        //         velocity.linear.x = -0.2;

        //         auto startTime = ros::WallTime::now();

        //         while (ros::ok()) {

        //             cerr<<" REEEEEEVERSE !!!!!!!!!! "<<endl;
        //             reverse_cmd_vel_pub_.publish(velocity);
        //             rate.sleep();

        //             auto end = ros::WallTime::now();

        //             auto duration = (end - startTime).toSec();
        //             if( duration > maxReversDuration){
        //                 break;
        //             }
                
        //             ros::spinOnce();
        //         }

        //         ros::Duration(0.1).sleep();

        //         return true;


        //     }       

        //     // imwrite("/home/yakir/distance_transform_coverage_ws/bags/roiImg.png", roiImg);

        //     return false;
        // }
        // catch( cv::Exception& e )
        // {
        //     const char* err_msg = e.what();
        //     std::cout << "exception caught: " << err_msg << std::endl;
        //     return false;


        // }       
           
        
    }

    void setCoverageState(bool coverageState){

        coverageStateStarts_ = coverageState;
    }

    static void mySigintHandler(int sig, void *ptr)
    {   

        cerr<<" user pressed CTRL+C "<<endl;
        exit_ = true;


    } 


private:

    void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) {

        gotFootPrint_ = true;
        currentRobotFootPrintLocationOodm_.polygon = msg->polygon;
        currentRobotFootPrintLocationOodm_.header = msg->header;

    
			

    }

    

    


    void publishRobotHistoryPath() {

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

            tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut );


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

    

    
    void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {

        globalMapOriginPositionX_ = msg->info.origin.position.x;
        globalMapOriginPositionY_ = msg->info.origin.position.y; 
        globalMapResolution_ = msg->info.resolution;

        auto endLocalCostMap = high_resolution_clock::now();
        auto durationFromLastCalc = duration_cast<seconds>(endLocalCostMap - startLocalCostMap_).count();

        /// do this every 2 seconds
        if( init_ && durationFromLastCalc > 2.0 && coverageStateStarts_) {

            costMapImg_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1, Scalar(0));
            memcpy(costMapImg_.data, msg->data.data(), msg->info.height * msg->info.width);
            
            costMapImg_.setTo(255, costMapImg_ != 0);

            string global_costmap_frame = msg->header.frame_id;

            // Mat dbg = costMapImg.clone();
            // cvtColor(dbg, dbg, COLOR_GRAY2BGR);

            for(int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++ ){

                // if this goal cant be n obstacle
                if( path_poses_with_status_.status_[i] == COVERED || 
                    path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH ){
                    
                    continue;
                }

                // transform to odom frame (global costmap framme)
                cv::Point3d p = cv::Point3d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x, 
                    path_poses_with_status_.coveragePathPoses_[i].pose.position.y, 0);


                auto poseInOdomFrame = transformFrames(p, global_costmap_frame , globalFrame_ ,msg->header.stamp);

              

                // convert odom pose to odom pix
                float xPix = (poseInOdomFrame.point.x - globalMapOriginPositionX_) / globalMapResolution_;
                float yPix = (poseInOdomFrame.point.y - globalMapOriginPositionY_) / globalMapResolution_;

                cv::Point pOnImg = cv::Point(xPix, yPix);
                
                // get the cost value
                int costVal = costMapImg_.at<uchar>(pOnImg.y, pOnImg.x);

                float distRobotFromGoal = 
                     goalCalculator.distanceCalculate( cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                         cv::Point2d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                             path_poses_with_status_.coveragePathPoses_[i].pose.position.y));
                

                //GOAL ON obstacle
                if( costVal != 0 ){     


                    // the goal inside the wanted radius
                    if( distRobotFromGoal < 2.0 ){

                        path_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);

                    } else {

                        /// goal outside the raius, make it uncovered
                        path_poses_with_status_.setStatByIndex(i, UN_COVERED);

                    }                  

                    // circle(dbg, pOnImg,  1, Scalar(0,255,0), -1, 8, 0);

                } /// goal not inside obstacle 
                else {                   


                    // if inside radius but last time int was inside obstacle,
                    // keep it as obstacle
                    if( distRobotFromGoal < 2.0 &&  
                         path_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE) {


                        path_poses_with_status_.setStatByIndex(i, COVERED_BY_OBSTACLE);
    
                            
                    } else {

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
            {                double old_origin_x = map_origin_position_x;
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


        addDilationForGlobalMap(currentGlobalMap_, walls_inflation_m_, mapResolution_);

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

            robotHistoryPathMsg_.poses.push_back(robotPose_);


            return true;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

    

    void publishEdgesFrontiers(const std::vector<Frontier>& currentEdgesFrontiers)
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


    void publishWaypointsWithStatus() {

        visualization_msgs::MarkerArray Markerarr;
        int count = 1;

        for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++)
        {
                visualization_msgs::Marker m;
                m.header.frame_id = globalFrame_;
                m.header.stamp = ros::Time::now();
                m.ns = "points_and_lines";
                m.id =  count + i;
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

                //checked             


                /// COVERED = BLACK
                if( path_poses_with_status_.status_[i] == COVERED){

                    m.color.r = 0;
                    m.color.g = 0.0;
                    m.color.b = 0.0;

                }  /// UN_COVERED = GREEN 
                else if( path_poses_with_status_.status_[i] == UN_COVERED){

                    m.color.r = 0.0;
                    m.color.g = 1.0;
                    m.color.b = 0.0;

                }   /// COVERED_BY_ROBOT_PATH = LIGHT BLUE 
                else if( path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH){

                    m.color.r = 0.0;
                    m.color.g = 1.0;
                    m.color.b = 1.0;
                }   /// COVERED_BY_OBSTACLE = RED 

                else if( path_poses_with_status_.status_[i] == COVERED_BY_OBSTACLE){

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
        geometry_msgs::Quaternion q )
    {

        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = globalFrame_;

        pose.pose.position.x = (pixel.x * mapResolution_) + map_origin_position_x;
        pose.pose.position.y = (pixel.y * mapResolution_) + map_origin_position_y;
        pose.pose.position.z = 0.0;

        if( q.w != 0.0){

            pose.pose.orientation.w = q.w;
            pose.pose.orientation.x = q.x;
            pose.pose.orientation.y = q.y;
            pose.pose.orientation.z = q.z;


        } else {
            
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

                
                if(  reducing_goals_) {

                    if (angle != prevAngle)
                    {

                        prevAngle = angle;
                        posesPath.push_back(pose);
                    }
                } else {

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

   

    cv::Point fixLocationOnGrid(const cv::Point &goal, const cv::Point& ref)
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


    

    void removeGoalsByRobotRout() {


        for (int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++) {

            /// if the goal already covered 
            if( path_poses_with_status_.status_[i] == COVERED_BY_ROBOT_PATH ) {
                continue;
            }

            float distRobotFromGoal = 
                    goalCalculator.distanceCalculate(cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                        cv::Point2d(path_poses_with_status_.coveragePathPoses_[i].pose.position.x,
                            path_poses_with_status_.coveragePathPoses_[i].pose.position.y));
        
            if ( distRobotFromGoal < robot_w_m_ / 2.0) {
            
                path_poses_with_status_.setStatByIndex(i, COVERED_BY_ROBOT_PATH);
            }        

        }
    }

   

    string getMoveBaseState(actionlib::SimpleClientGoalState state) {
        
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

    bool  sendGoal(const geometry_msgs::PoseStamped &goalMsg)
    {      

        //navigate to the point			
        moveBaseController_.navigate(goalMsg);

        bool result = true;

      
        while(ros::ok()) {

            ros::spinOnce();

            updateRobotLocation();

            removeGoalsByRobotRout();

            publishRobotHistoryPath();

            publishWaypointsWithStatus();


            if( exit_){

                return true;
            }

          

            moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
            auto move_base_state = moveBaseController_.moveBaseClient_.getState();

            string strState = getMoveBaseState(move_base_state);            



            if( move_base_state == actionlib::SimpleClientGoalState::ACTIVE 
                ||  move_base_state == actionlib::SimpleClientGoalState::PENDING)
            { 
                continue;
            }    
            
            if( move_base_state == actionlib::SimpleClientGoalState::SUCCEEDED){                

                cerr<<"strState:  "<<strState<<endl;

                result = true;
                break;
            }
            else
            {   
                cerr<<"strState:  "<<strState<<endl;

                result = false;
                break;
            } 
         

            ros::spinOnce();
        }  

        

        return result;
    }


    
    void saveCoverageImg(){

        if( !imgSaved_ && mappingMap_.data){

            Mat patternImg = mappingMap_.clone();
            Mat robotTreaceImg = mappingMap_.clone();

            cvtColor(patternImg, patternImg, COLOR_GRAY2BGR);
            cvtColor(robotTreaceImg, robotTreaceImg, COLOR_GRAY2BGR);


            // draw the pattern
            for( int i = 0; i < path_poses_with_status_.path_.size(); i++){

                if( i > 0 ){
                    cv::line(patternImg, path_poses_with_status_.path_[i], 
                        path_poses_with_status_.path_[i - 1], Scalar(34, 139, 139), 1);
                }             
            }

            // draw the robot trace (INCLUDE DIMS)
            for( int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++){


                cv::Point p1 = convertPoseToPix(robotHistoryPathMsg_.poses[i]);
                cv::Point p2 = convertPoseToPix(robotHistoryPathMsg_.poses[i+1]);

                cv::line(robotTreaceImg, p1, p2, Scalar(226, 43, 138), robot_w_m_ /  mapResolution_);      
            }
            // draw the trace only
            for( int i = 0; i < robotHistoryPathMsg_.poses.size() - 1; i++){


                cv::Point p1 = convertPoseToPix(robotHistoryPathMsg_.poses[i]);
                cv::Point p2 = convertPoseToPix(robotHistoryPathMsg_.poses[i+1]);

                cv::line(robotTreaceImg, p1, p2, Scalar(0, 255, 0), 1);
     
            }

            // draw the grid
            for(int i = 0; i < path_poses_with_status_.coveragePathPoses_.size(); i++ ) {  
                switch (path_poses_with_status_.status_[i])
                {
                    case UN_COVERED:
                    {
                        circle(robotTreaceImg, path_poses_with_status_.path_[i], 2, Scalar(0,255,0), -1, 8, 0);
                        break;

                    }
                    case COVERED:
                    {
                        circle(robotTreaceImg, path_poses_with_status_.path_[i], 2, Scalar(0,0,0), -1, 8, 0);
                        break;

                    }
                    case COVERED_BY_ROBOT_PATH:
                    {
                        circle(robotTreaceImg, path_poses_with_status_.path_[i], 2, Scalar(250,206,135), -1, 8, 0);
                        break; 

                    }  
                     case COVERED_BY_OBSTACLE:
                    {
                        circle(robotTreaceImg, path_poses_with_status_.path_[i], 2, Scalar(0,0,255), -1, 8, 0);
                        break; 

                    }  
                   
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
            string image_name_format = startingTime_ + '_' +to_string(durationMinutes)+ '_' + to_string(int(percentCoverage_));
            string full_img_name = coverage_img_path_ + image_name_format+".png";
            node_.setParam("/coverage/image_name", image_name_format);

            cerr<<"full_img_name: "<<full_img_name<<endl;

            cv::imwrite(full_img_name, robotTreaceImg);
            cv::imwrite(coverage_img_path_+"patthern.png", patternImg);


            imgSaved_ = true;
        }
    }

private:
    COVERAGE_STATE coverage_state_ = COVERAGE_STATE::COVERAGE;
    bool coverageStateStarts_ = false;

    EXPLORE_STATE explore_state_ = EXPLORE_STATE::NAV_TO_NEXT_FRONTIER;

    // move-base
    MoveBaseController moveBaseController_;

    // subs
    ros::Subscriber global_map_sub_;

    ros::Subscriber global_cost_map_sub_;

    ros::Subscriber camera_scan_sub_;

    ros::Subscriber robot_footprint_sub_;

    // pubs

    ros::Publisher reverse_cmd_vel_pub_;

    ros::Publisher cuurentCoveragePathPub_;

    ros::Publisher edges_frontires_marker_array_Pub;

    ros::Publisher waypoints_with_status_pub_;

    image_transport::Publisher  coverage_map_pub_;

    ros::Publisher robot_history_path_pub_;
    nav_msgs::Path robotHistoryPathMsg_;



    //classes 

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
    float  globalMapOriginPositionX_ = -1;
    float  globalMapOriginPositionY_ = -1;
    float  globalMapResolution_ = -1;
    cv::Mat costMapImg_;


    bool init_ = false;

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

    double radius_for_cleaning_route_goals_ = 0.2;

    double wanted_coverage_score_ = 0.95;

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

    // while (ros::ok()) {

    // mapCoverageManager.setCoverageState(true);


    //     ros::spinOnce();

    //     mapCoverageManager.makeReverseIfNeeded();

    // }



    

    ros::spin();


    return 0;
}




// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "map_coverage_exploration_node");

//     DistanceTransformGoalCalculator distanceTransformGoalCalculator;

//     cerr<<"2222222222222222222222222222222222 " <<endl;

//     float mapResolution_ = 0.05;
//     float distBetweenGoalsM_ = 0.3;

//     float robotWidthM = 0.53;
//     float robotHeightM = 0.53;
//     float robotWidthPix = (1.0 / mapResolution_) * robotWidthM;
//     float robotHeightPix = (1.0 / mapResolution_) * robotHeightM;

//     DisantanceMapCoverage disantanceMapCoverage(false);
//     GoalCalculator goalCalculator;

//     disantanceMapCoverage.setRectFreeSapceDim(robotWidthPix, robotHeightPix);

//     cv::Point2d currentPosition(277, 236);



    
//     int pixDist = (1.0 / mapResolution_) * distBetweenGoalsM_;
//     float walls_inflation_m_ = 0.3;

//     Mat currentAlgoMap_ = imread("/home/yakir/distance_transform_coverage_ws/bugs/1/map.pgm",0);

//     cv::flip(currentAlgoMap_, currentAlgoMap_, 0);
//     Mat mappingMap = currentAlgoMap_.clone();  
    
//     // cerr<<"111111111111111111111111111111111111 " <<endl;
    
//     // // circle(currentAlgoMap_, currentPosition, 2, Scalar(150), -1, 8, 0);
//     // imwrite("/home/yakir/distance_transform_coverage_ws/bugs/1/dbg.pgm", currentAlgoMap_);
//     // return 0;
   
//     addDilationForGlobalMap(currentAlgoMap_, walls_inflation_m_, mapResolution_);
//     addFreeSpaceDilation(currentAlgoMap_);

//     if( false) {
        
//         cerr<<"yakir "<<endl;
//         cv::Point globalStart_(380,207);
//         cv::Point safestGoal;

             
//         imshow("currentAlgoMap_", currentAlgoMap_);
//         waitKey(0);
//         if (!goalCalculator.findSafestLocation(currentAlgoMap_, globalStart_, safestGoal))
//         {

//             return -1;
//         }

//         return 0;
//     }

//     if( true) {


        
//         cv::Mat distanceTransformImg;


//         cv::Point2d goal = currentPosition;
//         // // calc the distance-transform-img from current goal
//         if( !distanceTransformGoalCalculator.calcDistanceTransfromImg(currentAlgoMap_,
//             currentPosition, distanceTransformImg, 1)){

//             cerr<<" failed to calcutate the disntace transform img"<<endl;   
//             return -1;

//         }

//         Mat grayDistImg;
//         distanceTransformGoalCalculator.normalizeDistanceTransform(distanceTransformImg, grayDistImg);

      
//         Mat dbg = mappingMap.clone();
//         cvtColor(dbg, dbg, COLOR_GRAY2BGR);

//         // calc the path-coverage of the current blob

//         vector<cv::Point> path =
//             disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition,
//                                                 goal, distanceTransformImg, pixDist);          



//         for( int i = 0; i < path.size(); i++){


//             if( i > 0 ){
//                 cv::line(dbg, path[i], path[i - 1], Scalar(34, 139, 139), 2);
//             }

//         }     

//         circle(dbg, goal, 2, Scalar(0,255,0), -1, 8, 0);
//         circle(dbg, currentPosition, 2, Scalar(0,0,255), -1, 8, 0);  

//         // imwrite("/home/yakir/distance_transform_coverage_ws/dbg.png", dbg);
//         imshow("dbg2",dbg);
//         // imshow("distanceTransformImg", grayDistImg);
//         waitKey(0);
//     }

    
                    

//     return 0;
// }




// class Test
// {

// public:
//     Test()
//     {      
     

//           // subs
//         global_cost_map_sub_ =
//             node_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1,
//                      &Test::globalCostMapCallback, this);    
        
        
//         robot_footprint_sub_ =
//             node_.subscribe<geometry_msgs::PolygonStamped>("/move_base/local_costmap/footprint", 1,
//                                                      &Test::footprintCallback, this); 


//         reverse_cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>(		
//         	"/cmd_vel", 1, false);	      

//         startLocalCostMap_ = high_resolution_clock::now();
//     }                                            


//     ~Test(){}

//     bool makeReverseIfNeeded() {

//         ros::spinOnce();


//         if( !updateRobotLocation()){
//             return false;
//         }

//         if( !costMapImg_.data){
//             return false;
//         }

//         if( ! gotFootPrint_){
//             return false;
//         }
//         cerr<<"1111111111111111111111111111111111111111111 "<<endl;




//         auto point1 = currentRobotFootPrintLocationOodm_.polygon.points[0];
//         auto point2 = currentRobotFootPrintLocationOodm_.polygon.points[1];
//         auto point3 = currentRobotFootPrintLocationOodm_.polygon.points[2];


//             // convert odom pose to odom pix
//         cv::Point2d pix1 = cv::Point2d( (point1.x - globalMapOriginPositionX_) / globalMapResolution_,
//             (point1.y - globalMapOriginPositionY_) / globalMapResolution_ );

//         cv::Point2d pix2 = cv::Point2d( (point2.x - globalMapOriginPositionX_) / globalMapResolution_,
//             (point2.y - globalMapOriginPositionY_) / globalMapResolution_ );

//         cv::Point2d pix3 = cv::Point2d( (point3.x - globalMapOriginPositionX_) / globalMapResolution_,
//             (point3.y - globalMapOriginPositionY_) / globalMapResolution_ );
        
//         cv::RotatedRect robotRotateRect = cv::RotatedRect(pix1, pix2 , pix3); 

//         // We take the edges that OpenCV calculated for us
//         // cv::Point2f vertices2f[4];
//         // robotRotateRect.points(vertices2f);

//         // Convert them so we can use them in a fillConvexPoly
//         // cv::Point vertices[4];    
//         // for(int i = 0; i < 4; ++i){
//         //     vertices[i] = vertices2f[i];
//         // }

//         // Mat dbg = costMapImg_.clone();
//         // cvtColor(dbg, dbg, COLOR_GRAY2BGR);


//         // for (int i = 0; i < 4; i++)
//         //     line(dbg, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 1);

//         // float angle = robotRotateRect.angle * M_PI / 180.0;
//         // // angle += M_PI; // you may want rotate it upsidedown
//         // float sinA = sin(angle), cosA = cos(angle);
//         // float data[6] = {
//         //     cosA, sinA, robotRotateRect.size.width/2.0f - cosA * robotRotateRect.center.x - sinA * robotRotateRect.center.y,
//         //     -sinA, cosA, robotRotateRect.size.height/2.0f - cosA * robotRotateRect.center.y + sinA * robotRotateRect.center.x};
//         // Mat rot_mat(2, 3, CV_32FC1, data);
//         // Mat result;

//         Mat roiImg =  costMapImg_(robotRotateRect.boundingRect()).clone();

//         // warpAffine(roiImg, result, rot_mat, robotRotateRect.size, INTER_CUBIC);

//         bool needToDoRevers = false;
//         for (int j = 0; j < roiImg.rows; j++)
//         {
//             for (int i = 0; i < roiImg.cols; i++)
//             {

//                 auto value = roiImg.at<uchar>(j , i);

//                 if( value == 255){

//                     needToDoRevers = true;
//                     break;
//                 }
                
//             }

//             if( needToDoRevers){
//                 break;
//             }
//         }

//         if ( needToDoRevers ){

//             ros::Rate rate(1);
//             float maxReversDuration = 3.0;

//             geometry_msgs::Twist velocity;
//             velocity.linear.x = -0.3;

//             auto startTime = ros::WallTime::now();

//             while (ros::ok()) {

//                 reverse_cmd_vel_pub_.publish(velocity);
//                 rate.sleep();

//                 auto end = ros::WallTime::now();

//                 auto duration = (end - startTime).toSec();
//                 if( duration > maxReversDuration){
//                     break;
//                 }
              
//                 ros::spinOnce();
//             }

//             return true;


//         }       

//         // imwrite("/home/yakir/distance_transform_coverage_ws/bags/roiImg.png", roiImg);

//         return false;
           
        
//     }

// private:

//     void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) {

//         gotFootPrint_ = true;
//         currentRobotFootPrintLocationOodm_.polygon = msg->polygon;
//         currentRobotFootPrintLocationOodm_.header = msg->header;

    
			

//     }

//     void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {


//         globalMapOriginPositionX_ = msg->info.origin.position.x;
//         globalMapOriginPositionY_ = msg->info.origin.position.y; 
//         globalMapResolution_ = msg->info.resolution;

//         auto endLocalCostMap = high_resolution_clock::now();
//         auto durationFromLastCalc = duration_cast<seconds>(endLocalCostMap - startLocalCostMap_).count();

//         /// do this every 2 seconds
//         if( durationFromLastCalc > 2.0) {

            
//             costMapImg_ = cv::Mat(msg->info.height, msg->info.width, CV_8UC1, Scalar(0));
//             memcpy(costMapImg_.data, msg->data.data(), msg->info.height * msg->info.width);
            
//             costMapImg_.setTo(255, costMapImg_ != 0);


//             startLocalCostMap_ = endLocalCostMap;
//         }



//     }


     

//     bool updateRobotLocation()
//     {

//         tf::StampedTransform transform;

//         try
//         {
//             // get current robot pose
//             tfListener_.lookupTransform(globalFrame_, baseFrame_,
//                                         ros::Time(0), transform);

//             robotPose_.header.frame_id = globalFrame_;
//             robotPose_.header.stamp = ros::Time::now();
//             robotPose_.pose.position.x = transform.getOrigin().x();
//             robotPose_.pose.position.y = transform.getOrigin().y();
//             robotPose_.pose.position.z = 0;
//             robotPose_.pose.orientation.x = transform.getRotation().x();
//             robotPose_.pose.orientation.y = transform.getRotation().y();
//             robotPose_.pose.orientation.z = transform.getRotation().z();
//             robotPose_.pose.orientation.w = transform.getRotation().w();



//             return true;
//         }

//         catch (...)
//         {
//             cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
//             return false;
//         }
//     }


//      geometry_msgs::PointStamped transformFrames(
//         Point3d objectPoint3d, string target_frame, string source_Frame, ros::Time t)
//     {

//         geometry_msgs::PointStamped pointStampedIn;
//         geometry_msgs::PointStamped pointStampedOut;

//         pointStampedIn.header.frame_id = source_Frame;
//         pointStampedIn.header.stamp = t;
//         pointStampedIn.point.x = objectPoint3d.x;
//         pointStampedIn.point.y = objectPoint3d.y;
//         pointStampedIn.point.z = objectPoint3d.z;

//         try
//         {
//             tf::StampedTransform transform_;   

//             tfListener_.lookupTransform(target_frame, source_Frame,
//                                         ros::Time(0), transform_);

//             tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut );


//             return pointStampedOut;
//         }
//         catch (tf::TransformException ex)
//         {
//             ROS_ERROR("%s", ex.what());            
//         }
//     }


    




// private:


//     ros::Publisher reverse_cmd_vel_pub_;

//     string globalFrame_ = "map";

//     string baseFrame_ = "base_footprint";

//     high_resolution_clock::time_point startLocalCostMap_;

//     tf::TransformListener tfListener_;

//     geometry_msgs::PoseStamped robotPose_;

//     ros::Subscriber global_cost_map_sub_;

//     ros::Subscriber robot_footprint_sub_;

//     ros::NodeHandle node_;


//     geometry_msgs::PolygonStamped currentRobotFootPrintLocationOodm_;


//      // global cost map params
//     float  globalMapOriginPositionX_ = -1;
//     float  globalMapOriginPositionY_ = -1;
//     float  globalMapResolution_ = -1;
//     cv::Mat costMapImg_;
//     bool gotFootPrint_ = false;





// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "map_coverage_exploration_node" , ros::init_options::NoSigintHandler);

//     Test test;
//     test.makeReverseIfNeeded();


   


//     return 0;
// }
