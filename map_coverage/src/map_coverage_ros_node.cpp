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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

    cerr<<curr_time_Str<<endl;

    return curr_time_Str;    
}


void addDilationForGlobalMap(Mat &imgMap, float robot_radius_meters_, float mapResolution)
{

    try
    {
        int dilationPix = (1.0 / mapResolution) * (robot_radius_meters_ * 2);        

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
        moveBaseController_.waitForServer(ros::Duration(10.0));
        ros::Duration(1).sleep();
        cerr << " connect to move-base !! " << endl;

        // rosparam
        ros::NodeHandle nodePrivate("~");
        nodePrivate.param("distance_between_goals_m", distBetweenGoalsM_, 0.5);
        nodePrivate.param("robot_raduis", robot_radius_meters_, 0.3);
        nodePrivate.param("exploration_score", nim_map_score_to_finish_exploration_, 70.0);
        nodePrivate.param("duration_wait_for_move_base_response", duration_wait_for_move_base_response_, 15.0);
        nodePrivate.param<string>("coverage_image_path", coverage_img_path_, string(""));  
        nodePrivate.param<string>("base_frame", baseFrame_, string("base_link"));  
        nodePrivate.param<string>("global_frame", globalFrame_, string("map"));  

        nodePrivate.param("/coverage/percentage", percentCoverage_, 0.0);    
        nodePrivate.param("/coverage/state", state_, string("INITIALIZING"));
        nodePrivate.param("/coverage/image_name", image_name_, string(""));  



        cerr<<"base_frame "<<baseFrame_<<endl;

        cerr<<"coverage_img_path_ "<<coverage_img_path_<<endl;


        
       

        // subs
        global_map_sub_ =
            node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                     &MapCoverageManager::globalMapCallback, this);

        // pubs
        cuurentCoveragePathPub_ = node_.advertise<nav_msgs::Path>(
            "/coverage_path", 1, false);   

        safest_goal_marker_pub_  = node_.advertise<visualization_msgs::Marker>("/safest_goal", 10);  

        edges_frontires_marker_array_Pub =
            node_.advertise<visualization_msgs::MarkerArray>("/edges_frontiers_marker_arr", 10);

        coverage_waypoints_marker_array_Pub =
            node_.advertise<visualization_msgs::MarkerArray>("/coverage_waypoints_marker_array", 10);

        global_start_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/global_start", 10);

        start_coverag_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/start_coverage", 10);

        goal_coverag_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/goal_coverage", 10);

        start_a_star_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/start_a_star", 10);

        goal_a_star_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/goal_a_star", 10);

        image_transport::ImageTransport it(node_);
        coverage_map_pub_ = it.advertise("/coverage_map_img", 1);
        nodePrivate.param("/coverage_map_img/compressed/jpeg_quality", 20);

        init_ = false;

        /// params
        mapResolution_ = -1;
        map_origin_position_x = -1;
        map_origin_position_y = -1;

        startingCoverageTime_ = high_resolution_clock::now();



    }

    ~MapCoverageManager() {}

    

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

    void publishCoverageImgMap() {


        Mat map = getCurrentMap();

        if (map.data) {

            cvtColor(map, map, COLOR_GRAY2BGR);

            // draw the path
            for( int i = 0; i < path_.size(); i++){

                if( i > 0 ){
                    cv::line(map, path_[i], path_[i - 1], Scalar(34, 139, 139), 2);
                }             
            }

            
            auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map).toImageMsg();
            coverage_map_pub_.publish(msg);
        }

           
    }

    void publishStartConverage(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 1;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 1.0;
        robotMarker.color.g = 0.0;
        robotMarker.color.b = 0.0;

        start_coverag_marker_pub_.publish(robotMarker);
    }

    void publishStartAstar(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 1;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 0.0;
        robotMarker.color.g = 0.0;
        robotMarker.color.b = 0.5;

        start_a_star_marker_pub_.publish(robotMarker);
    }

    void publishSafestGoalMarker(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 1;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 0.2;
        robotMarker.color.g = 0.8;
        robotMarker.color.b = 1.0;
        robotMarker.lifetime = ros::Duration(200);

       safest_goal_marker_pub_.publish(robotMarker);
    }

    void publishGlobalStartMarker(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 1;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 0.0;
        robotMarker.color.g = 1.0;
        robotMarker.color.b = 1.0;
        robotMarker.lifetime = ros::Duration(0);

        global_start_marker_pub_.publish(robotMarker);
    }

    void publishGoalAstar(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 1;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 0.0;
        robotMarker.color.g = 0.0;
        robotMarker.color.b = 1.0;

        goal_a_star_marker_pub_.publish(robotMarker);
    }

    void publishGoalConverage(const geometry_msgs::PoseStamped &pose)
    {

        visualization_msgs::Marker robotMarker;
        robotMarker.header.frame_id = globalFrame_;
        robotMarker.header.stamp = ros::Time::now();
        robotMarker.ns = "points_and_lines";
        robotMarker.id = 111111;
        robotMarker.action = visualization_msgs::Marker::ADD;
        robotMarker.type = visualization_msgs::Marker::SPHERE;
        robotMarker.pose.position = pose.pose.position;
        robotMarker.scale.x = 0.3;
        robotMarker.scale.y = 0.3;
        robotMarker.scale.z = 0.3;
        robotMarker.color.a = 1.0;
        robotMarker.color.r = 0.0;
        robotMarker.color.g = 1.0;
        robotMarker.color.b = 0.0;

        goal_coverag_marker_pub_.publish(robotMarker);
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

        mapping_map_ = currentGlobalMap_.clone();


        addDilationForGlobalMap(currentGlobalMap_, robot_radius_meters_, mapResolution_);

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

            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();

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

    void publishCoverageWayPoints(const vector<geometry_msgs::PoseStamped> &coveragePathPoses,
                                  int courrentIndex)
    {

        visualization_msgs::MarkerArray Markerarr;

        for (int i = 0; i < coveragePathPoses.size(); i++)
        {

            geometry_msgs::PoseStamped p = coveragePathPoses[i];

            visualization_msgs::Marker pOnEgge;
            pOnEgge.header.frame_id = globalFrame_;
            pOnEgge.header.stamp = ros::Time::now();
            pOnEgge.ns = "points_and_lines";
            pOnEgge.id = i + 1;
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
            if (i < courrentIndex)
            {

                pOnEgge.color.r = 0;
                pOnEgge.color.g = 1.0;
                pOnEgge.color.b = 0;
            }
            else
            {

                pOnEgge.color.r = 1.0;
                pOnEgge.color.g = 0;
                pOnEgge.color.b = 0;
            }

            Markerarr.markers.push_back(pOnEgge);
        }

        coverage_waypoints_marker_array_Pub.publish(Markerarr);
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

                if (angle != prevAngle)
                {

                    prevAngle = angle;
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


    bool exlpore()
    {
        string m = "";

        while (ros::ok())
        {
            ros::spinOnce();
           

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

            switch (explore_state_){

                case NAV_TO_SAFEST_GOAL:
                {   

                    node_.setParam("/coverage/state", "RUNNING");


                    cerr<<" NAV_TO_SAFEST_GOAL "<<endl;
                    currentAlgoMap_ = getCurrentMap();
                    globalStart_ = convertPoseToPix(robotPose_);

                    startingLocation_ = robotPose_;

                    startingTime_ = getCurrentTime();

                    cv::Point safestGoal;
                    if (!goalCalculator.findSafestLocation(currentAlgoMap_, globalStart_, safestGoal))
                    {

                        explore_state_ = ERROR_EXPLORE;
                        break;
                    }

                    safestGoal = fixLocationOnGrid(safestGoal, globalStart_);

                    publishSafestGoalMarker(convertPixToPose(safestGoal, robotPose_.pose.orientation));
                    publishSafestGoalMarker(convertPixToPose(safestGoal, robotPose_.pose.orientation));


                    cerr<<"exploration: safestGoal "<<safestGoal<<endl;

                    bool result = sendGoal(convertPixToPose(safestGoal, robotPose_.pose.orientation));  

                    cerr<<"exploration: mov_base_result "<<result<<endl;                            

                    explore_state_ = NAV_TO_NEXT_FRONTIER;

                    break;
                }
                case NAV_TO_NEXT_FRONTIER:
                {   

                    cerr<<"NAV_TO_NEXT_FRONTIER "<<endl;

                    float mapScore = 0.0;

                    currentAlgoMap_ = getCurrentMap();
                    auto robotPix = convertPoseToPix(robotPose_);

                    std::vector<Frontier> currentEdgesFrontiers;
                    mapScore = goalCalculator.calcEdgesFrontiers(currentAlgoMap_,
                                          currentEdgesFrontiers, robotPix);

                    cerr<<"map exploration score: "<<mapScore<<endl;
                    

                    if( currentEdgesFrontiers.size() < 2){

                        explore_state_ = FINISH_EXPLORE;
                        break;
                    }
                    

                    geometry_msgs::Quaternion q;
                    q.w = 1;
                    auto nextFrontierGoal = convertPixToPose(currentEdgesFrontiers[0].center, q);
                    publishSafestGoalMarker(nextFrontierGoal);
                    publishEdgesFrontiers(currentEdgesFrontiers);

                    bool result = sendGoal(nextFrontierGoal);

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

                    cerr<<"FINISH_EXPLORE "<<endl;

                    currentAlgoMap_ = getCurrentMap();
                    globalStart_ = convertPoseToPix(robotPose_);

                    cv::Point safestGoal;
                    if (!goalCalculator.findSafestLocation(currentAlgoMap_, globalStart_, safestGoal))
                    {
                        
                        cerr<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb "<<endl;
                        return true;
                    }

                    safestGoal = fixLocationOnGrid(safestGoal, globalStart_);

                    publishSafestGoalMarker(convertPixToPose(safestGoal, robotPose_.pose.orientation));
                    publishSafestGoalMarker(convertPixToPose(safestGoal, robotPose_.pose.orientation));


                    cerr<<"exploration FINISH_EXPLORE: safestGoal "<<safestGoal<<endl;

                    bool result = sendGoal(convertPixToPose(safestGoal, robotPose_.pose.orientation));  

                    cerr<<"exploration: mov_base_result "<<result<<endl;   

                    ros::Duration(1).sleep();
     
                    
                    return true;
                }

            }

        }
        
        return false;
    }

    void coverage()
    {
        while (ros::ok())
        {
            ros::spinOnce();

            if (coverage_state_ == COVERAGE_DONE)
            {

                return;
            }

            if (!init_)
            {

                continue;;
            }
            cerr << " map Recived !! " << endl;

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

                    auto currentPosition = convertPoseToPix(robotPose_);

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

                    path_ =
                        disantanceMapCoverage.getCoveragePath(currentAlgoMap_, currentPosition,
                                                            goal, distanceTransformImg, getDistanceBetweenGoalsPix(), true);          



                    // for( int i = 0; i < path_.size(); i++){


                    //     if( i > 0 ){
                    //         cv::line(dbg, path_[i], path_[i - 1], Scalar(34, 139, 139), 2);
                    //     }

                    // }     

                    // circle(dbg, goal, 2, Scalar(0,255,0), -1, 8, 0);
                    // circle(dbg, currentPosition, 2, Scalar(0,0,255), -1, 8, 0);   
                                    

                    // imwrite(ros::package::getPath("map_coverage") + "/dbg.png",dbg);
                    // imwrite(ros::package::getPath("map_coverage") + "/distanceTransformImg.pgm",distanceTransformImg);




                    // convert the path into poses
                    coveragePathPoses_ = covertPointsPathToPoseRout(path_);

                    // exectute currnet navigation the blob-coverage
                    cerr << " num of coverage waypoints " << coveragePathPoses_.size() << endl;
                    for (int i = 0; i < coveragePathPoses_.size(); i++)
                    {   

                        percentCoverage_ = ( float(i) / float(coveragePathPoses_.size()))  * 100.0;
                        node_.setParam("/coverage/percentage", percentCoverage_);

                      

                        publishCoverageImgMap();

                        
                        publishCoveragePath(coveragePathPoses_);
                        publishCoverageWayPoints(coveragePathPoses_, i);

                        bool result = sendGoal(coveragePathPoses_[i]);

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

                    prevAlgoMap_ = currentAlgoMap_.clone();

                    break;
                }
                case BACK_TO_STARTING_LOCATION:
                {   
                    cerr<<" BACK_TO_STARTING_LOCATION "<<endl;   

                    bool result = sendGoal(startingLocation_);

                    if (result)
                    {
                        cerr <<"BACK_TO_STARTING_LOCATION reached!" << endl;
                    }
                    else
                    {
                        cerr <<" Failed to reach BACK_TO_STARTING_LOCATION" << endl;
                    }

                    coverage_state_ = COVERAGE_DONE;  
                    
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
                    cerr<<" ERROR_COVERAGE "<<endl;  
                    
                    node_.setParam("/coverage/state", "STOPPED");

                    coverage_state_ = ERROR_COVERAGE;
                    break;
                }
            }


        }

       
    }   

    bool  sendGoal(const geometry_msgs::PoseStamped &goalMsg)
    {      

        //navigate to the point			
        moveBaseController_.navigate(goalMsg);

        auto start = ros::WallTime::now();
        bool result = true;
        while(ros::ok()) {

            moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));

            if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
                ||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
                ||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED){
                
                if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    result = true;
                } else {
                    result = false;
                }
                break;
            } 

            auto end = ros::WallTime::now();
            auto duration = (end - start).toSec();

            updateRobotLocation();

            float distFromGoal = 
                     goalCalculator.distanceCalculate( cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y),
                         cv::Point2d(goalMsg.pose.position.x, goalMsg.pose.position.y));


            cerr<<" the duration is "<<duration<<" distFromGoal "<<distFromGoal<<endl;
            if( duration > duration_wait_for_move_base_response_ && distFromGoal < 2.0){ 
                return false;
            }

            ros::spinOnce();
        }  

        

        return result;
    }

    
    void saveCoverageImg(){

        if( !imgSaved_ && mapping_map_.data){

            Mat coverageImg = mapping_map_.clone();
            cvtColor(coverageImg, coverageImg, COLOR_GRAY2BGR);

            // draw the path  

            for( int i = 0; i < path_.size(); i++){

                if( i > 0 ){
                    cv::line(coverageImg, path_[i], path_[i - 1], Scalar(34, 139, 139), 2);
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
           

            string image_name_format = startingTime_ + '_' +to_string(durationMinutes)+ '_' + to_string(int(percentCoverage_));
            string full_img_name = coverage_img_path_ + "/"+image_name_format+".png";
            node_.setParam("/coverage/image_name", image_name_format);

            cerr<<"full_img_name: "<<full_img_name<<endl;

            cv::imwrite(full_img_name, coverageImg);

            imgSaved_ = true;
        }
    }

private:
    COVERAGE_STATE coverage_state_ = COVERAGE_STATE::COVERAGE;

    EXPLORE_STATE explore_state_ = EXPLORE_STATE::NAV_TO_SAFEST_GOAL;

    // move-base
    MoveBaseController moveBaseController_;

    vector<geometry_msgs::PoseStamped> coveragePathPoses_;
    vector<cv::Point> path_;

    // subs
    ros::Subscriber global_map_sub_;

    // pubs

    ros::Publisher cuurentCoveragePathPub_;

    ros::Publisher start_coverag_marker_pub_;

    ros::Publisher goal_coverag_marker_pub_;

    ros::Publisher edges_frontires_marker_array_Pub;

    ros::Publisher coverage_waypoints_marker_array_Pub;

    ros::Publisher start_a_star_marker_pub_;

    ros::Publisher safest_goal_marker_pub_;

    ros::Publisher goal_a_star_marker_pub_;

    ros::Publisher global_start_marker_pub_;

    image_transport::Publisher  coverage_map_pub_;

    DistanceTransformGoalCalculator distanceTransformGoalCalculator;
    DisantanceMapCoverage disantanceMapCoverage;
    GoalCalculator goalCalculator;

    // ALGO-PARAMS

    std::vector<Frontier> currentBlobsFrontiers_;

    std::vector<Frontier> currentEdgesFrontiers_;

    vector<cv::Point> gloalPath_;

    cv::Point startCoveragePoint_;

    cv::Point goalCoveragePoint_;

    cv::Point aStarStart_;

    cv::Point globalStart_;

    cv::Mat currentAlgoMap_;

    cv::Mat prevAlgoMap_;

    ros::NodeHandle node_;

    // timers
    ros::Timer algoTimer_;

    // params

    geometry_msgs::PoseStamped robotPose_;

    geometry_msgs::PoseStamped robotStartLocation_;

    cv::Mat currentGlobalMap_;

    cv::Mat mapping_map_;

    float map_origin_position_x = -1;

    float map_origin_position_y = -1;

    float globalMapWidth_;

    float globalMapHeight_;

    float mapResolution_ = -1;

    bool init_ = false;

    float width_;

    float height_;

    double distBetweenGoalsM_ = 0.5;

    tf::TransformListener tfListener_;

    string globalFrame_ = "map";

    string baseFrame_ = "base_footprint";


    double robot_radius_meters_ = 0.3;

    double nim_map_score_to_finish_exploration_ = 70.0;

    double duration_wait_for_move_base_response_ = 15.0;

    double percentCoverage_ = 0.0;

    string coverage_img_path_ = "";

    string image_name_ = "";

    string state_ = "INITIALIZING";

    

    bool imgSaved_ = false;

    high_resolution_clock::time_point startingCoverageTime_;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_coverage_exploration_node");


    MapCoverageManager mapCoverageManager;
    if ( mapCoverageManager.exlpore()) {
        
        mapCoverageManager.coverage();

    }


    return 0;
}


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "map_coverage_exploration_node");

//     DistanceTransformGoalCalculator distanceTransformGoalCalculator;
//     DisantanceMapCoverage disantanceMapCoverage(true);
//     GoalCalculator goalCalculator;

//     float mapResolution_ = 0.05;
//     float distBetweenGoalsM_ = 0.5;
//     int pixDist = (1.0 / mapResolution_) * distBetweenGoalsM_;
//     float robot_radius_meters_ = 0.0;

//     Mat currentAlgoMap_ = imread("/home/yakir/distance_transform_coverage_ws/map.pgm",0);
//     cv::flip(currentAlgoMap_, currentAlgoMap_, 0);
//     Mat mappingMap = currentAlgoMap_.clone();  

   
//     addDilationForGlobalMap(currentAlgoMap_, robot_radius_meters_, mapResolution_);
//     addFreeSpaceDilation(currentAlgoMap_);

//     if( true) {
        
//         cerr<<"yakir "<<endl;
//         cv::Point globalStart_(80,118);
//         cv::Point safestGoal;

             
//         imshow("currentAlgoMap_", currentAlgoMap_);
//         waitKey(0);
//         if (!goalCalculator.findSafestLocation(currentAlgoMap_, globalStart_, safestGoal))
//         {

//             return -1;
//         }

//         return 0;
//     }

//     if( false) {

//         cv::Mat distanceTransformImg;

//         cv::Point2d currentPosition(380, 207);
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
//                                                 goal, distanceTransformImg, pixDist, true);          



//         for( int i = 0; i < path.size(); i++){


//             if( i > 0 ){
//                 cv::line(dbg, path[i], path[i - 1], Scalar(34, 139, 139), 2);
//             }

//         }     

//         circle(dbg, goal, 2, Scalar(0,255,0), -1, 8, 0);
//         circle(dbg, currentPosition, 2, Scalar(0,0,255), -1, 8, 0);  

//         imwrite("/home/yakir/distance_transform_coverage_ws/dbg.png", dbg);
//         imshow("dbg",dbg);
//         // imshow("distanceTransformImg", grayDistImg);
//         waitKey(0);
//     }

    
                    

//     return 0;
// }
