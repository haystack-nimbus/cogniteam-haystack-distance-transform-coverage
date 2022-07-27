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


/**
Algorithm in the appendix I of the chapter titled

JARVIS, Ray. Distance transform based path planning for robot navigation. *Recent trends in mobile robots*, 1993, 11: 3-31.

in the book

ZHENG, Yuang F. (ed.). *Recent trends in mobile robots*. World Scientific, 1993.

See also http://stackoverflow.com/questions/21215244/least-cost-path-using-a-given-distance-transform
*/

//  cv::Mat img 
//             = imread(pathImgs + to_string(1)+ ".pgm",0);
//     img.setTo(254, img > 254);
//     img.setTo(254, img > 205);
//     imwrite(pathImgs + "1.pgm", img);
//     return 0 ;

#include "../include/DisantanceMapCoverage.h"
#include "../include/GoalCalculator.h"
#include "../include/AstarPathManager.h"



int main(int, char **)
{
    float dist_between_points = 15;
 
    cv::Mat distanceTransformImg;

    DistanceTransformGoalCalculator distanceTransformGoalCalculator(dist_between_points);
    DisantanceMapCoverage disantanceMapCoverage;
    GoalCalculator goalCalculator; 
    AstarPathManager astarPathManager;


      

    string pathImgs = 
        "/home/cogniteam/map_coverage_ws/src/coverage_exploration/map_coverage/data/se1/";
       //"/home/yakir/distance_transform_coverage_ws/src/coverage_exploration/map_coverage/data/se1/";
    int imgIndex = 1;    


 
    //cv::Point start(146, 200);
    
    cv::Point startCoverage(110, 363);
    cv::Point goalCoverage;

    cv::Point aStarStart;

    cv::Point globalStart(startCoverage.x, startCoverage.y);
  
    cv::Mat preMap;

    Mat debugCurr; 
    cv::Mat debugGloal;


    RNG rng(12345);

    vector<cv::Point> gloalPath;

    vector<cv::Point> aStarPath;


    vector<int> grapghIndexes;

   vector<Frontier> currentBlobsFrontiers;


    bool mapClosed = false; 
    bool coverageDone = false;

    while ( !mapClosed && !coverageDone) {       

        cerr<<" load:  "<<pathImgs + to_string(imgIndex)+ ".pgm"<<endl;

        cv::Mat currentMap 
            = imread(pathImgs + to_string(imgIndex)+ ".pgm",0);
        if( !currentMap.data){
            break;
        }
        if( imgIndex == 1 ){
            auto res = goalCalculator.getIitialGoal(currentMap,
                 startCoverage, goalCoverage, dist_between_points,
                  globalStart);
            if( !res){
                cerr<<" first map bad !!  "<<endl;
                return -1;
            }
        }
        debugCurr = currentMap.clone();
        cvtColor(debugCurr, debugCurr, COLOR_GRAY2BGR);
        debugGloal = currentMap.clone();
        cvtColor(debugGloal, debugGloal, COLOR_GRAY2BGR);
   

        if( imgIndex > 1){
          
            cv::Point nextStartLocation;
            cv::Point nextGoalLocation;

            aStarStart = cv::Point(goalCoverage.x, goalCoverage.y);
           
            if( !(goalCalculator.getExplorationGoalByBlob(currentBlobsFrontiers,
                 preMap, currentMap, aStarStart, 
                 nextStartLocation, nextGoalLocation,
                 dist_between_points,
                 globalStart))){       


                cerr<<" failed to find the next goal "<<endl;
                break;

            } else {        

               
                goalCoverage = nextGoalLocation;
                startCoverage = nextStartLocation;

                cv::Mat astarMap = currentMap.clone();
                cvtColor(astarMap, astarMap, COLOR_GRAY2BGR);

                circle(astarMap,aStarStart, 5, Scalar(255, 0, 0), -1, 8, 0);    
                circle(astarMap,startCoverage, 5, Scalar(0, 0, 255), -1, 8, 0);    
                circle(astarMap,goalCoverage, 5, Scalar(0, 255, 0), -1, 8, 0);                

                // A-STAR between blobs
                astarPathManager.getRosPath(currentMap, aStarStart,
                     startCoverage,
                      aStarPath, dist_between_points);            


               for(int i = 0; i < aStarPath.size(); i++ ){

                    if( i != 0){
                        cv::arrowedLine(astarMap, aStarPath[i-1],
                             aStarPath[i],Scalar(0,0,255), 2);
                    }

                    // imshow("astarMap after",astarMap);
                    // waitKey(0);
                   
        
                }

                // imshow("astarMap after",astarMap);
                // waitKey(0); 
              
            }
        }

      
        distanceTransformGoalCalculator.calcDistanceTransfromImg(currentMap, 
            goalCoverage, distanceTransformImg, 1);

        cv::Mat grayScaledImg;
       

        if( imgIndex > 1) {          
            distanceTransformGoalCalculator.setScoreVisitedPoint(
                distanceTransformImg, gloalPath, LARGE_NUM);

            gloalPath.insert(gloalPath.end(), aStarPath.begin(),aStarPath.end());
            gloalPath.push_back(startCoverage); 
            aStarPath.clear();   
        }    
       


        vector<cv::Point> path = 
            disantanceMapCoverage.getCoveragePath(currentMap, startCoverage,
                goalCoverage ,distanceTransformImg, dist_between_points, true );
        cerr<<" getCoveragePath .size() "<<path.size()<<endl;

       
        grapghIndexes.push_back(gloalPath.size() + path.size());
        
        gloalPath.insert(gloalPath.end(), path.begin(),path.end());
        goalCoverage = cv::Point2d(gloalPath[gloalPath.size() -1 ].x,
             gloalPath[gloalPath.size() -1 ].y);

       
        Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
        for(int i = 0; i < path.size(); i++ ){

            if( i != 0){
                cv::arrowedLine(debugCurr, path[i-1], path[i],color, 1);
            }
        
        }

        // circle(debugCurr,startCoverage, 5, Scalar(0, 0, 255), -1, 8, 0);      
        // circle(debugCurr,goalCoverage, 5, Scalar(0, 100, 0), -1, 8, 0);   
 
        // imshow("debugCurr",debugCurr);
        // waitKey(0);       

           
        mapClosed = goalCalculator.calcEdgesFrontiers(currentMap,
         dist_between_points);

        imgIndex++;
        preMap = currentMap.clone();
    
        cerr<<"----------------------------------"<<endl;
    }

    cerr<<" fffffi "<<endl;

    Scalar color = Scalar(rng.uniform(100,255), rng.uniform(100, 255), rng.uniform(100, 255));
    for(int i = 0; i < gloalPath.size(); i++ ){

        for(int j =0; j < grapghIndexes.size(); j++){
            if(i == grapghIndexes[j]){
                color = Scalar(rng.uniform(100,255), rng.uniform(100, 255), rng.uniform(100, 255));
            }
        }
        
        int index = grapghIndexes[i];
        if( i != 0){
            cv::arrowedLine(debugGloal, gloalPath[i-1], 
                gloalPath[i],color, 3);
        }

        // imshow("debugGloal",debugGloal);
        // waitKey(0);

       
    }

   


    circle(debugGloal,globalStart, 5, Scalar(0, 255, 0), -1, 8, 0);        


    imwrite("/home/cogniteam/map_coverage_ws/debugGloal.png", debugGloal);

    
    // imshow("debugGloal",debugGloal);
    // waitKey(0);
   
   
    

   

   
    
    return 0;
   

}                 
