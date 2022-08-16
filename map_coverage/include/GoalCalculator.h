#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <angles/angles.h>


#include "params.h"

using namespace cv;
using namespace std;


#ifndef GOAL_CALCULATOR_H
#define GOAL_CALCULATOR_H

class GoalCalculator
{
public:
    GoalCalculator() {}

    ~GoalCalculator() {}

    bool findSafestLocation(const Mat& map,  const cv::Point& start,
         cv::Point& goal) {   

        // int valGlo = map.at<uchar>(start.y, start.x);
        // if(valGlo != free_space){     
            
        //     cerr<<" robot not on free_space valGlo "<<valGlo<<endl;
        //     return false;
        // }
        cv::Mat binary = map.clone();
        binary.setTo(255, binary > 250);
        binary.setTo(0, binary != 255); 
        dilate(binary, binary, Mat(), Point(-1, -1), 3, 1, 1);

        // imshow("binary",binary);
        // waitKey(0);
   

        Mat dist;
        distanceTransform(binary, dist, DIST_L2, 3);
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        normalize(dist, dist, 0, 1.0, NORM_MINMAX);
      
        double min, max; 
        cv::Point minLoc;
        minMaxLoc(dist, &min, &max, &minLoc, &goal);


        // Mat dbg = map.clone();

        // cvtColor(map, dbg, COLOR_GRAY2BGR);

        // circle(dbg, goal, 5, Scalar(0,255,0), -1, 8, 0); 
        // circle(dbg, goal, 5, Scalar(255,255,0), -1, 8, 0); 

        // imshow("binary", binary);
        // imshow("dist", dist);
        // imshow("dbg", dbg);
        // waitKey(0);       

        if( goal.x > 0 && goal.y > 0){
            return true;
        }     

        return false;

    }
    bool getInitialGoal(const Mat& map , 
         const cv::Point& currentPosition, 
          cv::Point& goal, int dist_between_points ){  

        
      
        
        Mat img = map.clone();
        img.setTo(255, img >= 254);
        img.setTo(0, img != 255);


        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(img, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));



       

        if( contours.size() == 0 ){

            return false;
        }   
        int maxDist = 0;
        for( int j = 0; j < contours.size(); j++){

            for( int i = 0; i < contours[j].size(); i++){
                double dist = distanceCalculate(contours[j][i],currentPosition);
                if (dist > maxDist ) {
                    maxDist = dist;
                    goal = contours[j][i];

                                     
                }

            }
        }

        cerr<<" currentPosition "<<currentPosition<<" goal "<<goal<<endl;
        return true;
       
          
    }

    void mergeBlobs(vector<Frontier>& oldlblobsFrontiers,
        const vector<Frontier>& newblobsFrontiers ){


        vector<bool> old_indexes;
        vector<bool> new_indexes;
        old_indexes.resize(oldlblobsFrontiers.size());  
        new_indexes.resize(newblobsFrontiers.size()); 
        for( int i = 0; i < old_indexes.size(); i++){
             old_indexes[i] = false;
        }
        for( int i = 0; i < new_indexes.size(); i++){
             new_indexes[i] = false;
        }


        
        vector<Frontier> updatedBlobsFrontiers;

        for( int j = 0; j < oldlblobsFrontiers.size(); j++){

            Frontier blobO = oldlblobsFrontiers[j];

            for( int i = 0; i < newblobsFrontiers.size(); i++){
                Frontier blobC = newblobsFrontiers[i];

                // new already inside
                if( new_indexes[j] == true){
                    continue;
                }

                double dist = distanceCalculate(blobO.center, blobC.center);

                // merge
                if( dist < 5){
                    old_indexes[j] = true;
                    new_indexes[i] = true;

                    updatedBlobsFrontiers.push_back(blobC);
                    break;
                }

            }

            // push old
            if( old_indexes[j] == false){
                updatedBlobsFrontiers.push_back(blobO);
            }

        }

        // insert new available
        for( int i = 0; i < new_indexes.size(); i++){
            if(new_indexes[i] == false){
                updatedBlobsFrontiers.push_back(newblobsFrontiers[i]);
            }
        }       

        oldlblobsFrontiers = updatedBlobsFrontiers;


    }

    void getCurrentBlobsF(
        vector<Frontier>& Blobs, 
        const Mat &prevMap, const Mat &map, 
        cv::Point& currentPosition,
        int minAreaPix) {
        

        Mat diff;
        absdiff(prevMap, map, diff);
        diff.setTo(255, diff > 0);
        diff.setTo(0, map == 0);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(diff, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));
       
        if (contours.size() == 0){
            return;
        }      

        float minDist = 99999;
        int minIndex = -1;
        for(int i =0; i < contours.size(); i++){
            
            if( contourArea(contours[i])  < minAreaPix){
                continue;
            }
            
            auto m = moments(contours[i]);
            cv::Point cneter( m.m10 / m.m00, m.m01 / m.m00);

            Frontier Blob;
            Blob.center = cneter;
            Blob.contour = contours[i];

            Blob.distFromPosition = distanceCalculate(cneter, currentPosition);
            
            Blobs.push_back(Blob);           
        }

    }
    

    int pickBestBlob(const Mat &map, cv::Point& currentPosition,
        vector<Frontier>& curreblobs,
        cv::Point& start, cv::Point& goal, int dist_between_points,
        const cv::Point& globalStart){


        double minDist = 9999;    
        int index = -1;
        Frontier bestBlob;
        for(int i = 0; i < curreblobs.size(); i++) {

            Frontier blobC = curreblobs[i];            

            double dist = distanceCalculate(currentPosition, blobC.center);
            if( dist < minDist ){
                minDist = dist;
                index = i;
            }

        }

        if( index == - 1){
            
            return -1;
        }

        bestBlob = curreblobs[index];


        float maxDistOnCountor = 0;
        cv::Point farestPoint;
        float minDistOnCountor = 999999;
        cv::Point closestPoint;

        for(int i = 0; i< bestBlob.contour.size(); i++){
            double dist = distanceCalculate(currentPosition, bestBlob.contour[i]);
            
            int val = map.at<uchar>(bestBlob.contour[i].y, bestBlob.contour[i].x);

            if( dist > maxDistOnCountor && val == 254
                && (abs(bestBlob.contour[i].y - globalStart.y)) % dist_between_points == 0
                && (abs(bestBlob.contour[i].x - globalStart.x ))% dist_between_points == 0){
               
                maxDistOnCountor = dist;
                farestPoint = bestBlob.contour[i];
            }
            if( dist < minDistOnCountor && val == 254
                && (abs(bestBlob.contour[i].y - globalStart.y)) % dist_between_points == 0
                && (abs(bestBlob.contour[i].x - globalStart.x ))% dist_between_points == 0){
               
                minDistOnCountor = dist;
                closestPoint = bestBlob.contour[i];
            }

        }

        start = closestPoint;
        goal = farestPoint;        

        return index;



    }
    
    bool getExplorationGoalByBlob(vector<Frontier>& oldblobsFrontiers,
        const Mat &prevMap, const Mat &map, 
        cv::Point& currentPosition,
        cv::Point& start, cv::Point& goal, int dist_between_points,
        const  cv::Point& globalStart) {
        
        vector<Frontier> curreblobs;  
        getCurrentBlobsF(curreblobs, prevMap, map, currentPosition, dist_between_points * dist_between_points);

         
        mergeBlobs(oldblobsFrontiers, curreblobs);

        auto res = pickBestBlob(map, currentPosition,
              oldblobsFrontiers, start, goal, dist_between_points, globalStart);        
      

        if(res != -1){

            oldblobsFrontiers.erase(oldblobsFrontiers.begin() + res);
            //  cv::Mat debug = map.clone();
            // cvtColor(debug, debug, COLOR_GRAY2BGR);

            // circle(debug,start, 5, Scalar(255, 0, 0), -1, 8, 0);      
            // circle(debug,goal, 5, Scalar(255, 0, 255), -1, 8, 0);      


            // imshow("debug",debug);
            // waitKey(0);

            return true;    
        }     

        return false;       
    }


    float calcEdgesFrontiers(const Mat& map, 
        std::vector<Frontier>& currentEdgesFrontiers,
        const cv::Point& robotPix, float mapResolution) {

        cv::Mat binarayImage = map.clone();
        binarayImage.setTo(255, map == 254); 
        binarayImage.setTo(0, map == 205);


        //count num of wall points
        float numOfBlockedPoints = 0.0;
        for (int y = 0; y < map.rows; y++)
        {
            for (int x = 0; x < map.cols; x++)
            {
                int val = map.at<uchar>(y, x);

                if ( val == 0){
                    numOfBlockedPoints+= 1;
                } 
            }
        }


        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(binarayImage, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));
       
        if (contours.size() == 0){
            return -1;
        }



        cv::Mat debug = map.clone();
        cvtColor(debug, debug, COLOR_GRAY2BGR);

        int maxArea = 0;
        int index = -1;
        for (int i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            if (area > maxArea)
            {   maxArea = area;
                index = i;
            }            
        }

       

        if( index == -1){

            cerr<<" no blobs "<<endl;
            return -1;
        }

        vector<cv::Point> freeGoals;
        for(int i = 0; i < contours[index].size(); i++){

            cv::Point p = contours[index][i];

            bool foundCollision = false;

            int val = map.at<uchar>(p.y, p.x);
            if( val != 254){  
                
                foundCollision = true;
                continue;
            }     

            for (int ray_angle = 0; ray_angle < 360; ray_angle++) {
                
                ///meters
                cv::Point2d rayPix;
                rayPix.x = p.x + (2 * cos(angles::from_degrees(ray_angle)));
                rayPix.y = p.y + (2 * sin(angles::from_degrees(ray_angle)));
            
                /// out of map
                if (rayPix.x > map.cols || rayPix.y > map.rows ||
                    rayPix.x < 0 || rayPix.y < 0)
                {   
                    foundCollision = true;
                    continue;
                }            
                
                cv::LineIterator it_globalMap(map, p, rayPix, 4 ); //4 more dense than 8
              
                for (int j = 0; j < it_globalMap.count; j++, ++it_globalMap)
                {
                    cv::Point2d pointBeam = it_globalMap.pos();                  
                  
                    if (pointBeam.x > map.cols || pointBeam.y > map.rows ||
                        pointBeam.x < 0 || pointBeam.y < 0)
                    {   
                        foundCollision = true; 
                        break;
                    }                      

                  
                    int value = map.at<uchar>(pointBeam.y, pointBeam.x);

                    if (value == 0)
                    {                           
                        foundCollision = true; 
                        break;
                    }
                    
                }           
            }
            
            if( !foundCollision){
                freeGoals.push_back(p);
            }
        }

        ////////////////////////////
        cv::Mat edgesContImg = map.clone();
        edgesContImg.setTo(0);
        contours.clear();
        hierarchy.clear();
        
        for(int i =0; i < freeGoals.size(); i++){
           edgesContImg.at<uchar>(freeGoals[i].y, freeGoals[i].x) = 255;          

        }       
       
       

        findContours(edgesContImg, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));     


        // imshow("debug",debug);
        // imshow("edgesContImg",edgesContImg);
        // waitKey(0);  

        float totalFreePoints = 0;
        for (int i = 0; i < contours.size(); i++) {
            
            Frontier f;
            f.contour =  contours[i];

            f.center.x = 0;
            f.center.y = 0;
            for(int k = 0; k < contours[i].size(); k++ ){

                f.center.x += contours[i][k].x;
                f.center.y += contours[i][k].y;
            }

            f.center.x = (float)f.center.x / (float)contours[i].size();
            f.center.y = (float)f.center.y / (float)contours[i].size();

            f.distFromPosition =   distanceCalculate(f.center, robotPix);


            float contAreaM =  (1.0 / mapResolution) * (cv::contourArea(f.contour) );    

            cerr<<" the frontier area in M "<<contAreaM<<endl;
            currentEdgesFrontiers.push_back(f);

            totalFreePoints += f.contour.size();
        }    


        // sort the frontier
        std::sort( currentEdgesFrontiers.begin(), currentEdgesFrontiers.end(),
              []( const Frontier &left, const Frontier &right )
                 { return ( left.distFromPosition < right.distFromPosition ); } );

        //calculate map score
        
        if( numOfBlockedPoints + totalFreePoints == 0 ){
            return   -1;
        }

        cerr<<" numOfBlockedPoints "<<numOfBlockedPoints<<" totalFreePoints "<<totalFreePoints<<endl;  
        float score =  numOfBlockedPoints / float(numOfBlockedPoints + totalFreePoints);

        if( score > 1.0){
            score = 1.0;
        }

        if( score  < 0){
            score = -1;
        }

        return score * 100;



    }

    // find the start and the goal for path coverage by diff maps
    bool foundGoalFromDiffMap(const Mat &prevMap, const Mat &map, 
         cv::Point& currentPosition, cv::Point& start, cv::Point& goal, int radius = 1){     


        Mat diff;
        absdiff(prevMap, map, diff);
        diff.setTo(255, diff > 0);
        diff.setTo(0, map == 0);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(diff, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));
       
        if (contours.size() == 0){
            return false;
        }      
        cvtColor(diff, diff, COLOR_GRAY2BGR);

        float minDist = 99999;
        int minIndex = -1;
        for(int i =0; i < contours.size(); i++){
            auto m = moments(contours[i]);
            cv::Point cneter( m.m10 / m.m00, m.m01 / m.m00);

            double dist = distanceCalculate(cneter, currentPosition);
            if( dist < minDist){
                minDist = dist;
                minIndex = i;
            }
        }

        float maxDistOnCountor = 0;
        cv::Point farestPoint;
        float minDistOnCountor = 999999;
        cv::Point closestPoint;

        for(int i = 0; i< contours[minIndex].size(); i++){
            double dist = distanceCalculate(currentPosition, contours[minIndex][i]);
            
            int val = map.at<uchar>(contours[minIndex][i].y, contours[minIndex][i].x);

            if( dist > maxDistOnCountor && val == 254){
                maxDistOnCountor = dist;
                farestPoint = contours[minIndex][i];
            }
            if( dist < minDistOnCountor && val == 254){
                minDistOnCountor = dist;
                closestPoint = contours[minIndex][i];
            }

        }

        start = closestPoint;
        goal = farestPoint;


        // imshow("diff",diff);
        // waitKey(1);
     
        return true;
    }  

    double distanceCalculate(cv::Point2d p1, cv::Point2d p2)
    {
        double x = p1.x - p2.x; //calculating number to square in next step
        double y = p1.y - p2.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

    

    
};

#endif