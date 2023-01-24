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


    // int cc = 0;
    void calcEdgesFrontiers(const Mat& map, std::vector<Frontier>& currentEdgesFrontiers, const cv::Point& robotPix,
                            float mapResolution,
                            const vector<cv::Point2d>& unreachedPointFromFronitiers) {

        cv::Mat binarayImage = map.clone();
        binarayImage.setTo(255, map >= 254);
        binarayImage.setTo(0, binarayImage != 255);


        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(binarayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));  

        cerr << " contours blobs size " << contours.size() << endl;

        if (contours.size() == 0)
        {   
            cerr<<" cant find any contours (white blobs "<<endl;
            return;
        }

        
        // find the largest white blob
        int maxArea = 0;
        int index = -1;
        for (int i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                index = i;
            }
            
        }

        for(int i = 0; i < contours.size(); i ++){

            if( i != index)
                 drawContours( binarayImage, contours, i, Scalar(0), -1, LINE_8, hierarchy, 0 );

        }
        
        for(int i = 0; i < unreachedPointFromFronitiers.size(); i++){
            
            binarayImage.at<uchar>(unreachedPointFromFronitiers[i].y, unreachedPointFromFronitiers[i].x) = 0;
        }
       
        // imshow("binarayImage", binarayImage);
        // waitKey(0);
        
        Mat freePixImg = cv::Mat(map.rows, map.cols , CV_8UC1, Scalar(0));
        vector<cv::Point> free_points;

        // loop over white blob iamge and find white pix that have gray Neighbors
        for (int j = 0; j < binarayImage.rows; j++)
        {
        for (int i = 0; i < binarayImage.cols; i++)
        {
            cv::Point2d currentP(i, j);

            auto value = binarayImage.at<uchar>(currentP);

            if ( value == 255) {
            
                // the Neighbors
                cv::Point up(currentP.x, currentP.y - 1);
                cv::Point down(currentP.x, currentP.y + 1);

                cv::Point left(currentP.x - 1, currentP.y);
                cv::Point right(currentP.x + 1, currentP.y);

                cv::Point topRight(currentP.x + 1, currentP.y - 1);
                cv::Point topLeft(currentP.x - 1, currentP.y - 1);

                cv::Point downRight(currentP.x + 1, currentP.y + 1);
                cv::Point downLeft(currentP.x - 1, currentP.y + 1);

                if( map.at<uchar>(up) == 205 || 
                    map.at<uchar>(down) == 205 ||
                    map.at<uchar>(left) == 205 ||
                    map.at<uchar>(right) == 205 ||
                    map.at<uchar>(topRight) == 205 ||
                    map.at<uchar>(topLeft) == 205 ||
                    map.at<uchar>(downLeft) == 205 || 
                    map.at<uchar>(downRight) == 205 ) {
                

                    freePixImg.at<uchar>(currentP) = 255; 

                    free_points.push_back(currentP);        
                }
                    
            }        
        }
        } 

        // imwrite("/home/yakir/distance_transform_coverage_ws/data/binarayImage" +to_string(cc) +".png",binarayImage);
        // imwrite("/home/yakir/distance_transform_coverage_ws/data/freePixImg" +to_string(cc) +".png",freePixImg);

        // cc++;
        
        // find the connected components and the centroids of them

        Mat labelImage(freePixImg.size(), CV_32S);
        Mat cnetroids;
        Mat statesImage(freePixImg.size(), CV_32S);

        int nLabels = connectedComponentsWithStats(freePixImg,labelImage,
                    statesImage, cnetroids, 8);
        
        cerr<<" nLabels "<<nLabels<<endl;
        if( nLabels == 0){

            cerr<<" there is no connected componnets !!"<<endl;
            return;
        }

        // cv::Mat debug = freePixImg.clone();
        // cvtColor(debug, debug, COLOR_GRAY2BGR);


        cerr<<" free_points "<<free_points.size()<<endl;
        vector<cv::Point> finalCneters;
        
        // loop over each centorid
        for(int row = 0; row < cnetroids.rows; row++){
            float x = cnetroids.at<double>(row, 0);
            float y = cnetroids.at<double>(row, 1);

            if(row == 0){
                continue;
            }
            //create the centroid
            cv::Point2d center(x, y);
            
            float minDist = 9999.9;
            int index = -1;

            // if the centorid is not on free space,
            // update the cnetorid to be on the closest free space point
            if( freePixImg.at<uchar>(center.y , center.x) != 255 ) {               

                //find the closest white point to the center and ovveride the cnter
                for(int k = 0; k < free_points.size(); k++ ) {              

                    float dist = distanceCalculate(free_points[k], center);
                    if (dist < minDist){
                        minDist = dist;
                        index = k;
                    }              
                }

                // shift the centoird to closest white pix
                if ( index != -1){
                    
                    center = free_points[index];
                    finalCneters.push_back(center);
                }
            

            } else {

                 
                //the centroid already on white pix
                finalCneters.push_back(center);
            }
        

        }


        // create the list of currentEdgesFrontiers
        for(int i = 0; i < finalCneters.size(); i++) {       

            Frontier f;
            f.center = finalCneters[i];
            f.distFromPosition = distanceCalculate(f.center, robotPix);

            currentEdgesFrontiers.push_back(f); 
        }   

        // sort the frontier
        std::sort( currentEdgesFrontiers.begin(), currentEdgesFrontiers.end(),
              []( const Frontier &left, const Frontier &right )
                 { return ( left.distFromPosition < right.distFromPosition ); } );
        

        
        // imshow( "Connected Components", dst );
        // imshow("freePixImg",freePixImg);
        //imshow( "cnetroidImage", cnetroidImage );


        // resize(debug, debug, cv::Size(debug.cols * 3, debug.rows * 3));  
        // imshow("debug",debug);
        // waitKey(0);                   
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