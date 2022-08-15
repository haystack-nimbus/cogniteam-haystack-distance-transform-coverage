#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <angles/angles.h>

#include "params.h"


using namespace cv;
using namespace std;


#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H

class DistanceTransformGoalCalculator
{
public:
    
    DistanceTransformGoalCalculator() {
    
    }

    ~DistanceTransformGoalCalculator() {}


    void setResolution(float resolution){

        resolution_ = resolution;
    }
    bool calcDistanceTransfromImg(const Mat &map, const cv::Point2d &goal,
                                   cv::Mat &outputDistanceTransformImg, int mapIndex)
    {

        if (goal.y < 0 || goal.x < 0 || goal.y > map.rows || goal.x > map.cols)
        {

            cerr << "error goal out of map !! " << goal<<endl;
            return false;
        }
        cv::Mat imgMap = map.clone();
        imgMap.setTo(255, imgMap == 254);
        imgMap.setTo(0, imgMap != 255);

        // pick the contour that contains the goal

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(imgMap, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));
        
        // imshow("imgMap",imgMap);
        // waitKey(0);
        
        if( contours.size() == 0 ){

            cerr<<" DistanceTransformGoalCalculator no contours"<<endl;
            return false;
        }             

        bool foundCont = false;
        for( int i =0; i < contours.size(); i++ ){

            if ( pointPolygonTest(contours[i], goal, false) > 0){
                
                foundCont = true;
            } else {
                
                drawContours(imgMap, contours, i, Scalar(0), -1 );
 
            }
        }

        // imshow("imgMap",imgMap);
        // waitKey(0);

        if( !foundCont){

            cerr<<" DistanceTransformGoalCalculator foundCont false"<<goal<<endl;

            return false;
        }



        // imshow("imgMap",imgMap);
        // waitKey(0);

        outputDistanceTransformImg = cv::Mat(map.rows,
                                             map.cols, CV_32S, cv::Scalar(LARGE_NUM));

        cv::Mat blocked(map.rows, map.cols, CV_8UC1, cv::Scalar(0));
        blocked.setTo(1, imgMap != 255);

        outputDistanceTransformImg.at<int>(goal.y, goal.x) = 0;

        cv::Mat previous_distanceTransform; // = outputDistanceTransformImg.clone();
        vector<int> values;
        values.resize(5);

        int cnt = 0;

        do
        {

            cnt++;

            previous_distanceTransform = outputDistanceTransformImg.clone();

            for (int y = 2; y < outputDistanceTransformImg.rows; y++)
            {
                for (int x = 2; x < outputDistanceTransformImg.cols; x++)
                {
                    if (blocked.at<uchar>(y, x) != 1)
                    {
                        values[0] = outputDistanceTransformImg.at<int>(y, x - 1) + 1;
                        values[1] = outputDistanceTransformImg.at<int>(y - 1, x - 1) + 1;
                        values[2] = outputDistanceTransformImg.at<int>(y - 1, x) + 1;
                        values[3] = outputDistanceTransformImg.at<int>(y - 1, x + 1) + 1;
                        values[4] = outputDistanceTransformImg.at<int>(y, x);

                        int dist = getMinElement(values);

                        outputDistanceTransformImg.at<int>(y, x) = dist;
                    }
                }
            }
            for (int y = outputDistanceTransformImg.rows - 1; y >= 1; y--)
            {
                for (int x = outputDistanceTransformImg.cols - 1; x >= 1; x--)
                {
                    if (blocked.at<uchar>(y, x) != 1)
                    {
                        values[0] = outputDistanceTransformImg.at<int>(y, x + 1) + 1;
                        values[1] = outputDistanceTransformImg.at<int>(y + 1, x + 1) + 1;
                        values[2] = outputDistanceTransformImg.at<int>(y + 1, x) + 1;
                        values[3] = outputDistanceTransformImg.at<int>(y + 1, x - 1) + 1;
                        values[4] = outputDistanceTransformImg.at<int>(y, x);

                        int dist = getMinElement(values);

                        outputDistanceTransformImg.at<int>(y, x) = dist;
                    }
                }
            }

        }

        while (imgsDistance(previous_distanceTransform, outputDistanceTransformImg) > 0.0);


        previous_distanceTransform.release();
        blocked.release();

        return true;
    }

    int getMinElement(const vector<int> &values, int x = 0, int y = 0)
    {

        int min = LARGE_NUM;
        for (int k = 0; k < 5; k++)
        {
            if (values[k] < min)
            {
                min = values[k];
            }
        }
        if (min < 0)
        {
           
            return 0;
        }
        return min;
    }

    void normalizeDistanceTransform(const Mat &distanceTransform, cv::Mat &grayScaledImg)
    {

        grayScaledImg = cv::Mat(distanceTransform.rows, distanceTransform.cols, CV_8UC1, cv::Scalar(0));

        int max = 0;
        int min = LARGE_NUM;

        for (int y = 0; y < distanceTransform.rows; y++)
        {
            for (int x = 0; x < distanceTransform.cols; x++)
            {
                if (distanceTransform.at<int>(y, x) < min && distanceTransform.at<int>(y, x) != LARGE_NUM)
                {
                    min = distanceTransform.at<int>(y, x);
                }
                if (distanceTransform.at<int>(y, x) > max && distanceTransform.at<int>(y, x) != LARGE_NUM)
                {
                    max = distanceTransform.at<int>(y, x);
                }
            }
        }

        int smin = 0;
        int smax = 255;
        for (int y = 0; y < distanceTransform.rows; y++)
        {
            for (int x = 0; x < distanceTransform.cols; x++)
            {
                int valDist = distanceTransform.at<int>(y, x);

                if (valDist != LARGE_NUM)
                {

                    int normalize = (valDist - min) * (smax - smin) / (max - min) + smin;

                    grayScaledImg.at<uchar>(y, x) = normalize;
                }
                else
                {

                    grayScaledImg.at<uchar>(y, x) = 0;
                }
            }
        }
    }

    int convertPathToContour(const Mat& img, const vector<cv::Point>& path,
        vector<vector<Point>>& contours, vector<Vec4i>& hierarchy){

        Mat binarayImage = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
        //cvtColor(grayScaledImg, grayScaledImg, COLOR_GRAY2BGR);

        vector<Point> contour;
        for( int i =0; i < path.size(); i++){
            
            if( i == path.size() -1 ){
                break;
            }
            contour.push_back(path[i]);

            circle(binarayImage,path[i], resolution_, Scalar(255), -1, 8, 0);      

            cv::Point start = path[i];
            cv::Point end = path[i + 1];
            cv::LineIterator it_map(img, start, start, 4); 

            for (int j = 0; j < it_map.count; j++, ++it_map)
            {
                contour.push_back(it_map.pos()); 
                
                circle(binarayImage,it_map.pos(), resolution_, Scalar(255), -1, 8, 0);      
               
            }
        }

        binarayImage.setTo(0, img == LARGE_NUM );
        //binarayImage.setTo(0, img == 0 );


        

        findContours(binarayImage, contours, hierarchy, RETR_EXTERNAL,
                     CHAIN_APPROX_NONE, Point(0, 0));

        if( contours.size() != 1 ){

            return -1;
        }             




        return 0;


    }

    void setScoreVisitedPoint(cv::Mat &distanceTransformImg,
         const vector<cv::Point>& path,   int score )
    {
       

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        bool res = convertPathToContour(distanceTransformImg, path,
             contours, hierarchy);

        //fillConvexPoly(distanceTransformImg, contour,Scalar(score));

        if( res != -1)
         drawContours(distanceTransformImg, contours, 0, Scalar(score), -1 );
        
       
         
       
    }

private:
    float imgsDistance(const cv::Mat &a, const cv::Mat &b)
    {

        assert(a.rows == b.rows);
        assert(a.cols == b.cols);
        float r = 0;
        for (int y = 0; y < a.rows; y++)
        {
            for (int x = 0; x < a.cols; x++)
            {
                r += abs(a.at<int>(y, x) - b.at<int>(y, x));
            }
        }
        return r;
    }

private:


    int resolution_;
};

#endif