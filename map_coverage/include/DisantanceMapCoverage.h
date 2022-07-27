#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <map>
#include <iostream>

#include "params.h"

#include "DistanceTransformGoalCalculator.h"

using namespace cv;
using namespace std;

#ifndef DISTANCE_MAP_COVERAGE_H
#define DISTANCE_MAP_COVERAGE_H


class DisantanceMapCoverage
{
public:
    DisantanceMapCoverage(bool debug = false) {

        debug_ = debug;

        cerr<<" DisantanceMapCoverage debug_ "<<debug_<<endl;
    }

    ~DisantanceMapCoverage() {}

    vector<cv::Point> getCoveragePath(const cv::Mat &imgMap,
                        const cv::Point& start,
                        const cv::Point& goal,
                        cv::Mat& distanceTransformImg,
                        double dist_between_points,                        
                        bool stopWhenGoalReached = false)
    {      
        cv::Mat visitedCells(distanceTransformImg.rows,
                             distanceTransformImg.cols, CV_8UC1, cv::Scalar(NOT_VISITED));

        cv::Mat grayScaleImg  = imgMap.clone();
        cvtColor(grayScaleImg, grayScaleImg, COLOR_GRAY2BGR);

        vector<cv::Point> path;
        cv::Point currentP(start.x, start.y);

        std::map<string, cv::Point> son_father;
        
        do
        {

            if (path.size() > 0 && debug_)
            {
                cv::line(grayScaleImg, currentP, path[path.size() - 1], Scalar(255, 255, 0), 2);
            }

            if( debug_)
                circle(grayScaleImg, currentP, 2, Scalar(0, 255, 255), -1, 8, 0);

            // try no find valid neighbor with largest distance
            cv::Point NeighborCell;
            bool foundN = findNeighborCell(distanceTransformImg, visitedCells,
                                           currentP, NeighborCell, grayScaleImg, dist_between_points);

            
            // if( foundN && stopWhenGoalReached){
                
            //     float currPvalue =
            //          distanceTransformImg.at<int>(currentP.y, currentP.x);
            //     float dist = manhattan_distance(currentP, goal);
            //     if ( (dist <= float(dist_between_points) * 1) ||
            //         currPvalue <= dist_between_points )
            //     {   

            //         cerr<<"11111111111111111111111111111 "<<endl;

            //         break;
            //     }
            // }

            // if not found
            if (!foundN)
            {   

                float currPvalue =
                     distanceTransformImg.at<int>(currentP.y, currentP.x);
                float dist = manhattan_distance(currentP, goal);
                if ( (dist <= float(dist_between_points) * 1) ||
                    currPvalue <= dist_between_points )
                {   

                    cerr<<"222222222222222222222222222 "<<endl;

                    break;
                }
                

                path.push_back(currentP);
                visitedCells.at<uchar>(currentP.y, currentP.x) = VISITED;

                string pString = getPointString(currentP);
                if( son_father.find(pString) == son_father.end()){
                    
                    cerr<<"reached to start again "<<endl;
                    break;
                }
                cv::Point father = son_father.at(pString);

                // cerr<<"go to father "<<father<<endl;
                NeighborCell = father;
                currentP = father;

                if( debug_){
                    grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[0] = 255;
                    grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[1] = 255;
                    grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[2] = 0;

                    circle(grayScaleImg,currentP, 4, Scalar(0, 100, 255), -1, 8, 0);   
                }
                if( debug_){
                    imshow("grayScaleImg",grayScaleImg);
                    waitKey(0); 
                 }
               
                continue;
            }


            if (debug_)
            {
                grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[0] = 255;
                grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[1] = 255;
                grayScaleImg.at<cv::Vec3b>(NeighborCell.y, NeighborCell.x)[2] = 0;
            }            

            // found NeighborCell with smallest distnace then current, go there
            if (distanceTransformImg.at<int>(NeighborCell.y, NeighborCell.x) <=
                distanceTransformImg.at<int>(currentP.y, currentP.x))
            {

                // cerr << "found NeighborCell small then current " << endl;

                visitedCells.at<uchar>(currentP.y, currentP.x) = VISITED;

                if (debug_)
                {

                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[0] = 0;
                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[1] = 255;
                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[2] = 0;


                }

                path.push_back(currentP);

                son_father[getPointString(NeighborCell)] = currentP;

                currentP = NeighborCell;
            }
            // found NeighborCell with larger distnace then current, go there
            else
            {

                // cerr << "found NeighborCell large then current " << endl;

                visitedCells.at<uchar>(currentP.y, currentP.x) = VISITED;

                if (debug_)
                {

                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[0] = 0;
                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[1] = 255;
                    grayScaleImg.at<cv::Vec3b>(currentP.y, currentP.x)[2] = 0;


                }

                path.push_back(currentP);

                son_father[getPointString(NeighborCell)] = currentP;

                currentP = NeighborCell;
            }

            if (debug_){

                 circle(grayScaleImg,currentP, 4, Scalar(255, 0, 0), -1, 8, 0);   

                imshow("grayScaleImg",grayScaleImg);
                waitKey(0);
            }
           

        } while (true);

       
        return path;
    }

private:
    string getPointString(cv::Point const &a)
    {
        string s = to_string(a.x) + "_" + to_string(a.y);
        return s;
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

    float manhattan_distance(const cv::Point& p1, const cv::Point& p2) {

        double distance;
        int x_dif, y_dif;
        
        x_dif = p2.x - p1.x;
        y_dif = p2.y - p1.y;

        if(x_dif < 0)
            x_dif = -x_dif;

        if(y_dif < 0)
            y_dif = -y_dif;

        distance = x_dif + y_dif;

        return distance;

    }


    bool checkCollision(const Mat &distanceTransform, const cv::Point &currentPoint,
                        const cv::Point &neighboar)
    {

        cv::LineIterator it_map(distanceTransform, currentPoint, neighboar, 4); //4 more dense than 8
        for (int j = 0; j < it_map.count; j++, ++it_map)
        {
            cv::Point2d pointBeam = it_map.pos();
            int valueTemp = distanceTransform.at<int>(pointBeam.y, pointBeam.x);
            if (valueTemp == LARGE_NUM)
            {
                return true;
            }
        }

        return false;
    }
    void neighboarValidation(Direction direction, const Mat &distanceTransform,
                             const Mat &visitedCells,
                             const cv::Point &currentPoint,
                             const cv::Point &neighboar,
                             vector<bool> &visitedCellsNe,
                             vector<bool> &validCells, Mat &grayScaleImg)
    {

        // if cell inside the img
        if (neighboar.y > 0 && neighboar.y < distanceTransform.rows &&
            neighboar.x > 0 && neighboar.x < distanceTransform.cols)
        {

            validCells[direction] = true;

            // if its blocked
            if (distanceTransform.at<int>(neighboar.y, neighboar.x) == LARGE_NUM)
            {
                validCells[direction] = false;
            }

            // if there is collisions
            if (checkCollision(distanceTransform, currentPoint, neighboar))
            {
                validCells[direction] = false;
            }

            if (visitedCells.at<uchar>(neighboar.y, neighboar.x) == VISITED)
            {
                visitedCellsNe[direction] = true;
            }
            else
            {
                visitedCellsNe[direction] = false;
            }

            if (validCells[direction] && debug_)
            {

                if (visitedCellsNe[direction])
                {

                    circle(grayScaleImg, neighboar, 2, Scalar(0, 0, 255), -1, 8, 0);
                }
                else
                {
                    circle(grayScaleImg, neighboar, 2, Scalar(0, 255, 0), -1, 8, 0);
                }
            }
        }
        else
        {

            validCells[direction] = false;
            visitedCellsNe[direction] = true;
        }
    }

    bool findNeighborCell(const Mat &distanceTransform, const Mat &visitedCells,
                          const cv::Point &currentP, cv::Point &NeighborCell,
                          cv::Mat &grayScaleImg, int dist_between_points)
    {

        cv::Point up(currentP.x, currentP.y - dist_between_points);
        cv::Point down(currentP.x, currentP.y + dist_between_points);

        cv::Point left(currentP.x - dist_between_points, currentP.y);
        cv::Point right(currentP.x + dist_between_points, currentP.y);

        cv::Point topRight(currentP.x + dist_between_points, currentP.y - dist_between_points);
        cv::Point topLeft(currentP.x - dist_between_points, currentP.y - dist_between_points);

        cv::Point downRight(currentP.x + dist_between_points, currentP.y + dist_between_points);
        cv::Point downLeft(currentP.x - dist_between_points, currentP.y + dist_between_points);

        vector<cv::Point> ne;
        ne.resize(8);
        ne[UP] = up;
        ne[DOWN] = down;
        ne[LEFT] = left;
        ne[RIGHT] = right;
        ne[TOP_RIGHT] = topRight;
        ne[TOP_LEFT] = topLeft;
        ne[DOWN_RIGHT] = downRight;
        ne[DOWN_LEFT] = downLeft;

        vector<bool> visitedCellsNe{true, true, true, true, true, true, true, true};

        vector<bool> validCells{false, false, false, false, false, false, false, false};

        vector<int> scores{0, 0, 0, 0, 0, 0, 0, 0};
    
        //  UP, DOWN, LEFT, RIGHT, TOP_RIGHT, TOP_LEFT, DOWN_RIGHT, DOWN_LEFT
        neighboarValidation(UP, distanceTransform, visitedCells, currentP,
                            ne[UP], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(DOWN, distanceTransform, visitedCells, currentP,
                            ne[DOWN], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(LEFT, distanceTransform, visitedCells, currentP,
                            ne[LEFT], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(RIGHT, distanceTransform, visitedCells, currentP,
                            ne[RIGHT], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(TOP_RIGHT, distanceTransform, visitedCells, currentP,
                            ne[TOP_RIGHT], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(TOP_LEFT, distanceTransform, visitedCells, currentP,
                            ne[TOP_LEFT], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(DOWN_RIGHT, distanceTransform, visitedCells, currentP,
                            ne[DOWN_RIGHT], visitedCellsNe, validCells, grayScaleImg);
        neighboarValidation(DOWN_LEFT, distanceTransform, visitedCells, currentP,
                            ne[DOWN_LEFT], visitedCellsNe, validCells, grayScaleImg);

        // cerr<<" validCells[UP] " <<validCells[UP]<<
        //     " validCells[DOWN] " <<validCells[DOWN]<<
        //     " validCells[LEFT] " <<validCells[LEFT]<<
        //     " validCells[RIGHT] " <<validCells[RIGHT]<<
        //     " validCells[TOP_LEFT] " <<validCells[TOP_LEFT]<<
        //     " validCells[TOP_RIGHT] " <<validCells[TOP_RIGHT]<<
        //     " validCells[DOWN_RIGHT] " <<validCells[DOWN_RIGHT]<<
        //     " validCells[DOWN_LEFT] " <<validCells[DOWN_LEFT]<<endl;

        int max = 0;
        int index_max = -1;
        // find max-dist un-visited neighboar
        for (int i = 0; i < ne.size(); i++)
        {

            if (!visitedCellsNe[i] && validCells[i])
            {

                int dist = scores[i] + distanceTransform.at<int>(ne[i].y, ne[i].x);
                if (dist > max)
                {

                    max = dist;
                    index_max = i;
                }
            }
        }

        // all neighboards are visited, pick on with the nearest

        int min = 0;
        int index_min = -1;

        if (index_max != -1)
        {

            NeighborCell = ne[index_max];
            return true;
        }

        return false;
    }

private:
    
    bool  debug_ = false;    
};

#endif