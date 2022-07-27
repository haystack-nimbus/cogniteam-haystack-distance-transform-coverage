#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <angles/angles.h>


#include <iostream>
#include "AStar.h"

#include "params.h"

using namespace cv;
using namespace std;


#ifndef ASTAR_PATH_MANAGER_H
#define ASTAR_PATH_MANAGER_H

class AstarPathManager
{
public:
    AstarPathManager() {}

    ~AstarPathManager() {}

    void getRosPath(const Mat& map, 
        const cv::Point& start, 
        const cv::Point& end, vector<cv::Point>& pathOutput,
        int dist_between_points){
        

        pathOutput.clear();
        
        AStar::Generator generator(dist_between_points);    
        generator.setHeuristic(AStar::Heuristic::manhattan);
        generator.setDiagonalMovement(true);

        setMap(generator,map);    
       
        auto path = generator.findPath({start.x, start.y }, {end.x, end.y});

        for(int i = path.size() -1 ; i >= 0; i--) {
            // std::cout << coordinate.x << " " << coordinate.y << "\n";
            pathOutput.push_back(cv::Point(path[i].x, path[i].y));
        }

    }

private:

    void setMap(AStar::Generator& generator, const Mat& map) {


        generator.setWorldSize({map.cols, map.rows});

        for (int y = 0; y < map.rows; y++)
        {
            for (int x = 0; x < map.cols; x++)
            {

                if (map.at<uchar>(y, x) != 254)
                {
                    AStar::Vec2i p;
                    p.x = x;
                    p.y = y;
                    generator.addCollision(p);
                } 
               
            }
        } 
    }


    

    
};

#endif