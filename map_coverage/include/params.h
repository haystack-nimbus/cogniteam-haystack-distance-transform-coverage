#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <angles/angles.h>

using namespace cv;
using namespace std;


#ifndef PARAMS_H
#define PARAMS_H

#define UNKNOWN -1
#define FREE 0
#define BLOCKED 100

#define LARGE_NUM 10000

#define VISITED 255
#define NOT_VISITED 0

const int free_space = 254;


struct GOAL_WITH_SCORE
{

    cv::Point center;
    float score;
    float angle;
};

struct Frontier
{

    vector<cv::Point> contour;
    cv::Point center; //pix
    float distFromPosition;
};

enum Direction
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    TOP_RIGHT = 4,
    TOP_LEFT = 5,
    DOWN_RIGHT = 6,
    DOWN_LEFT = 7
};

enum COVERAGE_STATE
{   
    COVERAGE,
    COVERAGE_BY_STRAIGHT_LINES,
    COVERAGE_DONE,
    BACK_TO_STARTING_LOCATION,
    ERROR_COVERAGE
    
   
};

enum EXPLORE_STATE
{   
    IDLE,
    NAV_TO_NEXT_FRONTIER,
    FINISH_EXPLORE,
    ERROR_EXPLORE    
    
   
};

#endif