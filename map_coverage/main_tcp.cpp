#include <bits/stdc++.h>
#include <vector>
#include <opencv2/core/types.hpp>
#include "TSP.h"

using namespace std;
using namespace cv;


int main()
{
    cv::Point2f a(1.f, 2.f), b(3.f, 4.f),c(6.f, 3.f), d(5.f, 20.f), e(11.f, 7.f);
    vector<cv::Point2f> listOfPoints;
    listOfPoints.push_back(a);
    listOfPoints.push_back(b);
    listOfPoints.push_back(c);
    listOfPoints.push_back(d);
    listOfPoints.push_back(e);
    TSP* tsp = new TSP(5);

    vector<cv::Point2f> res = tsp->calculatePath(listOfPoints);
    cout<<"Printing the path"<<endl;
    for(int i = 0; i < res.size(); i++) {
        cout<<"x: " + to_string(res[i].x) + "," + "y: "  + to_string(res[i].y)<<endl;
    }
    return 0;
}