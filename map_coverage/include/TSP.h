#ifndef TSP_TSP_H
#define TSP_TSP_H
#include <bits/stdc++.h>
#include <vector>
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

class TSP {
private:
    //data members
    vector<vector<float>> adj; //adjacency matrix
    int V; //the size of the list of points
    vector<int> final_ans; //Dynamic array to store the final answer

    //methods
    void calcDistances(vector<cv::Point2f> points);
    int minimum_key(int key[], bool mstSet[]);
    vector<vector<int>> MST(int parent[], vector<vector<float>> graph);
    vector<vector<int>> primMST(vector<vector<float>> graph);
    void DFS(int** edges_list,int num_nodes,int starting_vertex,bool* visited_nodes);

public:
    TSP(int size);
    ~TSP(){}
    vector<cv::Point2f> calculatePath(vector<cv::Point2f> points);
};


#endif //TSP_TSP_H
