#include "TSP.h"

TSP::TSP(int size) {
    this->V = size;
    adj.resize(size);
    for(int i = 0 ; i < size ; ++i)
    {
        //Grow Columns by n
        adj[i].resize(size);
    }
}

void TSP::calcDistances(vector<cv::Point2f> points) {
    for(int i = 0; i < points.size(); i++) {
        for(int j = 0; j < points.size(); j++) {
            float x1 = points[i].x;
            float y1 = points[i].y;
            float x2 = points[j].x;
            float y2 = points[j].y;
            float res = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
            adj[i][j] = res;
        }
    }
}

int TSP::minimum_key(int key[], bool mstSet[])
{
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
        if (mstSet[v] == false && key[v] < min)
            min = key[v], min_index = v;

    return min_index;
}

vector<vector<int>> TSP::MST(int parent[], vector<vector<float>> graph)
{
    vector<vector<int>> v;
    for (int i = 1; i < V; i++)
    {
        vector<int> p;
        p.push_back(parent[i]);
        p.push_back(i);
        v.push_back(p);
        p.clear();
    }
    return v;
}

// getting the Minimum Spanning Tree from the given graph
// using Prim's Algorithm
vector<vector<int>> TSP::primMST(vector<vector<float>> graph)
{
    int parent[V];
    int key[V];

    // to keep track of vertices already in MST
    bool mstSet[V];

    // initializing key value to INFINITE & false for all mstSet
    for (int i = 0; i < V; i++)
        key[i] = INT_MAX, mstSet[i] = false;

    // picking up the first vertex and assigning it to 0
    key[0] = 0;
    parent[0] = -1;

    // The Loop
    for (int count = 0; count < V - 1; count++)
    {
        // checking and updating values wrt minimum key
        int u = minimum_key(key, mstSet);
        mstSet[u] = true;
        for (int v = 0; v < V; v++)
            if (graph[u][v] && mstSet[v] == false && graph[u][v] < key[v])
                parent[v] = u, key[v] = graph[u][v];
    }
    vector<vector<int>> v;
    v = MST(parent, graph);
    return v;
}

// getting the preorder walk of the MST using DFS
void TSP::DFS(int** edges_list,int num_nodes,int starting_vertex,bool* visited_nodes)
{
    // adding the node to final answer
    final_ans.push_back(starting_vertex);

    // checking the visited status
    visited_nodes[starting_vertex] = true;

    // using a recursive call
    for(int i=0;i<num_nodes;i++)
    {
        if(i==starting_vertex)
        {
            continue;
        }
        if(edges_list[starting_vertex][i]==1)
        {
            if(visited_nodes[i])
            {
                continue;
            }
            DFS(edges_list,num_nodes,i,visited_nodes);
        }
    }
}

vector<cv::Point2f> TSP::calculatePath(vector<cv::Point2f> points) {
    // initial graph
    calcDistances(points);

    vector<vector<int>> v;
    v.resize(V);
    for(int i = 0 ; i < V ; ++i)
    {
        //Grow Columns by n
        v[i].resize(V);
    }

    // getting the output as MST
    v = primMST(adj);

    // creating a dynamic matrix
    int** edges_list = new int*[V];
    for(int i=0;i<V;i++)
    {
        edges_list[i] = new int[V];
        for(int j=0;j<V;j++)
        {
            edges_list[i][j] = 0;
        }
    }

    // setting up MST as adjacency matrix
    for(int i=0;i<v.size();i++)
    {
        int first_node = v[i][0];
        int second_node = v[i][1];
        edges_list[first_node][second_node] = 1;
        edges_list[second_node][first_node] = 1;
    }

    // a checker function for the DFS
    bool* visited_nodes = new bool[V];
    for(int i=0;i<V;i++)
    {
        bool visited_node;
        visited_nodes[i] = false;
    }

    //performing DFS
    DFS(edges_list,V,0,visited_nodes);

    // adding the source node to the path
    final_ans.push_back(final_ans[0]);

    vector<cv::Point2f> result;

    for(int i = 0; i < final_ans.size(); i++) {
        result.push_back(points[final_ans[i]]);
    }
    return result;
}

