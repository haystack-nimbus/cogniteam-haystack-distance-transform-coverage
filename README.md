# distance_transform_coverage_exploration
## Introduction
This package implement the algoritm by Yakir Huri.
The algorithm consists of three steps:
In the first step, the robot will choose the safest goal on the map, ie the place furthest from obstacles/walls. The robot will navigate to this goal

In the second step, the robot will run the EXPLORATION algorithm.
This algorithm is an iterative algorithm. In each iteration all the possible Frontiers on the map are calculated.
In each iteration the robot will navigate to a goal from the Frontier that received the highest score.
If there is no frontiers, or if the map-score (calculated in each iteration) above the threshold, the EXPLORATION will end.

In the last step the robot will run the COVERAGE algorithm.
By using the DISTANCE-TRANSFORM operation the robot will try to calculate an optimal coverage-path in order to cover the whole map.
After calculating the path, the robot will drive on this path by sending goals in the MOVE-BASE API.



### parameters
#### distance_between_goals_m
the min distance bewteen goals inisde of the coverage path in meters.
#### robot_raduis
robot radius in meters
#### exploration_score
the min score threshold of the map that represnt how closed the map is (contour). 

### topics
#### Subscribed Topics

/map

/tf

/tf_static

/move_base_api

#### Published Topics

/move_base_api

![Alt text](distance_transform_coverage_exploration.png?raw=true "Title")
