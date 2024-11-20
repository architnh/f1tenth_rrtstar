
# Motion Planning using RRT and RRT*

## Overview :

<img src=imgs/spline_rrt_star.gif height="360" width="640" > <p></p>
Figure 1: Implementation on real car.


The goal of this project was to explore Sampling based algorithms: RRT and its variants. Through this project, we first need to implement a binary occupancy grid (0 for free, 1 for occupied), and process the Hokuyu 2D Lidar scans at a refresh rate of 25ms. Th figure 1 shown above shows the robust and reliable local path planner updates using the RRT* approach.


## RRT Pseudocode:

![rrt_algo](imgs/rrt_algo.png)

The pseudocode of the basic version of RRT that we shall use for this implementation is listed as above. As for RRT*, we shall use the pseudocode mentioned below.

![rrt_algo](imgs/rrt_star_algo.png)

## F1TENTH RRT vs. Generic RRT
<img src=imgs/rrtvsrrtstar.gif height="360" width="640" > <p></p>
Figure 2: RRT* vs RRT implementation as visualized on RVIZ.

In general, RRT is often used as a global planner where the tree is kept throughout the time steps. Whenever there is a new obstacle, and the occupancy grid changes, the tree will change accordingly. In our case, RRT is used as a local planner for obstacle avoidance. This is due to the fact that we don't have a well-defined starting point and goal point when we're racing on a track and we want to run continuous laps. In our implementation, we are only keeping a tree for the current time step in an area around the car. As speed is of the utmost importance (one update of laserscan every 25 ms), farster the process of finding nodes, and traversing the tree, faster is the implementation.

## RRT vs. RRT* 
A comparison of the output I obtained from experiments with RRT* and RRT algorithm is shown below.<p></p>


### RRT* introduces two major improvements over RRT - <p></p>
1) Each node is associated with its cost. This is a function which measures the distance from the goal point. The goal of the algorithm is to optimize the tree and reduce this cost. <p></p>
2) At each iteration, the vicinity of node being sampled is checked for nearby nodes on the tree which can be rewired to reduce the path traversed. If such nodes exist, they are rewired and the net cost of reaching the node is reduced. 

### Videos- <p></p>
[RRT](https://youtu.be/u7Lv9G6eQF8) &emsp;
[RRT*](https://youtu.be/NfnwbfQKN34) <p></p>

Because of the way the algorithm works, it goes on improving itself with every iteration. This results in a much cleaner, smaller path from the start to the target. Because the distance between two nodes is kept static and predefined for our problem, we can say with certainty that the number of intermediate nodes connecting the start to the goal is much lesser in RRT* as against RRT. RRT* also plans a straighter and cleaner path which is not haphazard. This allowed us to significantly increase the speed of the car during loop racing. It reduced the lap time significantly. I observed a considerable difference in the performance on straights and turns.
## References:
https://arxiv.org/pdf/1105.1186.pdf

