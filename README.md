## rrt_star
# Background
This code implements RRT* algortihm, for the Autonomous Robots course project.

Team Name: F1Racers

Members:
1. Surya S Dwivedi
2. Ishank Arora

# Insturctions:
Below are the instructions to run the code. Note that you need to add the map file experiment.txt inside the maps folder in the simulator

1. Replace all the files in src/navigation in the course starter code by the above files
2. Run make from the starter code directory
3. Start simulator with the proper map(experiment.txt) in the config file 
4. Start web socket
5. Run ./bin/navigation from inside the starter code directory and choose start and end points in the web viz

Note that the code takes some time to construct the graph if the number of iterations are more than >10000, so please be patient ! You can adjust rrt_iterations parameters in navigation.cc to change number of iterations.

# Results

The tree is shown in black, and path computed using A* is shown in red.

5000 iterations: 

![image](iter5000.png)


10000 iterations:

![image](iter10000.png)

20000 iteartions:

![image](iter20000.png)

The graph for the path cost obtained using A* at various iterations of RRT*. Note the high variability at the lower iterations as the graph is not optimal, as the number of iterations increase, cost becomes less variable.

![image](graph.png)
