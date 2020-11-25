# Obstacle Avoidance using Dijkstras Algorithm - C++ implementation

The current position of the vehicle and the goal or final position, number of obstacles and the position of the obstacles is entered by the user.
4 nodes (B’,B’1 B’2B’3 ) are formed around the obstacle which are a determined margin away from the obstacle and is plotted at different angles as shown below

![Alt text](Resources/nodes.png?raw=true "Title")

All the nodes are connected if the edges formed are a predetermined margin away from any obstacle.
A connected and weighted graph is formed with the starting position and the goal and Dijkstra’s algorithm is used to find the shortest path.
