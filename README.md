# PathMazeProjectCE
Second Semester Project done by Computer Engineering students @KU using Qt framework and C++, and built with qmake in Qt Creator.

# Pathfinding Algorithms
The algorithms implemented here are the following:

1.A*
2.Dijkstra
3.BFS
4.DFS

Apart from using Qt data types, we've used multithreading (a technique where a program is divided into smaller units of execution called threads. Each thread runs independently but shares resources like memory, allowing tasks to be performed simultaneously. This helps improve performance by utilizing multiple CPU cores efficiently) for real-time simulation and visualization. With multithreading, we can change the speed of the simulation dynamically(i.e. while the simulation is running). Also, we've added the feature of "resume" and "pause" pathfinding and pathtracing in our application, which makes it useful for teaching and learning purposes!

# GUI
The GUI was created using Qt Creator. There are two main tabs:

## Simulation:
Selecting the algorithm, placing the start and goal, and adding/removing obstacles.

![Animation_Simulation](https://github.com/user-attachments/assets/910c2b71-fc63-482f-af6b-1a29f2f1a38b)

## Visualization:
Increasing the number of nodes horizontally and vertically, and changing the size of these nodes.
![Animation_Visualization2](https://github.com/user-attachments/assets/d493d017-b5be-42fe-b4df-61642cced8d6)

