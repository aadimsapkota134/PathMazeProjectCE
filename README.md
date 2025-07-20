# PathMazeProjectCE
**`Second Semester Project`**  
Computer Engineering students @ **`Kathmandu University`**  
Built with **`C++`** using the **`Qt Framework`** and **`qmake`** in **`Qt Creator`**


## Pathfinding Algorithms
The algorithms implemented here are the following:

* A*
* Dijkstra
* BFS(Breadth-First Search)
* DFS(Depth-First Search)

![Untitled video - Made with Clipchamp](https://github.com/user-attachments/assets/fe2553f3-624e-41bc-ba9b-38fc7ee15b68)

## Maze Generation Algorthims
Users can select difficulty levels: `easy`, `medium`, `hard` or `extreme`. Each maps to a different algorithm:

- **`easy`**: Recursive Backtracking (DFS)
- **`medium`**: Prim's Algorithm
- **`hard`**: Kruskal's Algorithm
- **`Extreme`**: Wilson's Algorithm

<img width="533" height="141" alt="image" src="https://github.com/user-attachments/assets/eab55c76-d43b-490a-8124-75139fd4b4a6" />


## Features

- **Multithreading**  
  Enables real-time simulation and visualization by dividing execution into independent threads. Allows dynamic speed adjustment during runtime.

- **Control Features**  
  Includes **`pause`** and **`resume`** options for both pathfinding and pathtracing simulations. Useful for demonstration and educational purposes.

- **Comparison Tab**  
  Automatically compares algorithms based on:
  - Number of visited nodes
  - Time taken to find path
  - Grid size  
  Results are exported to a **`.csv`** file for possible external analysis in the future.

## GUI Overview

Created with Qt Creator. Structured into three primary tabs:

### 1. **`Simulation`**

- Select pathfinding algorithm  
- Set **`start`** and **`goal`** nodes  
- Add or remove **`obstacles`**

![Animation_Simulation](https://github.com/user-attachments/assets/910c2b71-fc63-482f-af6b-1a29f2f1a38b)

### 2. **`Visualization`**

- Adjust horizontal and vertical **`node`** count  
- Modify **`node size`**

![Animation_Visualization2](https://github.com/user-attachments/assets/d493d017-b5be-42fe-b4df-61642cced8d6)

### 3. **`Comparison`**

- Compare all algorithms or a single algorithm across different **`grid sizes`**  
- Delete selected rows or clear all rows from the **`comparison table`**  
  (**`.csv`** file remains unchanged)
![image](https://github.com/user-attachments/assets/73d7ba88-b82a-48f6-98ef-0683eb8dcb8e)


## Play Yourself
 For those who want to test their skills on finding a path in a maze, we've also made a 'Play Yourself' corner so that 
 user can traverse the maze and try to find the path between start and end node. The standard arrow keys and WASD keys 
 can be used for traversing in up-down-left-right direction in the grid.
 <img width="1798" height="922" alt="image" src="https://github.com/user-attachments/assets/166c8192-a21a-4b42-ac7f-111439a4492b" />


Feel free to contact me at aadimsapkota0@gmail.com for any details or enquiries regarding our project.


