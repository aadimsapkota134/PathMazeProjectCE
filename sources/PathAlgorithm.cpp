#include "PathAlgorithm.h"
#include "QtConcurrent/qtconcurrentrun.h"
#include <iostream>
#include <queue>
#include <map>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <QtConcurrent>
#include <QFuture>
#include <stack>
#include <list>
#include <QDebug> // Include for qDebug()
#include <QDateTime> // Include for QDateTime::currentMSecsSinceEpoch()
#include <QThread> // Include for QThread::currentThreadId()

//Constructor
PathAlgorithm::PathAlgorithm(QObject* parent): QObject (parent)
{
    // The algorithm is not running at startup
    running = false;
    simulationOnGoing = false;
    endReached = false;

    speedVisualization = 250;
    qDebug() << "PathAlgorithm: Constructor called. Main thread ID:" << QThread::currentThreadId();
}

//Destructor
PathAlgorithm::~PathAlgorithm()
{

}

//Getters/Setters: current Algorithm from gridView
ALGOS PathAlgorithm::getCurrentAlgorithm() const
{
    return currentAlgorithm;
}

//Getters/Setters: current Algorithm from gridView
void PathAlgorithm::setCurrentAlgorithm(ALGOS algorithm)
{
    this->currentAlgorithm = algorithm;
}

void PathAlgorithm::setSpeedVizualization(int speed)
{
    this->speedVisualization = speed;
}

// Getters/Setters: Simulation on going
void PathAlgorithm::setSimulationOnGoing(bool onGoing)
{
    this->simulationOnGoing = onGoing;
}
// Implement the new method:
void PathAlgorithm::setGridNodes(const grid& newGridNodes, int width, int height)
{
    this->gridNodes = newGridNodes; // Copies the entire grid struct, including the Nodes vector and start/end indices
    this->widthGrid = width;        // Update PathAlgorithm's internal width
    this->heightGrid = height;      // Update PathAlgorithm's internal height
    qDebug() << "PathAlgorithm: Internal grid dimensions updated to " << this->widthGrid << "x" << this->heightGrid;
    qDebug() << "PathAlgorithm: Internal gridNodes.Nodes.size() is now: " << this->gridNodes.Nodes.size();
    qDebug() << "PathAlgorithm: Internal startIndex: " << this->gridNodes.startIndex << ", endIndex: " << this->gridNodes.endIndex;

    // IMPORTANT: Also reset the visited/nextUp flags on the new grid, just in case
    for(Node& node: this->gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false;
    }
}

std::vector<Node> PathAlgorithm::retrieveNeighborsGrid(const grid& gridNodes, const Node& currentNode, int widthGrid, int heightGrid)
{

    std::vector<Node> neighbors;

    // right: adding +1 to x:
    if (currentNode.xCoord + 1 <= widthGrid)
    {
        int rightIndex = coordToIndex(currentNode.xCoord + 1, currentNode.yCoord, widthGrid);
        neighbors.push_back(gridNodes.Nodes[rightIndex]);

    }

    // down: adding -1 to y:
    if (currentNode.yCoord - 1 >= 1)
    {
        int downIndex = coordToIndex(currentNode.xCoord, currentNode.yCoord -1, widthGrid);
        neighbors.push_back(gridNodes.Nodes[downIndex]);
    }

    // left: adding -1 to x:
    if (currentNode.xCoord - 1 >= 1)
    {
        int leftIndex = coordToIndex(currentNode.xCoord - 1, currentNode.yCoord, widthGrid);
        neighbors.push_back(gridNodes.Nodes[leftIndex]);
    }

    // up: adding +1 to y:
    if (currentNode.yCoord + 1 <= heightGrid)
    {
        int upIndex = coordToIndex(currentNode.xCoord, currentNode.yCoord + 1, widthGrid);
        neighbors.push_back(gridNodes.Nodes[upIndex]);
    }

    return neighbors;
}

QString PathAlgorithm::algorithmToString(ALGOS algo) {
    switch (algo) {
    case BFS: return "BFS";
    case DFS: return "DFS";
    case DIJKSTRA: return "DIJKSTRA";
    case ASTAR: return "ASTAR";
    case BACKTRACK: return "BACKTRACK";
    case NOALGO: return "NOALGO";
    default: return "UNKNOWN_ALGO";
    }
}
void PathAlgorithm::checkGridNode(grid gridNodes, int heightGrid, int widthGrid)
{
    // Display grid
    std::cerr << "State of grid node \n";
    int countVisited = 0; int countObstacle = 0; int countFree = 0;

    for (Node node: gridNodes.Nodes)
    {
        std::cerr << "(" << node.xCoord << ", " <<  node.yCoord << "): ";

        if (node.visited){std::cerr << ": V"; countVisited++;}

        if (node.obstacle){std::cerr << ": O"; countObstacle++;}
        else{std::cerr << ": F"; countFree++;}

        if (node.xCoord == widthGrid){std::cerr << " \n";}
        else{std::cerr << " | ";}

    }
    std::cerr << "Totals: " << "Visited: " << countVisited
              << " - Obstacles: " << countObstacle
              << " - Free:" << countFree << "\n";

    // Check size of vector
    if (static_cast<int>(gridNodes.Nodes.size()) != static_cast<int>(heightGrid * widthGrid))
    {std::cerr << "Number of nodes in gridNodes: " << gridNodes.Nodes.size() << " vs " << heightGrid * widthGrid << " [ISSUE] \n";}
    else{std::cerr << "Number of nodes in gridNodes: " << gridNodes.Nodes.size() << "\n";}

}

void PathAlgorithm::runAlgorithm(ALGOS algorithm)
{
    simulationOnGoing=true;
    running=true;
    qDebug() << "PathAlgorithm: runAlgorithm called. Thread ID:" << QThread::currentThreadId();

    switch (algorithm) {
    case BFS:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performBFSAlgorithm, this);
        break;
    case DFS:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performDFSAlgorithm, this);
        break;
    case DIJKSTRA:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performDijkstraAlgorithm, this);
        break;
    case ASTAR:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performAStarAlgorithm, this);
        break;
    case BACKTRACK:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performRecursiveBackTrackerAlgorithm, this);
        break;
    case PRIMS:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performPrimsMazeAlgorithm, this);
        break;
    case KRUSKAL:
        futureOutput = QtConcurrent::run(&pool, &PathAlgorithm::performKruskalsMazeAlgorithm, this);
        break;
    case NOALGO:
        std::cerr <<"NO ALGO \n";
    default:
        break;

    }

}

void PathAlgorithm::resumeAlgorithm()
{
    running = true;
    futureOutput.resume();
    qDebug() << "PathAlgorithm: futureOutput.resume() called.";
}

void PathAlgorithm::pauseAlgorithm()
{
    running = false;
    futureOutput.suspend();
    qDebug() << "PathAlgorithm: futureOutput.suspend() called.";
}

void PathAlgorithm::stopAlgorithm()
{
    running = false;
    futureOutput.cancel();
    qDebug() << "PathAlgorithm: futureOutput.cancel() called.";
}

// BFS Algorithm
// In PathAlgorithm.cpp

void PathAlgorithm::performBFSAlgorithm(QPromise<int>& promise)
{
    qDebug() << "BFS: Algorithm started in worker thread:" << QThread::currentThreadId();

    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        // If cancelled before starting, emit with 0, 0
        emit pathfindingSearchCompleted(0, 0);
        return;
    }

    bool reachEnd = false;
    std::queue<Node*> nextNodesQueue; // Use Node* to avoid copying large Node objects
    std::map<int, int> parentMap; // Map child index to parent index for path reconstruction

    // Starting point
    Node* startNode = &(gridNodes.Nodes[gridNodes.startIndex]);
    nextNodesQueue.push(startNode);
    gridNodes.Nodes[gridNodes.startIndex].visited = true; // Mark start as visited immediately
    parentMap[gridNodes.startIndex] = -1; // No parent for start node

    int nodesVisitedCount = 0; // Counter for visited nodes

    while(!nextNodesQueue.empty())
    {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "BFS: Algorithm cancelled during loop.";
            // Emit with current stats on cancel
            emit pathfindingSearchCompleted(nodesVisitedCount, 0);
            return;
        }

        Node* currentNode = nextNodesQueue.front();
        nextNodesQueue.pop();
        nodesVisitedCount++; // Increment when a node is dequeued and processed

        int currentIndex = coordToIndex(currentNode->xCoord, currentNode->yCoord, widthGrid);

        // Update this node as visited in the gridView (if it's not the start/end and not already visited)
        // The start node is already marked visited, so we don't need to re-emit for it here.
        // For other nodes, we emit VISIT when we process them.
        if (currentIndex != gridNodes.startIndex && currentIndex != gridNodes.endIndex) {
            emit updatedScatterGridView(VISIT, currentIndex);
        }

        // Check if goal is reached
        if (currentIndex == gridNodes.endIndex)
        {
            std::cerr << "Reached end \n";
            reachEnd = true;
            break; // Path found, exit search loop
        }
        // Note: retrieveNeighborsGrid returns copies...

        // Manual neighbor retrieval for BFS/DFS (more typical)
        int dx[] = {0, 0, 1, -1}; // Up, Down, Right, Left
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            int neighborX = currentNode->xCoord + dx[i];
            int neighborY = currentNode->yCoord + dy[i];

            if (neighborX >= 1 && neighborX <= widthGrid &&
                neighborY >= 1 && neighborY <= heightGrid)
            {
                int neighborIndex = coordToIndex(neighborX, neighborY, widthGrid);
                Node* neighborNode = &(gridNodes.Nodes[neighborIndex]);

                if (!neighborNode->visited && !neighborNode->obstacle) {
                    neighborNode->visited = true; // Mark as visited when added to queue
                    nextNodesQueue.push(neighborNode);
                    parentMap[neighborIndex] = currentIndex; // Store parent

                    // Update this node as 'next' in the gridView
                    if (neighborIndex != gridNodes.endIndex) { // Don't mark end as 'next' if found
                        emit updatedScatterGridView(NEXT, neighborIndex);
                    }
                }
            }
        }


        // Time and checking for stop from running button
        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
    }

    int pathLength = 0;
    // If the end is reached, we output the path and calculate pathLength
    if (reachEnd){
        endReached = true;

        // Path reconstruction and pathLength calculation
        std::vector<int> pathIndices;
        int currentPathIndex = gridNodes.endIndex;
        while (currentPathIndex != -1) {
            pathIndices.insert(pathIndices.begin(), currentPathIndex); // Insert at beginning to reverse
            if (parentMap.count(currentPathIndex)) {
                currentPathIndex = parentMap[currentPathIndex];
            } else {
                currentPathIndex = -1; // Should not happen if path is valid, but good for safety
            }
        }
        pathLength = pathIndices.size() > 0 ? pathIndices.size() - 1 : 0; // Path length is edges (nodes - 1)

        emit pathfindingSearchCompleted(nodesVisitedCount, pathLength); // Emit signal when search is complete
        qDebug() << "BFS: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        // Visualization of the path
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.endIndex].xCoord, gridNodes.Nodes[gridNodes.endIndex].yCoord), true, true); // Clear and start line from end

        for (size_t i = pathIndices.size() - 1; i > 0; --i) { // Iterate from goal back to start (excluding start)
            promise.suspendIfRequested();
            if (promise.isCanceled()) {
                qDebug() << "BFS: Algorithm cancelled during visualization.";
                break;
            }
            int pathNodeIndex = pathIndices[i];
            emit updatedScatterGridView(PATH, pathNodeIndex);
            emit updatedLineGridView(QPointF(gridNodes.Nodes[pathNodeIndex].xCoord, gridNodes.Nodes[pathNodeIndex].yCoord), true, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
        // Ensure start node is also part of the line if not already
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{
        endReached = false;
        emit pathfindingSearchCompleted(nodesVisitedCount, 0); // Emit signal even if path not found
        qDebug() << "BFS: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    // Reset visited flags for next run
    for(Node& node: gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false; // Also reset nextUp for BFS/DFS
    }

    qDebug() << "DEBUG: " << algorithmToString(currentAlgorithm) << ": About to emit algorithmCompleted()."; // Add this line
    emit algorithmCompleted();

    qDebug() << "DEBUG: " << algorithmToString(currentAlgorithm) << ": Emitted algorithmCompleted() and pathfindingSearchCompleted()."; // Add this line
}

// DFS Algorithm
// In PathAlgorithm.cpp

void PathAlgorithm::performDFSAlgorithm(QPromise<int>& promise)
{
    qDebug() << "DFS: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled())
    {
        emit pathfindingSearchCompleted(0, 0); // Emit with current stats on cancel
        return;
    }
    bool reachEnd = false;
    std::stack<Node*> nextNodesStack; // Use Node*
    std::map<int, int> parentMap; // Map child index to parent index

    Node* startNode = &(gridNodes.Nodes[gridNodes.startIndex]);
    nextNodesStack.push(startNode);
    gridNodes.Nodes[gridNodes.startIndex].visited = true; // Mark start as visited
    parentMap[gridNodes.startIndex] = -1; // No parent for start node

    int nodesVisitedCount = 0; // Counter for visited nodes

    while(!nextNodesStack.empty())
    {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "DFS: Algorithm cancelled during loop.";
            emit pathfindingSearchCompleted(nodesVisitedCount, 0); // Emit with current stats on cancel
            return;
        }

        Node* currentNode = nextNodesStack.top();
        nextNodesStack.pop();
        nodesVisitedCount++; // Increment when a node is popped and processed

        int currentIndex = coordToIndex(currentNode->xCoord, currentNode->yCoord, widthGrid);

        if (currentIndex != gridNodes.startIndex && currentIndex != gridNodes.endIndex) {
            emit updatedScatterGridView(VISIT, currentIndex);
        }

        if (currentIndex == gridNodes.endIndex)
        {
            std::cerr << "Reached end \n";
            reachEnd = true;
            break;
        }

        // Manual neighbor retrieval for DFS
        int dx[] = {0, 0, 1, -1}; // Up, Down, Right, Left
        int dy[] = {1, -1, 0, 0};

        for (int i = 0; i < 4; ++i) {
            int neighborX = currentNode->xCoord + dx[i];
            int neighborY = currentNode->yCoord + dy[i];

            if (neighborX >= 1 && neighborX <= widthGrid &&
                neighborY >= 1 && neighborY <= heightGrid)
            {
                int neighborIndex = coordToIndex(neighborX, neighborY, widthGrid);
                Node* neighborNode = &(gridNodes.Nodes[neighborIndex]);

                if (!neighborNode->visited && !neighborNode->obstacle) {
                    neighborNode->visited = true; // Mark as visited when added to stack
                    nextNodesStack.push(neighborNode);
                    parentMap[neighborIndex] = currentIndex; // Store parent

                    if (neighborIndex != gridNodes.endIndex) {
                        emit updatedScatterGridView(NEXT, neighborIndex);
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
    }

    int pathLength = 0;
    if (reachEnd){
        endReached = true;

        // Path reconstruction and pathLength calculation
        std::vector<int> pathIndices;
        int currentPathIndex = gridNodes.endIndex;
        while (currentPathIndex != -1) {
            pathIndices.insert(pathIndices.begin(), currentPathIndex);
            if (parentMap.count(currentPathIndex)) {
                currentPathIndex = parentMap[currentPathIndex];
            } else {
                currentPathIndex = -1;
            }
        }
        pathLength = pathIndices.size() > 0 ? pathIndices.size() - 1 : 0;

        emit pathfindingSearchCompleted(nodesVisitedCount, pathLength);
        qDebug() << "DFS: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        // Visualization of the path
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.endIndex].xCoord, gridNodes.Nodes[gridNodes.endIndex].yCoord), true, true);

        for (size_t i = pathIndices.size() - 1; i > 0; --i) {
            promise.suspendIfRequested();
            if (promise.isCanceled()) {
                qDebug() << "DFS: Algorithm cancelled during visualization.";
                break;
            }
            int pathNodeIndex = pathIndices[i];
            emit updatedScatterGridView(PATH, pathNodeIndex);
            emit updatedLineGridView(QPointF(gridNodes.Nodes[pathNodeIndex].xCoord, gridNodes.Nodes[pathNodeIndex].yCoord), true, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{
        endReached = false;
        emit pathfindingSearchCompleted(nodesVisitedCount, 0);
        qDebug() << "DFS: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    // Reset visited flags for next run
    for(Node& node: gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false; // Also reset nextUp for BFS/DFS
    }

    emit algorithmCompleted();
    qDebug() << "DFS: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}
void PathAlgorithm::performDijkstraAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Dijkstra: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        emit pathfindingSearchCompleted(0, 0);
        return;
    }

    // Reset node properties for a new run
    for(Node& node: gridNodes.Nodes)
    {
        node.neighbours.clear();
        FillNeighboursNode(node); // Ensure neighbors are filled
        node.localGoal      = INFINITY;
        node.parent         = nullptr;
        node.visited        = false; // Reset visited flag
    }

    auto distance = [](Node* a, Node* b)
    {
        return sqrtf(   (a->xCoord - b->xCoord) * (a->xCoord - b->xCoord)
                     +(a->yCoord - b->yCoord) * (a->yCoord - b->yCoord));
    };

    Node* nodeStart = &(gridNodes.Nodes[gridNodes.startIndex]);
    Node* nodeEnd = &(gridNodes.Nodes[gridNodes.endIndex]);

    nodeStart->localGoal = 0.0f;

    // Use std::priority_queue for Dijkstra
    std::priority_queue<Node*, std::vector<Node*>, CompareNodesDijkstra> nodesToTest;
    nodesToTest.push(nodeStart);

    int nodesVisitedCount = 0; // Counter for visited nodes

    while(!nodesToTest.empty())
    {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "Dijkstra: Algorithm cancelled during loop.";
            emit pathfindingSearchCompleted(nodesVisitedCount, 0);
            return;
        }

        Node* nodeCurrent = nodesToTest.top();
        nodesToTest.pop();

        // If this node has already been visited (meaning we found a shorter path to it earlier), skip it.
        if (nodeCurrent->visited) {
            continue;
        }

        nodeCurrent->visited = true; // Mark as visited after extracting from PQ
        nodesVisitedCount++; // Increment when a node is processed (removed from open list)

        int indexCurrent = coordToIndex(nodeCurrent->xCoord, nodeCurrent->yCoord, widthGrid);
        if (indexCurrent != gridNodes.startIndex && indexCurrent != gridNodes.endIndex) {
            emit updatedScatterGridView(VISIT, indexCurrent);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));

        if (nodeCurrent == nodeEnd) { // Goal reached
            break;
        }

        for (Node* nodeNeighbour: nodeCurrent->neighbours)
        {
            if(!nodeNeighbour->obstacle) // Only consider non-obstacle neighbors
            {
                float potentialLowerGoal = nodeCurrent->localGoal + distance(nodeCurrent, nodeNeighbour);
                if (potentialLowerGoal < nodeNeighbour->localGoal){
                    nodeNeighbour->parent = nodeCurrent;
                    nodeNeighbour->localGoal = potentialLowerGoal;

                    // Always push to PQ if a better path is found, even if potentially "visited" by a longer path
                    nodesToTest.push(nodeNeighbour);

                    // Only emit NEXT if it's not the end node and it hasn't been visited in a finalized path yet
                    if (!nodeNeighbour->visited && nodeNeighbour != nodeEnd) {
                        emit updatedScatterGridView(NEXT, coordToIndex(nodeNeighbour->xCoord, nodeNeighbour->yCoord, widthGrid));
                    }
                }
            }
        }
    }

    int pathLength = 0;
    if (nodeEnd->parent != nullptr){ // Path found
        // Path reconstruction and pathLength calculation
        std::vector<int> pathIndices;
        Node* currentPathNode = nodeEnd;
        while (currentPathNode != nullptr) {
            pathIndices.insert(pathIndices.begin(), coordToIndex(currentPathNode->xCoord, currentPathNode->yCoord, widthGrid));
            currentPathNode = currentPathNode->parent;
        }
        pathLength = pathIndices.size() > 0 ? pathIndices.size() - 1 : 0;

        emit pathfindingSearchCompleted(nodesVisitedCount, pathLength);
        qDebug() << "Dijkstra: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        // Visualization of the path
        emit updatedLineGridView(QPointF(nodeEnd->xCoord, nodeEnd->yCoord), true, true);

        for (size_t i = pathIndices.size() - 1; i > 0; --i) { // Iterate from goal back to start (excluding start)
            promise.suspendIfRequested();
            if (promise.isCanceled()) {
                qDebug() << "Dijkstra: Algorithm cancelled during visualization.";
                break;
            }
            int pathNodeIndex = pathIndices[i];
            emit updatedScatterGridView(PATH, pathNodeIndex);
            emit updatedLineGridView(QPointF(gridNodes.Nodes[pathNodeIndex].xCoord, gridNodes.Nodes[pathNodeIndex].yCoord), true, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{ // No path found
        endReached = false;
        emit pathfindingSearchCompleted(nodesVisitedCount, 0);
        qDebug() << "Dijkstra: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    // Reset visited flags for next run
    for(Node& node: gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false; // Not strictly used in Dijkstra, but good to reset
    }

    emit algorithmCompleted();
    qDebug() << "Dijkstra: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}


void PathAlgorithm::performAStarAlgorithm(QPromise<int>& promise)
{
    qDebug() << "AStar: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        emit pathfindingSearchCompleted(0, 0);
        return;
    }

    // Reset node properties for a new run
    for(Node& node: gridNodes.Nodes)
    {
        node.neighbours.clear();
        FillNeighboursNode(node);
        node.globalGoal     = INFINITY;
        node.localGoal      = INFINITY;
        node.parent         = nullptr;
        node.visited        = false; // Reset visited flag
    }

    auto distance = [](Node* a, Node* b)
    {
        return sqrtf(   (a->xCoord - b->xCoord) * (a->xCoord - b->xCoord)
                     +(a->yCoord - b->yCoord) * (a->yCoord - b->yCoord));
    };

    // Improved heuristic for 4-directional grid: Manhattan Distance
    auto heuristic = [](Node* a, Node* b){
        return fabsf(a->xCoord - b->xCoord) + fabsf(a->yCoord - b->yCoord);
    };

    Node* nodeStart = &(gridNodes.Nodes[gridNodes.startIndex]);
    Node* nodeEnd = &(gridNodes.Nodes[gridNodes.endIndex]);

    nodeStart->localGoal = 0.0f;
    nodeStart->globalGoal = heuristic(nodeStart, nodeEnd);

    // Use std::priority_queue for A*
    std::priority_queue<Node*, std::vector<Node*>, CompareNodesAStar> nodesToTest;
    nodesToTest.push(nodeStart);

    int nodesVisitedCount = 0; // Counter for visited nodes

    while(!nodesToTest.empty())
    {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "AStar: Algorithm cancelled during loop.";
            emit pathfindingSearchCompleted(nodesVisitedCount, 0);
            return;
        }

        Node* nodeCurrent = nodesToTest.top();
        nodesToTest.pop();

        // If this node has already been visited (meaning we found a shorter path to it earlier), skip it.
        if (nodeCurrent->visited) {
            continue;
        }

        nodeCurrent->visited = true; // Mark as visited after extracting from PQ
        nodesVisitedCount++; // Increment when a node is processed (removed from open list)

        int indexCurrent = coordToIndex(nodeCurrent->xCoord, nodeCurrent->yCoord, widthGrid);
        if (indexCurrent != gridNodes.startIndex && indexCurrent != gridNodes.endIndex) {
            emit updatedScatterGridView(VISIT, indexCurrent);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));

        if (nodeCurrent == nodeEnd) { // Goal reached
            break;
        }

        for (Node* nodeNeighbour: nodeCurrent->neighbours)
        {
            if(!nodeNeighbour->obstacle) // Only consider non-obstacle neighbors
            {
                float potentialLowerGoal = nodeCurrent->localGoal + distance(nodeCurrent, nodeNeighbour);
                if (potentialLowerGoal < nodeNeighbour->localGoal){
                    nodeNeighbour->parent = nodeCurrent;
                    nodeNeighbour->localGoal = potentialLowerGoal;
                    nodeNeighbour->globalGoal = nodeNeighbour->localGoal + heuristic(nodeNeighbour, nodeEnd);

                    // Always push to PQ if a better path is found, even if potentially "visited" by a longer path
                    nodesToTest.push(nodeNeighbour);

                    // Only emit NEXT if it's not the end node and it hasn't been visited in a finalized path yet
                    if (!nodeNeighbour->visited && nodeNeighbour != nodeEnd) {
                        emit updatedScatterGridView(NEXT, coordToIndex(nodeNeighbour->xCoord, nodeNeighbour->yCoord, widthGrid));
                    }
                }
            }
        }
    }

    int pathLength = 0;
    if (nodeEnd->parent != nullptr){ // Path found
        // Path reconstruction and pathLength calculation
        std::vector<int> pathIndices;
        Node* currentPathNode = nodeEnd;
        while (currentPathNode != nullptr) {
            pathIndices.insert(pathIndices.begin(), coordToIndex(currentPathNode->xCoord, currentPathNode->yCoord, widthGrid));
            currentPathNode = currentPathNode->parent;
        }
        pathLength = pathIndices.size() > 0 ? pathIndices.size() - 1 : 0;

        emit pathfindingSearchCompleted(nodesVisitedCount, pathLength);
        qDebug() << "AStar: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        // Visualization of the path
        emit updatedLineGridView(QPointF(nodeEnd->xCoord, nodeEnd->yCoord), true, true);

        for (size_t i = pathIndices.size() - 1; i > 0; --i) { // Iterate from goal back to start (excluding start)
            promise.suspendIfRequested();
            if (promise.isCanceled()) {
                qDebug() << "AStar: Algorithm cancelled during visualization.";
                break;
            }
            int pathNodeIndex = pathIndices[i];
            emit updatedScatterGridView(PATH, pathNodeIndex);
            emit updatedLineGridView(QPointF(gridNodes.Nodes[pathNodeIndex].xCoord, gridNodes.Nodes[pathNodeIndex].yCoord), true, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{ // No path found
        endReached = false;
        emit pathfindingSearchCompleted(nodesVisitedCount, 0);
        qDebug() << "AStar: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    // Reset visited flags for next run
    for(Node& node: gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false; // Not strictly used in A*, but good to reset
    }

    emit algorithmCompleted();
    qDebug() << "AStar: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}

// Maze generation

void PathAlgorithm::performRecursiveBackTrackerAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Maze: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        emit pathfindingSearchCompleted(0, 0); // Emit with 0,0 on cancel for maze
        return;
    }

    // Initialize all nodes as obstacles (except start/end)
    for (int index = 0; index < widthGrid * heightGrid; index++){
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during init.";
            emit pathfindingSearchCompleted(0, 0); // Emit with 0,0 on cancel for maze
            return;
        }

        // Only set to obstacle if it's not the start or end node
        if (index != gridNodes.startIndex && index != gridNodes.endIndex){
            gridNodes.Nodes[index].obstacle = true;
            emit updatedScatterGridView(FREETOOBSTACLE, index);
        }
    }

    std::stack<Node*> stackUnVisitedNodes;
    // Choose a random starting cell for maze generation
    int randomIndex = rand() % (widthGrid * heightGrid);
    Node* startMazeNode = &(gridNodes.Nodes[randomIndex]);

    stackUnVisitedNodes.push(startMazeNode);
    startMazeNode->visited = true;
    // Ensure the start node for maze generation is not an obstacle
    if (startMazeNode->obstacle) {
        startMazeNode->obstacle = false;
        emit updatedScatterGridView(OBSTACLETOFREE, randomIndex);
    }

    int numberVisitedCells = 1;
    int offset = 2; // For carving paths in a grid, typically move 2 steps
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Initial sleep for maze generation

    while(numberVisitedCells < widthGrid * heightGrid && !stackUnVisitedNodes.empty() ){
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during loop.";
            emit pathfindingSearchCompleted(0, 0); // Emit with 0,0 on cancel for maze
            return;
        }

        Node* currentNode = stackUnVisitedNodes.top();
        //int currentIndex = coordToIndex(currentNode->xCoord, currentNode->yCoord, widthGrid);
        std::vector<int> availableNeighboursDirections; // 0:East, 1:South, 2:West, 3:North

        // Check potential neighbors (2 steps away for carving)
        // East
        int eastNeighborX = currentNode->xCoord + offset;
        int eastPathX = currentNode->xCoord + offset - 1;
        if (eastNeighborX <= widthGrid && !gridNodes.Nodes[coordToIndex(eastNeighborX, currentNode->yCoord, widthGrid)].visited){
            availableNeighboursDirections.push_back(0);
        }
        // South
        int southNeighborY = currentNode->yCoord - offset;
        int southPathY = currentNode->yCoord - offset + 1;
        if (southNeighborY >= 1 && !gridNodes.Nodes[coordToIndex(currentNode->xCoord, southNeighborY, widthGrid)].visited){
            availableNeighboursDirections.push_back(1);
        }
        // West
        int westNeighborX = currentNode->xCoord - offset;
        int westPathX = currentNode->xCoord - offset + 1;
        if (westNeighborX >= 1 && !gridNodes.Nodes[coordToIndex(westNeighborX, currentNode->yCoord, widthGrid)].visited){
            availableNeighboursDirections.push_back(2);
        }
        // North
        int northNeighborY = currentNode->yCoord + offset;
        int northPathY = currentNode->yCoord + offset - 1;
        if (northNeighborY <= heightGrid && !gridNodes.Nodes[coordToIndex(currentNode->xCoord, northNeighborY, widthGrid)].visited){
            availableNeighboursDirections.push_back(3);
        }


        if (!(availableNeighboursDirections.empty())){
            int randomDirection = availableNeighboursDirections[rand() % availableNeighboursDirections.size()];
            Node* nextNode = nullptr;
            int pathIndexToClear = -1;

            switch (randomDirection) {
            case 0: // East
                nextNode = &(gridNodes.Nodes[coordToIndex(eastNeighborX, currentNode->yCoord, widthGrid)]);
                pathIndexToClear = coordToIndex(eastPathX, currentNode->yCoord, widthGrid);
                break;
            case 1: // South
                nextNode = &(gridNodes.Nodes[coordToIndex(currentNode->xCoord, southNeighborY, widthGrid)]);
                pathIndexToClear = coordToIndex(currentNode->xCoord, southPathY, widthGrid);
                break;
            case 2: // West
                nextNode = &(gridNodes.Nodes[coordToIndex(westNeighborX, currentNode->yCoord, widthGrid)]);
                pathIndexToClear = coordToIndex(westPathX, currentNode->yCoord, widthGrid);
                break;
            case 3: // North
                nextNode = &(gridNodes.Nodes[coordToIndex(currentNode->xCoord, northNeighborY, widthGrid)]);
                pathIndexToClear = coordToIndex(currentNode->xCoord, northPathY, widthGrid);
                break;
            default:
                break;
            }

            if (nextNode) {
                // Carve path (make it free)
                if (pathIndexToClear != -1) {
                    gridNodes.Nodes[pathIndexToClear].obstacle = false;
                    emit updatedScatterGridView(OBSTACLETOFREE, pathIndexToClear);
                }
                // Make next node free and push to stack
                nextNode->obstacle = false;
                nextNode->visited = true;
                emit updatedScatterGridView(OBSTACLETOFREE, coordToIndex(nextNode->xCoord, nextNode->yCoord, widthGrid));
                stackUnVisitedNodes.push(nextNode);
                numberVisitedCells++;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }else{
            // Backtrack
            stackUnVisitedNodes.pop();
        }
    }

    // Reset visited flags for next pathfinding run
    for(Node& node: gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false;
    }

    emit algorithmCompleted();
    // For maze generation, nodesVisited and pathLength are typically 0 or not applicable
    emit pathfindingSearchCompleted(0,0);
    qDebug() << "Maze: Algorithm completed. Emitting algorithmCompleted() and pathfindingSearchCompleted()";
}
//maze generation using prims algorithm
void PathAlgorithm::performPrimsMazeAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Maze: Prim's Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        emit pathfindingSearchCompleted(0, 0);
        return;
    }

    // Initialize all nodes as obstacles (except start/end)
    for (int index = 0; index < widthGrid * heightGrid; index++) {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during init.";
            emit pathfindingSearchCompleted(0, 0);
            return;
        }

        if (index != gridNodes.startIndex && index != gridNodes.endIndex) {
            gridNodes.Nodes[index].obstacle = true;
            emit updatedScatterGridView(FREETOOBSTACLE, index);
        }
    }

    // Choose a random starting cell for maze generation
    int randomIndex = rand() % (widthGrid * heightGrid);
    Node* startNode = &(gridNodes.Nodes[randomIndex]);

    // Ensure the start node is not an obstacle
    if (startNode->obstacle) {
        startNode->obstacle = false;
        emit updatedScatterGridView(OBSTACLETOFREE, randomIndex);
    }

    // List of frontier cells (edges between maze and walls)
    std::vector<Node*> frontier;
    addFrontierCells(startNode, frontier);

    int numberVisitedCells = 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (!frontier.empty()) {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during loop.";
            emit pathfindingSearchCompleted(0, 0);
            return;
        }

        // Select a random frontier cell
        int randomFrontierIndex = rand() % frontier.size();
        Node* frontierNode = frontier[randomFrontierIndex];
        frontier.erase(frontier.begin() + randomFrontierIndex);

        // Get all neighboring maze cells (2 steps away)
        std::vector<Node*> mazeNeighbors = getMazeNeighbors(frontierNode);

        if (!mazeNeighbors.empty()) {
            // Connect the frontier cell to a random maze neighbor
            Node* mazeNeighbor = mazeNeighbors[rand() % mazeNeighbors.size()];
            connectNodes(frontierNode, mazeNeighbor);

            // Add new frontier cells
            addFrontierCells(frontierNode, frontier);
            numberVisitedCells++;

            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
    }

    // Reset visited flags for next pathfinding run
    for (Node& node : gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false;
    }

    emit algorithmCompleted();
    emit pathfindingSearchCompleted(0, 0);
    qDebug() << "Maze: Prim's Algorithm completed.";
}

// Helper function to add frontier cells (unvisited neighbors 2 steps away)
void PathAlgorithm::addFrontierCells(Node* node, std::vector<Node*>& frontier) {
    const int offsets[4][2] = {{2, 0}, {-2, 0}, {0, 2}, {0, -2}}; // E, W, N, S

    for (const auto& offset : offsets) {
        int newX = node->xCoord + offset[0];
        int newY = node->yCoord + offset[1];

        if (newX >= 1 && newX <= widthGrid && newY >= 1 && newY <= heightGrid) {
            int index = coordToIndex(newX, newY, widthGrid);
            Node* neighbor = &(gridNodes.Nodes[index]);

            if (neighbor->obstacle && !neighbor->visited) {
                neighbor->visited = true; // Mark as frontier
                frontier.push_back(neighbor);
            }
        }
    }
}

// Helper function to get maze neighbors (visited cells 2 steps away)
std::vector<Node*> PathAlgorithm::getMazeNeighbors(Node* node) {
    std::vector<Node*> mazeNeighbors;
    const int offsets[4][2] = {{2, 0}, {-2, 0}, {0, 2}, {0, -2}}; // E, W, N, S

    for (const auto& offset : offsets) {
        int newX = node->xCoord + offset[0];
        int newY = node->yCoord + offset[1];

        if (newX >= 1 && newX <= widthGrid && newY >= 1 && newY <= heightGrid) {
            int index = coordToIndex(newX, newY, widthGrid);
            Node* neighbor = &(gridNodes.Nodes[index]);

            if (!neighbor->obstacle) { // Part of the maze
                mazeNeighbors.push_back(neighbor);
            }
        }
    }

    return mazeNeighbors;
}

// Helper function to connect two nodes by clearing the path between them
void PathAlgorithm::connectNodes(Node* a, Node* b) {
    // Calculate midpoint
    int midX = (a->xCoord + b->xCoord) / 2;
    int midY = (a->yCoord + b->yCoord) / 2;

    // Clear both nodes and the path between them
    a->obstacle = false;
    b->obstacle = false;
    int midIndex = coordToIndex(midX, midY, widthGrid);
    gridNodes.Nodes[midIndex].obstacle = false;

    // Emit signals for visualization
    emit updatedScatterGridView(OBSTACLETOFREE, coordToIndex(a->xCoord, a->yCoord, widthGrid));
    emit updatedScatterGridView(OBSTACLETOFREE, midIndex);
    emit updatedScatterGridView(OBSTACLETOFREE, coordToIndex(b->xCoord, b->yCoord, widthGrid));
}

//maze generation using kruskal algorithm
void PathAlgorithm::performKruskalsMazeAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Maze (Kruskal's): Algorithm started in worker thread:" << QThread::currentThreadId();

    // Cancel check
    promise.suspendIfRequested();
    if (promise.isCanceled()) {
        emit pathfindingSearchCompleted(0, 0);
        return;
    }

    // Step 1: Initialize all nodes as obstacles except start/end
    for (int index = 0; index < widthGrid * heightGrid; ++index) {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            emit pathfindingSearchCompleted(0, 0);
            return;
        }

        if (index != gridNodes.startIndex && index != gridNodes.endIndex) {
            gridNodes.Nodes[index].obstacle = true;
            emit updatedScatterGridView(FREETOOBSTACLE, index);
        }
    }

    // Step 2: Initialize Disjoint Set Union (DSU)
    std::vector<int> parent(widthGrid * heightGrid);
    std::vector<int> rank(widthGrid * heightGrid, 0);

    for (int i = 0; i < widthGrid * heightGrid; ++i)
        parent[i] = i; // every cell initially its own parent

    auto findSet = [&](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]]; // Path compression
            x = parent[x];
        }
        return x;
    };

    auto unionSet = [&](int a, int b) {
        a = findSet(a);
        b = findSet(b);
        if (a != b) {
            if (rank[a] < rank[b])
                parent[a] = b;
            else if (rank[b] < rank[a])
                parent[b] = a;
            else {
                parent[b] = a;
                rank[a]++;
            }
            return true;
        }
        return false;
    };

    // Step 3: Build walls list between adjacent odd-indexed cells
    struct Wall {
        int wallIndex;
        int cell1Index;
        int cell2Index;
    };
    std::vector<Wall> walls;

    const std::vector<QPair<int, int>> directions = {{2, 0}, {0, 2}}; // right and down

    for (int y = 1; y < heightGrid - 1; y += 2) {
        for (int x = 1; x < widthGrid - 1; x += 2) {
            int cellIndex = coordToIndex(x, y, widthGrid);
            for (auto dir : directions) {
                int nx = x + dir.first;
                int ny = y + dir.second;
                if (nx < widthGrid && ny < heightGrid) {
                    int neighborIndex = coordToIndex(nx, ny, widthGrid);
                    int wallX = x + dir.first / 2;
                    int wallY = y + dir.second / 2;
                    int wallIndex = coordToIndex(wallX, wallY, widthGrid);
                    walls.push_back({wallIndex, cellIndex, neighborIndex});
                }
            }
        }
    }

    // Step 4: Shuffle walls
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(walls.begin(), walls.end(), g);

    // Step 5: Carve paths by removing walls
    for (const Wall& wall : walls) {
        promise.suspendIfRequested();
        if (promise.isCanceled()) {
            emit pathfindingSearchCompleted(0, 0);
            return;
        }

        int set1 = findSet(wall.cell1Index);
        int set2 = findSet(wall.cell2Index);

        if (set1 != set2) {
            unionSet(set1, set2);

            // Carve wall
            gridNodes.Nodes[wall.wallIndex].obstacle = false;
            emit updatedScatterGridView(OBSTACLETOFREE, wall.wallIndex);

            // Carve both cells
            gridNodes.Nodes[wall.cell1Index].obstacle = false;
            emit updatedScatterGridView(OBSTACLETOFREE, wall.cell1Index);

            gridNodes.Nodes[wall.cell2Index].obstacle = false;
            emit updatedScatterGridView(OBSTACLETOFREE, wall.cell2Index);

            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        }
    }

    // Step 6: Reset visited/nextUp flags
    for (Node& node : gridNodes.Nodes) {
        node.visited = false;
        node.nextUp = false;
    }

    emit algorithmCompleted();
    emit pathfindingSearchCompleted(0, 0);
    qDebug() << "Maze (Kruskal's): Algorithm completed.";
}

void PathAlgorithm::FillNeighboursNode(Node& node)
{

    // east: adding +1 to x:
    if (node.xCoord + 1 <= widthGrid)
    {
        int eastIndex = coordToIndex(node.xCoord + 1, node.yCoord, widthGrid);
        node.neighbours.push_back(&(gridNodes.Nodes[eastIndex]));
    }

    // South: adding -1 to y:
    if (node.yCoord - 1 >= 1)
    {
        int southIndex = coordToIndex(node.xCoord, node.yCoord -1, widthGrid);
        node.neighbours.push_back(&(gridNodes.Nodes[southIndex]));
    }

    // West: adding -1 to x:
    if (node.xCoord - 1 >= 1)
    {
        int westIndex = coordToIndex(node.xCoord - 1, node.yCoord, widthGrid);
        node.neighbours.push_back(&(gridNodes.Nodes[westIndex]));
    }

    // north: adding +1 to y:
    if (node.yCoord + 1 <= heightGrid)
    {
        int northIndex = coordToIndex(node.xCoord, node.yCoord + 1, widthGrid);
        node.neighbours.push_back(&(gridNodes.Nodes[northIndex]));
    }
}
