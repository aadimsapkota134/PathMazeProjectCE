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
void PathAlgorithm::performBFSAlgorithm(QPromise<int>& promise)
{
    qDebug() << "BFS: Algorithm started in worker thread:" << QThread::currentThreadId();

    // Allow to pause and stop the simulation (to debug)
    promise.suspendIfRequested();
    if (promise.isCanceled())
        return;

    // Reach the goal
    bool reachEnd = false;

    // Initialize the queue of Nodes to visit in the next step
    std::queue<Node> nextNodes;

    // Initializing a vector of nodes (through copy of the original): to keep track of parents: index: index parent, value: Node child
    std::vector<Node> parentNodes = gridNodes.Nodes;

    // Initializing a vector of nodes (through copy of the original): to recreate the path: index: index parent, value: Node child
    std::vector<Node> pathNodes = gridNodes.Nodes;

    // Starting point
    nextNodes.push(gridNodes.Nodes[gridNodes.startIndex]);

    // Keeping track of the number of nodes left to check in the next layer and in the current layer
    int nodesInNextLayer = 0;
    int nodesLeftInCurrentLayer =1 ;

    // Counting the optimal number of moves needed to go from start to finish
    int moveCount {};

    // Initialization current node
    Node currentNode;

    //
    bool addingPoint = true;

    while(!nextNodes.empty())
    {
        qDebug() << "BFS: Before suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested(); // Check for pause/resume within the loop
        qDebug() << "BFS: After suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "BFS: Algorithm cancelled during loop.";
            return;
        }

        // Current Node
        currentNode =  nextNodes.front(); nextNodes.pop();
        int currentIndex = coordToIndex(currentNode.xCoord, currentNode.yCoord, widthGrid);

        // updating Line gridView
        emit updatedLineGridView(QPointF(currentNode.xCoord, currentNode.yCoord), addingPoint);
        addingPoint = false;

        if (currentIndex == gridNodes.endIndex)
        {
            std::cerr << "Reached end \n";
            reachEnd = true;
            break; // Path found, exit search loop
        }

        if (currentNode.visited == false && currentNode.obstacle == false)
        {

            // This node is now visited
            currentNode.visited = true;
            gridNodes.Nodes[currentIndex].visited = true;

            // Update this node as visited in the gridView
            emit updatedScatterGridView(VISIT, currentIndex);

            // Retrieve neighbors and pushing it to the next nodes to check
            std::vector<Node> neighbors = retrieveNeighborsGrid(gridNodes, currentNode, widthGrid, heightGrid);

            for (auto nextNode=neighbors.begin(); nextNode < neighbors.end(); nextNode++)
            {
                // Pushing the neighbors in the next nodes to be checked if the node has not been added in the nextNode before
                if ((*nextNode).nextUp == false){

                    int nextIndex = coordToIndex((*nextNode).xCoord, (*nextNode).yCoord, widthGrid);

                    gridNodes.Nodes[nextIndex].nextUp = true;
                    nextNodes.push(*nextNode);

                    emit updatedScatterGridView(NEXT, nextIndex);

                    // Keeping track of the number of nodes in the next layers left to check
                    nodesInNextLayer++;

                    //Keeping track of parent node
                    parentNodes[nextIndex] = currentNode;
                    parentNodes[nextIndex].xCoord = currentNode.xCoord;
                    parentNodes[nextIndex].yCoord = currentNode.yCoord;
                }
            }

            // This node has been visited
            nodesLeftInCurrentLayer--;

            // Time and checking for stop from running button
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization)); // Use speedVisualization
            qDebug() << "BFS: After sleep. Time:" << QDateTime::currentMSecsSinceEpoch();
        }

        // if all nodes in the current layer have been checked
        if (nodesLeftInCurrentLayer == 0)
        {
            nodesLeftInCurrentLayer = nodesInNextLayer;
            nodesInNextLayer = 0;
            moveCount++;
            addingPoint = true;
        }
    }

    // If the end is reached, we output the path
    if (reachEnd){
        endReached = true;
        emit pathfindingSearchCompleted(); // Emit signal when search is complete
        qDebug() << "BFS: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        Node goal = currentNode;
        Node reverse = goal;

        // Line Path
        emit updatedLineGridView(QPointF(reverse.xCoord, reverse.yCoord), true, true);

        while(reverse.xCoord != gridNodes.Nodes[gridNodes.startIndex].xCoord || reverse.yCoord != gridNodes.Nodes[gridNodes.startIndex].yCoord){
            qDebug() << "BFS: Before suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            promise.suspendIfRequested(); // Check for pause/resume during visualization
            qDebug() << "BFS: After suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            if (promise.isCanceled()) {
                qDebug() << "BFS: Algorithm cancelled during visualization.";
                break; // Allow cancellation during visualization
            }

            emit updatedLineGridView(QPointF(reverse.xCoord, reverse.yCoord), true, false);

            int reverseIndex = coordToIndex(reverse.xCoord, reverse.yCoord, widthGrid);
            Node parentNode = parentNodes[reverseIndex];

            reverse = parentNode;

            // Updating the GridView with PATH
            emit updatedScatterGridView(PATH, reverseIndex);
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization)); // Use speedVisualization
            qDebug() << "BFS: After sleep during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
        }

        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);


    }else{
        endReached = false;
        emit pathfindingSearchCompleted(); // Emit signal even if path not found
        qDebug() << "BFS: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    emit algorithmCompleted(); // This signal is now primarily for post-visualization cleanup
    qDebug() << "BFS: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}

// DFS Algorithm
void PathAlgorithm::performDFSAlgorithm(QPromise<int>& promise)
{
    qDebug() << "DFS: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled())
        return;

    bool reachEnd = false;
    std::stack<Node> nextNodes;
    std::vector<Node> parentNodes = gridNodes.Nodes;
    std::vector<Node> pathNodes = gridNodes.Nodes;
    nextNodes.push(gridNodes.Nodes[gridNodes.startIndex]);
    int nodesInNextLayer = 0;
    int nodesLeftInCurrentLayer =1 ;
    int moveCount {};
    Node currentNode;
    bool addingPoint = true;
    while(!nextNodes.empty())
    {
        qDebug() << "DFS: Before suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested();
        qDebug() << "DFS: After suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "DFS: Algorithm cancelled during loop.";
            return;
        }

        currentNode =  nextNodes.top(); nextNodes.pop();
        int currentIndex = coordToIndex(currentNode.xCoord, currentNode.yCoord, widthGrid);

        emit updatedLineGridView(QPointF(currentNode.xCoord, currentNode.yCoord), addingPoint);
        addingPoint = false;

        if (currentIndex == gridNodes.endIndex)
        {
            std::cerr << "Reached end \n";
            reachEnd = true;
            break;
        }

        if (currentNode.visited == false && currentNode.obstacle == false)
        {
            currentNode.visited = true;
            gridNodes.Nodes[currentIndex].visited = true;
            emit updatedScatterGridView(VISIT, currentIndex);
            std::vector<Node> neighbors = retrieveNeighborsGrid(gridNodes, currentNode, widthGrid, heightGrid);

            for (auto nextNode=neighbors.begin(); nextNode < neighbors.end(); nextNode++)
            {
                if ((*nextNode).nextUp == false){
                    int nextIndex = coordToIndex((*nextNode).xCoord, (*nextNode).yCoord, widthGrid);
                    gridNodes.Nodes[nextIndex].nextUp = true;
                    nextNodes.push(*nextNode);
                    emit updatedScatterGridView(NEXT, nextIndex);
                    nodesInNextLayer++;
                    parentNodes[nextIndex] = currentNode;
                    parentNodes[nextIndex].xCoord = currentNode.xCoord;
                    parentNodes[nextIndex].yCoord = currentNode.yCoord;
                }
            }
            nodesLeftInCurrentLayer--;
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
            qDebug() << "DFS: After sleep. Time:" << QDateTime::currentMSecsSinceEpoch();
        }

        if (nodesLeftInCurrentLayer == 0){
            nodesLeftInCurrentLayer = nodesInNextLayer;
            nodesInNextLayer = 0;
            moveCount++;
            addingPoint = true;
        }
    }

    promise.addResult(moveCount);

    if (reachEnd){
        endReached = true;
        emit pathfindingSearchCompleted();
        qDebug() << "DFS: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        Node goal = currentNode;
        Node reverse = goal;
        emit updatedLineGridView(QPointF(reverse.xCoord, reverse.yCoord), true, true);

        int count=0;
        while(reverse.xCoord != gridNodes.Nodes[gridNodes.startIndex].xCoord || reverse.yCoord != gridNodes.Nodes[gridNodes.startIndex].yCoord)
        {
            qDebug() << "DFS: Before suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            promise.suspendIfRequested();
            qDebug() << "DFS: After suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            if (promise.isCanceled()) {
                qDebug() << "DFS: Algorithm cancelled during visualization.";
                break;
            }

            emit updatedLineGridView(QPointF(reverse.xCoord, reverse.yCoord), true, false);
            int reverseIndex = coordToIndex(reverse.xCoord, reverse.yCoord, widthGrid);
            Node parentNode = parentNodes[reverseIndex];
            emit updatedScatterGridView(PATH, reverseIndex);
            reverse = parentNode;
            count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
            qDebug() << "DFS: After sleep during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{
        endReached = false;
        emit pathfindingSearchCompleted();
        qDebug() << "DFS: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    emit algorithmCompleted();
    qDebug() << "DFS: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}


void PathAlgorithm::performDijkstraAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Dijkstra: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled())
        return;

    for(Node& node: gridNodes.Nodes)
    {
        FillNeighboursNode(node);
        node.localGoal      = INFINITY;
        node.parent         = nullptr;
    }

    auto distance = [](Node* a, Node* b)
    {
        return sqrtf(   (a->xCoord - b->xCoord) * (a->xCoord - b->xCoord)
                     +(a->yCoord - b->yCoord) * (a->yCoord - b->yCoord));
    };

    Node* nodeStart = &(gridNodes.Nodes[gridNodes.startIndex]);
    Node* nodeEnd = &(gridNodes.Nodes[gridNodes.endIndex]);

    Node* nodeCurrent = &(gridNodes.Nodes[gridNodes.startIndex]);
    nodeStart->localGoal = 0.0f;

    std::list<Node*> nodesToTest;
    nodesToTest.push_back(nodeCurrent);

    while(!nodesToTest.empty())
    {
        qDebug() << "Dijkstra: Before suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested();
        qDebug() << "Dijkstra: After suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "Dijkstra: Algorithm cancelled during loop.";
            return;
        }

        nodesToTest.sort([](const Node* a, const Node* b){return a->localGoal < b->localGoal;});
        while(!nodesToTest.empty() && nodesToTest.front()->visited)     {   nodesToTest.pop_front();    }
        if (nodesToTest.empty())    {   break;  }

        nodeCurrent = nodesToTest.front();
        nodeCurrent->visited = true;

        int indexCurrent = coordToIndex(nodeCurrent->xCoord, nodeCurrent->yCoord, widthGrid);
        emit updatedScatterGridView(VISIT, indexCurrent);

        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        qDebug() << "Dijkstra: After sleep. Time:" << QDateTime::currentMSecsSinceEpoch();

        for (Node* nodeNeighbour: nodeCurrent->neighbours)
        {
            if(!nodeNeighbour->visited && !nodeNeighbour->obstacle)
            {
                int nextUpIndex = coordToIndex(nodeNeighbour->xCoord, nodeNeighbour->yCoord, widthGrid);
                if (nextUpIndex == gridNodes.endIndex){
                    nodesToTest.clear();
                    nodeNeighbour->parent = nodeCurrent;
                    break;
                }else{
                    nodesToTest.push_back(nodeNeighbour);
                    emit updatedScatterGridView(NEXT, nextUpIndex);
                }
            }

            float potentialLowerGoal = nodeCurrent->localGoal + distance(nodeCurrent, nodeNeighbour);
            if (potentialLowerGoal < nodeNeighbour->localGoal){
                nodeNeighbour->parent = nodeCurrent;
                nodeNeighbour->localGoal = potentialLowerGoal;
            }
        }
    }

    if (nodeEnd->parent != nullptr){
        emit pathfindingSearchCompleted();
        qDebug() << "Dijkstra: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        Node* reverseNode = nodeEnd;
        emit updatedLineGridView(QPointF(reverseNode->xCoord, reverseNode->yCoord), true, true);

        while(reverseNode->parent != nullptr)
        {
            qDebug() << "Dijkstra: Before suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            promise.suspendIfRequested();
            qDebug() << "Dijkstra: After suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            if (promise.isCanceled()) {
                qDebug() << "Dijkstra: Algorithm cancelled during visualization.";
                break;
            }

            reverseNode = reverseNode->parent;
            int reverseIndex = coordToIndex(reverseNode->xCoord, reverseNode->yCoord, widthGrid);

            emit updatedScatterGridView(PATH, reverseIndex);
            emit updatedLineGridView(QPointF(reverseNode->xCoord, reverseNode->yCoord), true, false);

            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
            qDebug() << "Dijkstra: After sleep during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{
        endReached = -1;
        emit pathfindingSearchCompleted();
        qDebug() << "Dijkstra: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    emit algorithmCompleted();
    qDebug() << "Dijkstra: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}

void PathAlgorithm::performAStarAlgorithm(QPromise<int>& promise)
{
    qDebug() << "AStar: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled())
        return;

    for(Node& node: gridNodes.Nodes)
    {
        FillNeighboursNode(node);
        node.globalGoal     = INFINITY;
        node.localGoal      = INFINITY;
        node.parent         = nullptr;
    }

    auto distance = [](Node* a, Node* b)
    {
        return sqrtf(   (a->xCoord - b->xCoord) * (a->xCoord - b->xCoord)
                     +(a->yCoord - b->yCoord) * (a->yCoord - b->yCoord));
    };

    auto heuristic = [distance](Node* a, Node* b){return distance(a, b);};

    Node* nodeStart = &(gridNodes.Nodes[gridNodes.startIndex]);
    Node* nodeEnd = &(gridNodes.Nodes[gridNodes.endIndex]);

    Node* nodeCurrent = &(gridNodes.Nodes[gridNodes.startIndex]);
    nodeStart->localGoal = 0.0f;
    nodeStart->globalGoal = heuristic(nodeStart, nodeEnd);

    std::list<Node*> nodesToTest;
    nodesToTest.push_back(nodeCurrent);

    while(!nodesToTest.empty())
    {
        qDebug() << "AStar: Before suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested();
        qDebug() << "AStar: After suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "AStar: Algorithm cancelled during loop.";
            return;
        }

        nodesToTest.sort([](const Node* a, const Node* b){return a->globalGoal < b->globalGoal;});
        while(!nodesToTest.empty() && nodesToTest.front()->visited)     {   nodesToTest.pop_front();    }
        if (nodesToTest.empty())    {   break;  }

        nodeCurrent = nodesToTest.front();
        nodeCurrent->visited = true;

        int indexCurrent = coordToIndex(nodeCurrent->xCoord, nodeCurrent->yCoord, widthGrid);
        emit updatedScatterGridView(VISIT, indexCurrent);

        std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
        qDebug() << "AStar: After sleep. Time:" << QDateTime::currentMSecsSinceEpoch();

        for (Node* nodeNeighbour: nodeCurrent->neighbours)
        {
            int nextUpIndex = coordToIndex(nodeNeighbour->xCoord, nodeNeighbour->yCoord, widthGrid);
            if(!nodeNeighbour->visited && !nodeNeighbour->obstacle)
            {
                if (nextUpIndex == gridNodes.endIndex){
                    nodesToTest.clear();
                    nodeNeighbour->parent = nodeCurrent;
                    break;
                }else{
                    nodesToTest.push_back(nodeNeighbour);
                    emit updatedScatterGridView(NEXT, nextUpIndex);
                }
            }

            float potentialLowerGoal = nodeCurrent->localGoal + distance(nodeCurrent, nodeNeighbour);
            if (potentialLowerGoal < nodeNeighbour->localGoal)
            {
                nodeNeighbour->parent = nodeCurrent;
                nodeNeighbour->localGoal = potentialLowerGoal;
                nodeNeighbour->globalGoal = nodeNeighbour->localGoal + heuristic(nodeNeighbour, nodeEnd);
            }
        }
    }

    if (nodeEnd->parent != nullptr){
        emit pathfindingSearchCompleted();
        qDebug() << "AStar: Pathfinding search completed. Emitting pathfindingSearchCompleted()";

        Node* reverseNode = nodeEnd;
        emit updatedLineGridView(QPointF(reverseNode->xCoord, reverseNode->yCoord), true, true);
        while(reverseNode->parent != nullptr)
        {
            qDebug() << "AStar: Before suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            promise.suspendIfRequested();
            qDebug() << "AStar: After suspendIfRequested() during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
            if (promise.isCanceled()) {
                qDebug() << "AStar: Algorithm cancelled during visualization.";
                break;
            }

            reverseNode = reverseNode->parent;
            int reverseIndex = coordToIndex(reverseNode->xCoord, reverseNode->yCoord, widthGrid);

            emit updatedScatterGridView(PATH, reverseIndex);
            emit updatedLineGridView(QPointF(reverseNode->xCoord, reverseNode->yCoord), true, false);

            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
            qDebug() << "AStar: After sleep during visualization. Time:" << QDateTime::currentMSecsSinceEpoch();
        }
        emit updatedLineGridView(QPointF(gridNodes.Nodes[gridNodes.startIndex].xCoord, gridNodes.Nodes[gridNodes.startIndex].yCoord), true, false);

    }else{
        endReached = -1;
        emit pathfindingSearchCompleted();
        qDebug() << "AStar: Pathfinding search not found. Emitting pathfindingSearchCompleted()";
    }

    emit algorithmCompleted();
    qDebug() << "AStar: Algorithm completed (visualization done). Emitting algorithmCompleted()";
}

// Maze generation
void PathAlgorithm::performRecursiveBackTrackerAlgorithm(QPromise<int>& promise)
{
    qDebug() << "Maze: Algorithm started in worker thread:" << QThread::currentThreadId();
    promise.suspendIfRequested();
    if (promise.isCanceled())
        return;

    for (int index = 0; index < widthGrid * heightGrid; index++){
        qDebug() << "Maze: Before suspendIfRequested() during init. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested();
        qDebug() << "Maze: After suspendIfRequested() during init. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during init.";
            return;
        }

        if (index != gridNodes.startIndex || index != gridNodes.endIndex){
            gridNodes.Nodes[index].obstacle = true;
            emit updatedScatterGridView(FREETOOBSTACLE, index);
        }
    }

    std::stack<Node*> stackUnVisitedNodes;
    int randomIndex = rand() % (widthGrid * heightGrid);
    stackUnVisitedNodes.push(&(gridNodes.Nodes[randomIndex]));
    gridNodes.Nodes[randomIndex].visited = true;
    int numberVisitedCells = 1;
    int offset = 2;
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Initial sleep for maze generation

    while(numberVisitedCells < widthGrid * heightGrid && !stackUnVisitedNodes.empty() ){
        qDebug() << "Maze: Before suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        promise.suspendIfRequested();
        qDebug() << "Maze: After suspendIfRequested(). Loop Iteration. Time:" << QDateTime::currentMSecsSinceEpoch();
        if (promise.isCanceled()) {
            qDebug() << "Maze: Algorithm cancelled during loop.";
            return;
        }

        Node* currentNode = stackUnVisitedNodes.top();
        int currentIndex = coordToIndex(currentNode->xCoord, currentNode->yCoord, widthGrid);
        std::vector<int> availableNeighbours;

        int eastIndex = coordToIndex(currentNode->xCoord + offset - 1, currentNode->yCoord, widthGrid);
        int eastIndexOffset = coordToIndex(currentNode->xCoord + offset, currentNode->yCoord, widthGrid);
        if (currentNode->xCoord + offset <= widthGrid && !gridNodes.Nodes[eastIndexOffset].visited){
            availableNeighbours.push_back(0);
        }

        int southIndex          =   coordToIndex(currentNode->xCoord,currentNode->yCoord - offset + 1, widthGrid);
        int southIndexOffset    =   coordToIndex(currentNode->xCoord,currentNode->yCoord - offset, widthGrid);
        if (currentNode->yCoord - offset >= 1 && !gridNodes.Nodes[southIndexOffset].visited){
            availableNeighbours.push_back(1);
        }
        int westIndex = coordToIndex(currentNode->xCoord - offset + 1, currentNode->yCoord, widthGrid);
        int westIndexOffset = coordToIndex(currentNode->xCoord - offset, currentNode->yCoord, widthGrid);

        if (currentNode->xCoord - offset >= 1 && !gridNodes.Nodes[westIndexOffset].visited){
            availableNeighbours.push_back(2);
        }

        int northIndex = coordToIndex(currentNode->xCoord, currentNode->yCoord + offset -1, widthGrid);
        int northIndexOffset = coordToIndex(currentNode->xCoord, currentNode->yCoord + offset, widthGrid);
        if (currentNode->yCoord + offset <= heightGrid && !gridNodes.Nodes[northIndexOffset].visited){
            availableNeighbours.push_back(3);
        }

        if (!(availableNeighbours.empty())){
            int randomNeighour = availableNeighbours[rand() % availableNeighbours.size()];

            switch (randomNeighour) {
            case 0:
                emit updatedScatterGridView(OBSTACLETOFREE, eastIndex);
                emit updatedScatterGridView(OBSTACLETOFREE, eastIndexOffset);
                stackUnVisitedNodes.push(&(gridNodes.Nodes[eastIndexOffset]));
                break;
            case 1:
                emit updatedScatterGridView(OBSTACLETOFREE, southIndex);
                emit updatedScatterGridView(OBSTACLETOFREE, southIndexOffset);
                stackUnVisitedNodes.push(&(gridNodes.Nodes[southIndexOffset]));
                break;
            case 2:
                emit updatedScatterGridView(OBSTACLETOFREE, westIndex);
                emit updatedScatterGridView(OBSTACLETOFREE, westIndexOffset);
                stackUnVisitedNodes.push(&(gridNodes.Nodes[westIndexOffset]));
                break;
            case 3:
                emit updatedScatterGridView(OBSTACLETOFREE, northIndex);
                emit updatedScatterGridView(OBSTACLETOFREE, northIndexOffset);
                stackUnVisitedNodes.push(&(gridNodes.Nodes[northIndexOffset]));
                break;
            default:
                break;
            }

            numberVisitedCells++;
            gridNodes.Nodes[currentIndex].visited = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(speedVisualization));
            qDebug() << "Maze: After sleep. Time:" << QDateTime::currentMSecsSinceEpoch();
        }else{
            gridNodes.Nodes[currentIndex].visited = true;
            if (!stackUnVisitedNodes.empty()){
                stackUnVisitedNodes.pop();
            }else{
                break;
            }
        }
    }

    for(Node& node: gridNodes.Nodes)    {   node.visited = false;   }

    emit algorithmCompleted();
    emit pathfindingSearchCompleted();
    qDebug() << "Maze: Algorithm completed. Emitting algorithmCompleted() and pathfindingSearchCompleted()";
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
