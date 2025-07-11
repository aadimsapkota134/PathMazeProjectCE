#ifndef PATHALGORITHM_H
#define PATHALGORITHM_H
#include <QObject>
#include <QDebug>

#include <QtConcurrent>
#include <QFuture>
#include "GridView.h"
#include <queue> // for std::priority_queue
#include <cmath> // For fabsf

// For Dijkstra: orders by localGoal (min-heap)
struct CompareNodesDijkstra {
    bool operator()(Node* a, Node* b) {
        // For a min-heap, return true if 'a' has lower priority than 'b' (i.e., a's goal is greater than b's)
        return a->localGoal > b->localGoal;
    }
};

// For AStar: orders by globalGoal (min-heap)
struct CompareNodesAStar {
    bool operator()(Node* a, Node* b) {
        // For a min-heap, return true if 'a' has lower priority than 'b' (i.e., a's goal is greater than b's)
        return a->globalGoal > b->globalGoal;
    }
};


class PathAlgorithm : public QObject
{

    Q_OBJECT
public:

    //Constructor
    explicit PathAlgorithm(QObject* parent = nullptr);

    //Destructor
    virtual ~PathAlgorithm();

    //Getters/Setters: current Algorithm from gridView
    ALGOS getCurrentAlgorithm() const;
    void setCurrentAlgorithm(ALGOS algorithm);

    //Getters/Setters: speed visualization
    void setSpeedVizualization(int speed);

    // Getters/Setters: Simulation on going
    void setSimulationOnGoing(bool onGoing);

    // Running pausing and canceling algorithms
    void runAlgorithm(ALGOS algorithm);
    void pauseAlgorithm();
    void resumeAlgorithm();
    void stopAlgorithm();

    // Path planning Algorithms
    void performBFSAlgorithm(QPromise<int>& promise);
    void performDFSAlgorithm(QPromise<int>& promise);
    void performDijkstraAlgorithm(QPromise<int>& promise);
    void performAStarAlgorithm(QPromise<int>& promise);

    // Maze generation using dfs
    void performRecursiveBackTrackerAlgorithm(QPromise<int>& promise);
    //Maze generation using Prims
    void performPrimsMazeAlgorithm(QPromise<int>& promise);
    //maze generation using kruskal algorithm
    void PathAlgorithm::performKruskalsMazeAlgorithm(QPromise<int>& promise)

    // Retrieving the neighbors of a point in a grid
    std::vector<Node> retrieveNeighborsGrid(const grid& gridNodes, const Node& currentNode, int widthGrid, int heightGrid);
    void FillNeighboursNode(Node& node);

    void checkGridNode(grid gridNodes, int heightGrid, int widthGrid);

public: Q_SIGNALS:
    void updatedScatterGridView (UPDATETYPES VISIT,     int currentIndex);
    void updatedLineGridView    (QPointF currentPoint,  bool addingPoint,   bool clearPriorToUpdate=false);

    void algorithmCompleted(); // Emitted after visualization (e.g., for maze)
    void pathfindingSearchCompleted(int nodesVisited, int pathLength); // <--- NEW SIGNAL: Emitted when path is found, before visualization

public:

    ALGOS currentAlgorithm;
    bool running;
    bool simulationOnGoing;
    bool endReached;
    int speedVisualization;

    // grid nodes manipulated by the path planning object
    grid gridNodes;
    int heightGrid;
    int widthGrid;

    // Multithreading
    QThreadPool pool;
    QFuture<int> futureOutput;
public:
   QString algorithmToString(ALGOS algo);
public:
    //  new method to update to grids:
    void setGridNodes(const grid& newGridNodes, int width, int height);
};

#endif // PATHALGORITHM_H
