#ifndef PATHALGORITHM_H
#define PATHALGORITHM_H
#include <QObject>
#include <QDebug>

#include <QtConcurrent>
#include <QFuture>
#include "GridView.h"

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

    // Maze generation
    void performRecursiveBackTrackerAlgorithm(QPromise<int>& promise);

    // Retrieving the neighbors of a point in a grid
    std::vector<Node> retrieveNeighborsGrid(const grid& gridNodes, const Node& currentNode, int widthGrid, int heightGrid);
    void FillNeighboursNode(Node& node);

    void checkGridNode(grid gridNodes, int heightGrid, int widthGrid);

public: Q_SIGNALS:
    void updatedScatterGridView (UPDATETYPES VISIT,     int currentIndex);
    void updatedLineGridView    (QPointF currentPoint,  bool addingPoint,   bool clearPriorToUpdate=false);

    void algorithmCompleted();

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

};

#endif // PATHALGORITHM_H
