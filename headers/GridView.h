#ifndef GRIDVIEW_H
#define GRIDVIEW_H

#include <QtCharts/QChartGlobal>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QLineSeries>
#include <QGridLayout>
#include <QObject>
#include <vector>

QT_USE_NAMESPACE

// Possible interactions in the gridview chosen in the Interaction Box
typedef enum {START, END, OBST, NOINTERACTION} INTERACTIONS;

// Possible grid arrangement chosen in the Maze Box
typedef enum {EMPTY, MAZE, NOARRANG} ARRANGEMENTS;

// Possible Algorithm chosen in the Algorithm Box
typedef enum {BFS, DFS, DIJKSTRA, ASTAR,BACKTRACK,PRIMS,KRUSKAL,WILSONS, NOALGO} ALGOS;

// Possible update in the grid view from the Path Algorithm
typedef enum {CURRENT, FREE, VISIT, OBSTACLETOFREE, FREETOOBSTACLE, NEXT, PATH, LINE} UPDATETYPES;


// Node structure
struct Node
{
    // For all algorithms
    int xCoord{}, yCoord{};
    bool visited = false;
    bool obstacle = false;

    // used in BFS and DFS (true: in nextNodes)
    bool nextUp = false;

    // used in ASTAR
    float globalGoal;
    float localGoal;
    Node* parent;
    std::vector<Node*> neighbours;

};

// Grid structure
struct grid //: public QObject
{
    //Q_OBJECT

public:
    std::vector<Node> Nodes;
    int startIndex;
    int endIndex;
    int currentIndex;
};

// NEW: Struct to hold maze features for difficulty estimation
struct MazeFeatures {
    int gridWidth;
    int gridHeight;
    float wallDensity;
    int shortestPathLength;
    int nodesVisited;
    int numDeadEnds;
    float branchingFactor;

    // Default constructor
    MazeFeatures() : gridWidth(0), gridHeight(0), wallDensity(0.0f),
        shortestPathLength(0), nodesVisited(0),
        numDeadEnds(0), branchingFactor(0.0f) {}
};

// Converting point coordinates to index
int coordToIndex(const QPointF& point, int widthGrid);
int coordToIndex(int x, int y, int widthGrid);

class GridView: public QChartView
{
    Q_OBJECT

public:

    //Constructor
    explicit GridView(int widthGrid, int heightGrid, int markerSize, QChartView* parent=0);

    // Destructor
    virtual ~GridView();

    // Setter/Getter: currentInteraction
    void            setCurrentInteraction(int index);
    void            setCurrentInteraction(INTERACTIONS interaction);
    INTERACTIONS    getCurrentInteraction() const;

    // Setter/Getter: currentAlgorithm from Box to PathAlgorithm
    void    setCurrentAlgorithm(int index);
    ALGOS   getCurrentAlgorithm() const;

    // Setter/Getter: current state of simulation running
    void setSimulationRunning(bool state);
    bool getSimulationRunning() const;

    // Setter/Getter current Arrangement
    ARRANGEMENTS getCurrentArrangement() const;

    // Setter/Getter: height grid
    int getHeightGrid() const;

    // Setter/Gsetter: grid of Nodes for the path planning algorithms
    grid& getGrid();

    // Create the chart and the grid
    QChart* createChart();

    // Populating the grid with points, depending on the arrangement selected
    void populateGridMap(ARRANGEMENTS arrangement, bool reset);

    // Computing the distance between two points
    qreal computeDistanceBetweenPoints(const QPointF& pointA, const QPointF& pointB);

    // Setup view for algorithm
    void AlgorithmView(bool on);

    // Replacing points in gridView
    void replaceStartbyCurrent();
    void replaceFreebyCurrent       (int updateIndex);
    void replaceFreebyNext          (int updateIndex);
    void replaceFreebyVisited       (int updateIndex);
    void replaceFreebyObstacle      (int updateIndex);
    void replaceObstaclebyCurrent   (int updateIndex);
    void replaceObstaclebyFree      (int updateIndex);
    void replaceVisitedbyPath       (int updateIndex);
    void replaceNextbyFree          (int updateIndex);
    void replaceNextbyVisited       (int updateIndex);

    // Update Line
    void updateLine(QPointF updatePoint, bool addingPoint);

    // Modifying View
    void setElementsMarkerSize();

    //Function to get the path length from the pathLine series
    int getPathLength() const;
public:
    // Add this new function declaration:
    void setupNodes();
    // NEW: Feature extraction methods
    float calculateWallDensity() const;
    int countDeadEnds() const;
    float calculateBranchingFactor() const;

public Q_SLOTS:

    // Event handleClickedPoint
    void handleClickedPoint(const QPointF& point);

    // Handles the changes in the gridView by the path planning algorithm
    bool handleUpdatedScatterGridView(UPDATETYPES updateType, int updateIndex);
    bool handleUpdatedLineGridView(QPointF updatePoint, bool addingPoint, bool clearPriorToUpdate=false);

public:

    QChart* chart;

    QScatterSeries* freeElements;
    QScatterSeries* obstacleElements;
    QScatterSeries* visitedElements;
    QScatterSeries* nextElements;
    QScatterSeries* pathElements;
    QScatterSeries* startElement;
    QScatterSeries* endElement;

    QLineSeries* pathLine;

    int widthGrid;
    int heightGrid;
    qreal markerSize;

    INTERACTIONS currentInteraction;
    ARRANGEMENTS currentArrangement;
    ALGOS        currentAlgorithm;

    bool simulationRunning;

    grid gridNodes;

};

#endif // GRIDVIEW_H
