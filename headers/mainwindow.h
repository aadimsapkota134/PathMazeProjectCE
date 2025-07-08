#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>         // Add this
#include <QElapsedTimer>// Add this
#include <QTableWidget>   // Add this for QTableWidget
#include <QTableWidgetItem>
#include "GridView.h"
#include "PathAlgorithm.h"
#include "qlabel.h"

QT_BEGIN_NAMESPACE

namespace Ui
{
class MainWindow;
}

QT_END_NAMESPACE
//structure to hold comparison data for each algorithm run
struct AlgorithmComparisonData {
    QString algorithmName;
    qint64  timeElapsedMs;
    int     nodesVisited;
    int     pathLength;
    QString gridSize;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    // Constructor
    MainWindow(QWidget *parent = nullptr);

    // Destructor
    virtual ~MainWindow();

    // Setting up objects
    void setupInteractionComboBox();
    void setupAlgorithmsComboBox();
    void setupGridView(QString gridViewName);
    void setupComparisonTable();

    // Getters
    GridView& getGridView();


public: Q_SIGNALS:
    //void launchedBFS();

public slots:

    // Run the simulation and pause it
    void on_runButton_clicked();

    // Reset the ChartView
    void on_resetButton_clicked();

    // Generate button
    void on_mazeButton_clicked();

    // Handles the different interactions changes
    void on_interactionBox_currentIndexChanged(int index);

    // Handles the different algorithm changes
    void on_algorithmsBox_currentIndexChanged(int index);

    // Action to do when the path algorithm is finished
    void onAlgorithmCompleted();

    // Action to do when the pathfinding search completes (before visualization)
    void onPathfindingSearchCompleted(int nodesVisited, int pathLength);

private slots:
    void on_dialWidth_valueChanged(int value);

    void on_dialHeight_valueChanged(int value);

    void on_sliderMarker_valueChanged(int value);

    void on_sliderMarker_sliderReleased();

    void on_dialWidth_sliderReleased();

    void on_dialHeight_sliderReleased();

    void on_mazeButton_released();

    void on_speedSpinBox_valueChanged(int arg1);

    void updateElapsedTime(); // New slot for timer updates

    void on_clearComparisonButton_clicked(); // NEW: Slot for clearing all comparison data
    void on_deleteSelectedRowButton_clicked(); // NEW: Slot for deleting a selected row
private:

    Ui::MainWindow* ui;
    GridView gridView;
    PathAlgorithm pathAlgorithm;
    QTimer* animationTimer;        // Declare QTimer
    QElapsedTimer elapsedTimer;    // Declare QElapsedTimer
    QLabel* timeDisplayLabel;   // Declare QLabel for time display

    qint64 pausedTimeOffset; // NEW: Stores time accumulated before pausing
    QList<AlgorithmComparisonData> comparisonDataList; // NEW: To store comparison results
};
#endif // MAINWINDOW_H
