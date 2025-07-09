#include <iostream> // Consider removing if not directly used for std::cout/cin
#include <QChartView>
#include <QMessageBox>
#include <QLabel>         // Include QLabel
#include <QTime>
#include <QFile>          // For CSV export
#include <QTextStream>    // For CSV export
#include "mainWindow.h"
#include "ui_mainWindow.h"
#include "GridView.h"


MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow), gridView(30, 30, 19), pathAlgorithm()
{
    // Setup of the window
    ui->setupUi(this);


    // Customize chart background
    QLinearGradient backgroundGradient;
    backgroundGradient.setStart(QPointF(0, 0));
    backgroundGradient.setFinalStop(QPointF(0, 1));
    backgroundGradient.setColorAt(0.0, QRgb(0xd2d0d1));
    backgroundGradient.setColorAt(1.0, QRgb(0x4c4547));
    backgroundGradient.setCoordinateMode(QGradient::ObjectBoundingMode);

    QBrush brush(backgroundGradient);

    QPalette palette;
    palette.setBrush(QPalette::Window, brush);
    setPalette(palette);

    // Setup
    ui->dialWidth   ->setValue  (gridView.widthGrid);
    ui->dialWidth   ->setMinimum(5);
    ui->dialWidth   ->setMaximum(35);
    ui->lcdWidth    ->display   (gridView.widthGrid);

    ui->dialHeight  ->setValue  (gridView.heightGrid);
    ui->dialHeight  ->setMinimum(5);
    ui->dialHeight  ->setMaximum(35);
    ui->lcdHeight   ->display   (gridView.heightGrid);

    ui->sliderMarker->setValue  (gridView.markerSize);
    ui->lcdMarker   ->display   (gridView.markerSize);

    // Initial Simulation speed
    ui->speedSpinBox->setMaximum(100);
    int speed = ui->speedSpinBox->maximum() / 5;
    ui->speedSpinBox->setValue  (speed);

    // Initial state for the run button
    ui->runButton->setChecked(false); // Ensure it starts in the "play" state
    ui->runButton->setText(QString("Start PathFinding")); // Initial text


    // Setting up the chart view
    setupGridView("gridView");

    // Setting up the Interaction Combo Box
    setupInteractionComboBox();

    // Setting up the Algorithms Combo Box
    setupAlgorithmsComboBox();

    // Setup the comparison table
    setupComparisonTable();

    // A change in the grid view create a change in the chartview
    connect(&pathAlgorithm, &PathAlgorithm::updatedScatterGridView, &gridView, &GridView::handleUpdatedScatterGridView);
    connect(&pathAlgorithm, &PathAlgorithm::updatedLineGridView,    &gridView, &GridView::handleUpdatedLineGridView);

    // Connecting the end signal of path planning to the window
    connect(&pathAlgorithm, &PathAlgorithm::algorithmCompleted, this, &MainWindow::onAlgorithmCompleted);
    //Connecting signal for pathfinding search completion (for timer stop)
    connect(&pathAlgorithm, &PathAlgorithm::pathfindingSearchCompleted, this, &MainWindow::onPathfindingSearchCompleted);

    // Connect pathfinding search completed to feature extraction
    connect(&pathAlgorithm, &PathAlgorithm::pathfindingSearchCompleted, this, &MainWindow::extractAndExportMazeFeatures);


    // --- Timer Setup ---
    animationTimer = new QTimer(this); // Initialize the QTimer
    // Connecting the timer's timeout signal to our new slot for updating the display
    connect(animationTimer, &QTimer::timeout, this, &MainWindow::updateElapsedTime);
    animationTimer->setInterval(10); // Set update frequency to every 10 milliseconds (0.01 seconds)

    // Initialize the QLabel for time display
    timeDisplayLabel = new QLabel("Time: 0.000 s", this);
    timeDisplayLabel->setAlignment(Qt::AlignCenter);
    timeDisplayLabel->setStyleSheet("font-weight: bold; color: #34ace0;"); // Styling for visibility
    ui->hLayout->addWidget(timeDisplayLabel); // Add the label to your existing horizontal layout
    pausedTimeOffset = 0; //initialize paused time offset

    // Connect the clear comparison button
    connect(ui->clearComparisonButton, &QPushButton::clicked, this, &MainWindow::on_clearComparisonButton_clicked);

    QPushButton* deleteRowButton = new QPushButton("Delete Selected Row", this);
    ui->verticalLayout_2->addWidget(deleteRowButton); // in the comparisonTab's vertical layout
    connect(deleteRowButton, &QPushButton::clicked, this, &MainWindow::on_deleteSelectedRowButton_clicked);
}


MainWindow::~MainWindow() //destructor
{
    delete ui;
    delete animationTimer; // Clean up timer
    // timeDisplayLabel is a child of centralWidget, so it will be deleted automatically
}

void MainWindow::onPathfindingSearchCompleted(int nodesVisited, int pathLength)
{
    // Stop the animation timer
    animationTimer->stop();

    // Calculate the total elapsed time
    qint64 totalElapsedTime = pausedTimeOffset + elapsedTimer.elapsed();

    // Create a new AlgorithmComparisonData entry
    AlgorithmComparisonData data;
    data.algorithmName = ui->algorithmsBox->currentText(); // Get the name of the selected algorithm
    data.timeElapsedMs = totalElapsedTime;
    data.nodesVisited = nodesVisited;
    data.pathLength = pathLength;
    data.gridSize = QString("%1x%2").arg(gridView.widthGrid).arg(gridView.heightGrid); // Initialize with current grid size
    // For wallDensity, numDeadEnds, and branchingFactor, these will be updated in extractAndExportMazeFeatures
    data.wallDensity = 0.0; // Placeholder
    data.numDeadEnds = 0;   // Placeholder
    data.branchingFactor = 0.0; // Placeholder


    comparisonDataList.append(data);

    // Update the comparison table immediately with the initial data
    updateComparisonTable();

    // The extractAndExportMazeFeatures slot is already connected to pathfindingSearchCompleted
    // and will further populate/update the entry with maze specific features and prediction.
}

// Setup the comparison table
void MainWindow::setupComparisonTable()
{
    ui->comparisonTable->setColumnCount(8); // Algorithm, Time, Nodes Visited, Path Length, Grid Size
    QStringList headers;
    headers << "Algorithm" << "Time (s)" << "Nodes Visited" << "Path Length" << "Grid Size"<< "Wall Density" << "Dead Ends" << "Branching Factor";
    ui->comparisonTable->setHorizontalHeaderLabels(headers);
    ui->comparisonTable->horizontalHeader()->setStretchLastSection(true);
    ui->comparisonTable->setSelectionBehavior(QAbstractItemView::SelectRows); // Select entire rows
    ui->comparisonTable->setSelectionMode(QAbstractItemView::SingleSelection); // Allow single row selection
}

// NEW: Function to update the QTableWidget with all data from comparisonDataList
void MainWindow::updateComparisonTable()
{
    ui->comparisonTable->setRowCount(comparisonDataList.size()); // Set row count to match data list size
    for (int i = 0; i < comparisonDataList.size(); ++i) {
        const AlgorithmComparisonData& data = comparisonDataList.at(i);
        ui->comparisonTable->setItem(i, 0, new QTableWidgetItem(data.algorithmName));
        ui->comparisonTable->setItem(i, 1, new QTableWidgetItem(QString::number(data.timeElapsedMs / 1000.0, 'f', 3))); // Display in seconds
        ui->comparisonTable->setItem(i, 2, new QTableWidgetItem(QString::number(data.nodesVisited)));
        ui->comparisonTable->setItem(i, 3, new QTableWidgetItem(QString::number(data.pathLength)));
        ui->comparisonTable->setItem(i, 4, new QTableWidgetItem(data.gridSize));
        ui->comparisonTable->setItem(i, 5, new QTableWidgetItem(QString::number(data.wallDensity, 'f', 4)));
        ui->comparisonTable->setItem(i, 6, new QTableWidgetItem(QString::number(data.numDeadEnds)));
        ui->comparisonTable->setItem(i, 7, new QTableWidgetItem(QString::number(data.branchingFactor, 'f', 4)));
    }
}

void MainWindow::setupInteractionComboBox()
{
    // Default text
    ui->interactionBox->setPlaceholderText(QStringLiteral("--Select Interaction--"));
    ui->interactionBox->setCurrentIndex(-1);

    // Adding first interation: Add starting point
    ui->interactionBox->addItem("Add Start");

    // Adding second interaction: Add end point
    ui->interactionBox->addItem("Add Goal");

    // Adding first interation: Add starting point
    ui->interactionBox->addItem("Add Obstacles");
}

void MainWindow::setupAlgorithmsComboBox()
{
    ui->algorithmsBox->setPlaceholderText(QStringLiteral("--Select Algorithm--"));
    ui->algorithmsBox->setCurrentIndex(-1);

    //algos used
    ui->algorithmsBox->addItem("BFS Algorithm");
    ui->algorithmsBox->addItem("DFS Algorithm");
    ui->algorithmsBox->addItem("Dijkstra's Algorithm");
    ui->algorithmsBox->addItem("A* Algorithm");

}

void MainWindow::setupGridView(QString gridViewName)
{

    // Setting up chartview
    ui->gridView->setObjectName(gridViewName);
    ui->gridView->setMinimumWidth(qreal(700));
    ui->gridView->setMinimumHeight(qreal(700));
    // Setup nodes in GridView
    gridView.setupNodes(); // This creates and populates gridView.gridNodes.Nodes

    // passing the updated grid data and dimensions to PathAlgorithm
    pathAlgorithm.setGridNodes(gridView.gridNodes, gridView.widthGrid, gridView.heightGrid);
    // This is crucial for a clean visualization when grid changes
    gridView.chart->removeAllSeries();
    gridView.chart->addSeries(gridView.freeElements);
    gridView.chart->addSeries(gridView.obstacleElements);
    gridView.chart->addSeries(gridView.visitedElements);
    gridView.chart->addSeries(gridView.nextElements);
    gridView.chart->addSeries(gridView.pathElements);
    gridView.chart->addSeries(gridView.pathLine);
    gridView.chart->addSeries(gridView.startElement);
    gridView.chart->addSeries(gridView.endElement);
    // Create Chart in chartview
    QChart* chart = gridView.createChart();
    ui->gridView->setChart(chart);

}



GridView& MainWindow::getGridView()
{
    return gridView;
}

void MainWindow::on_runButton_clicked()
{
    qDebug() << "DEBUG: on_runButton_clicked() entered.";
    if (ui->algorithmsBox->currentIndex() == -1){
        QMessageBox::information(this, "Information", "Please select a path finding algorithm");
        // Reset button state if no algorithm is selected
        ui->runButton->setChecked(false);
        ui->runButton->setText(QString("Start PathFinding")); // Consistent initial text
    } else if (pathAlgorithm.simulationOnGoing){ // If simulation has been started before (running or paused)
        if (pathAlgorithm.running){
            qDebug() << "DEBUG: Run button: Simulation is ongoing, handling pause/resume.";
            // Algorithm is currently running, so pause it
            pathAlgorithm.pauseAlgorithm();
            gridView.setSimulationRunning(false);
            pathAlgorithm.running = false; // Explicitly set running to false
            ui->runButton->setChecked(false);
            ui->runButton->setText(QString("Resume PathFinding")); // Indicate it's paused
            animationTimer->stop(); //stop the display timer
            pausedTimeOffset += elapsedTimer.elapsed(); //accumulate the elapsed time
        } else {
            // Algorithm is paused, so resume it
            pathAlgorithm.resumeAlgorithm();
            gridView.setSimulationRunning(true);
            pathAlgorithm.running = true; // Explicitly set running to true
            ui->runButton->setChecked(true);
            ui->runButton->setText(QString("Pause PathFinding")); // Indicate it's running
            elapsedTimer.restart(); //Restart elapsed timer for new segment
            animationTimer->start();
        }
    } else {
        // This is the initial start of the pathfinding algorithm
        pathAlgorithm.running = true;
        pathAlgorithm.simulationOnGoing = true; //Set this to true on initial start

        // set the grid node of the path algorithm object;
        pathAlgorithm.gridNodes = gridView.gridNodes;
        pathAlgorithm.heightGrid = gridView.heightGrid;
        pathAlgorithm.widthGrid = gridView.widthGrid;

        // Setting the run button as checkable and checked (setCheckable should ideally be in UI XML or constructor)
        ui->runButton->setCheckable(true);
        ui->runButton->setChecked(true); // Set to checked for "pause" icon/state
        ui->runButton->setText(QString("Pause PathFinding")); // Indicate it's running

        // Blocking the interaction with the gridView
        gridView.setSimulationRunning(true);

        // Enabling the current QScatter series point as visible
        gridView.AlgorithmView(true);
        pausedTimeOffset = 0; // Reset offset for a new run
        // Start the elapsed timer

        elapsedTimer.start();
        animationTimer->start(); // Start the animation timer


        // Call path finding
        pathAlgorithm.runAlgorithm(pathAlgorithm.getCurrentAlgorithm());
    }
}

void MainWindow::on_mazeButton_clicked()
{
    gridView.setCurrentAlgorithm(BACKTRACK);
    pathAlgorithm.running = true;
    pathAlgorithm.simulationOnGoing = true; // Set to true for maze generation too

    // set the grid node of the path algorithm object;
    pathAlgorithm.gridNodes = gridView.gridNodes;
    pathAlgorithm.heightGrid = gridView.heightGrid;
    pathAlgorithm.widthGrid = gridView.widthGrid;

    // Blocking the interaction with the gridView
    gridView.setSimulationRunning(true);

    // Enabling the current QScatter series point as visible
    gridView.AlgorithmView(true);

    // Reset elapsed timer and paused offset for maze generation
    pausedTimeOffset = 0; // <--- This is the crucial line added/ensured
    elapsedTimer.start();
    animationTimer->start();

    // Call path finding
    pathAlgorithm.runAlgorithm(gridView.getCurrentAlgorithm());
}

void MainWindow::on_resetButton_clicked()
{
    // Calling populate grid with same previous arrangement
    gridView.populateGridMap(gridView.getCurrentArrangement(), true);

    // Reset pathfinding flags and button state on reset
    pathAlgorithm.running = false;
    pathAlgorithm.simulationOnGoing = false;
    ui->runButton->setChecked(false);
    ui->runButton->setText(QString("Start PathFinding"));
    gridView.setSimulationRunning(false); // Ensure grid interaction is re-enabled
    // Reset interaction to NOINTERACTION
    gridView.setCurrentInteraction(NOINTERACTION);
    ui->interactionBox->setCurrentIndex(-1);
    // Reset algorithm selection to NOALGO internally
    pathAlgorithm.setCurrentAlgorithm(NOALGO);


    ui->algorithmsBox->setCurrentIndex(-1); // Deselects any algorithm in the combobox

    // Reset button text for run button
    ui->runButton->setText("Start PathFinding");




    // Stop timers and reset display
    animationTimer->stop(); // <-- This line stops the timer
    timeDisplayLabel->setText("Time: 0.000 s"); // <-- This line resets the displayed text
    pausedTimeOffset = 0;
}

void MainWindow::on_interactionBox_currentIndexChanged(int index)
{
    // Updating the current interaction chosen by the user
    gridView.setCurrentInteraction(index);
    // Stop timers and reset display
    animationTimer->stop();
    timeDisplayLabel->setText("Time: 0.000 s");

}

void MainWindow::on_algorithmsBox_currentIndexChanged(int index)
{
    // Changing the current Algorithm
    gridView.setCurrentAlgorithm(index);
    pathAlgorithm.setCurrentAlgorithm(static_cast<ALGOS>(index));
}


void MainWindow::onAlgorithmCompleted()
{
    gridView.setSimulationRunning(false);
    pathAlgorithm.setSimulationOnGoing(false);
    pathAlgorithm.running = false; // Ensure running flag is false on completion
    ui->runButton->setChecked(false);
    ui->runButton->setText(QString("Start PathFinding")); // Consistent initial text

    // gridView.setCurrentAlgorithm(ui->algorithmsBox->currentIndex());


}
// Slot to handle when the pathfinding search itself completes
void MainWindow::extractAndExportMazeFeatures(int nodesVisited, int pathLength) {
    float currentWallDensity = gridView.calculateWallDensity();
    int currentNumDeadEnds = gridView.countDeadEnds();
    float currentBranchingFactor = gridView.calculateBranchingFactor();
    QString currentGridSize = QString("%1x%2").arg(gridView.widthGrid).arg(gridView.heightGrid);

    qDebug() << "Extracted Maze Features:";
    qDebug() << "  Grid Size:" << currentGridSize;
    qDebug() << "  Wall Density:" << QString::number(currentWallDensity, 'f', 4);
    qDebug() << "  Shortest Path Length (from signal):" << pathLength;
    qDebug() << "  Nodes Visited (from signal):" << nodesVisited;
    qDebug() << "  Number of Dead Ends:" << currentNumDeadEnds;
    qDebug() << "  Branching Factor:" << QString::number(currentBranchingFactor, 'f', 4);

    // Always update the last entry, assuming onPathfindingSearchCompleted has already added one.
    if (comparisonDataList.isEmpty()) {
        qWarning() << "No algorithm data in list to update maze features to. This should not happen.";
        return;
    }
    AlgorithmComparisonData& currentData = comparisonDataList.last();

    // Update the maze-specific features and grid size
    currentData.wallDensity = currentWallDensity;
    currentData.numDeadEnds = currentNumDeadEnds;
    currentData.branchingFactor = currentBranchingFactor;
    currentData.gridSize = currentGridSize; // Ensure gridSize is updated here as well

    // If it was a maze generation, ensure the algorithm name is correctly set
    if (gridView.getCurrentAlgorithm() == BACKTRACK) {
        currentData.algorithmName = "Maze Generation";
        // For maze generation, nodesVisited and pathLength from the signal are 0.
        // If these were populated by onPathfindingSearchCompleted, keep them as 0.
        currentData.nodesVisited = nodesVisited;
        currentData.pathLength = pathLength;
    }

    // After adding/updating data in comparisonDataList, refresh the table
    updateComparisonTable();

    // --- CSV Export Logic ---
    if (comparisonDataList.isEmpty()) {
        qWarning() << "Cannot export to CSV: comparisonDataList is empty.";
        return;
    }
    exportFeaturesToCSV(comparisonDataList.last()); // Pass the last (most recent) data entry
}


// Update the exportFeaturesToCSV function signature and implementation
void MainWindow::exportFeaturesToCSV(const AlgorithmComparisonData& dataToExport) {
    QFile file("maze_data.csv");
    QTextStream stream(&file);

    // Check if file exists to write header
    if (!file.exists()) {
        qDebug() << "maze_data.csv does not exist. Attempting to create and write header.";
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            // Updated header to include all new fields
            stream << "Algorithm,TimeElapsedMs,NodesVisited,PathLength,GridSize,WallDensity,NumDeadEnds,BranchingFactor\n";
            qDebug() << "Header written to maze_data.csv successfully.";
        } else {
            qWarning() << "Could not open maze_data.csv for writing header. Error:" << file.errorString();
            return;
        }
    }

    // Append data
    qDebug() << "Attempting to append data to maze_data.csv.";
    if (file.open(QIODevice::Append | QIODevice::Text)) {
        stream << dataToExport.algorithmName << ","
               << dataToExport.timeElapsedMs << ","
               << dataToExport.nodesVisited << ","
               << dataToExport.pathLength << ","
               << dataToExport.gridSize << ","
               << QString::number(dataToExport.wallDensity, 'f', 4) << ","
               << dataToExport.numDeadEnds << ","
               << QString::number(dataToExport.branchingFactor, 'f', 4);
         file.close();
        qDebug() << "Maze features appended to maze_data.csv successfully.";
    } else {
        qWarning() << "Could not open maze_data.csv for appending data. Error:" << file.errorString();
    }
}


void MainWindow::on_dialWidth_valueChanged(int value)
{
    ui->lcdWidth->display(value);

}


void MainWindow::on_dialHeight_valueChanged(int value)
{
    ui->lcdHeight->display(value);

}


void MainWindow::on_sliderMarker_valueChanged(int value)
{
    ui->lcdMarker->display(value);
}


void MainWindow::on_sliderMarker_sliderReleased()
{
    // Set the new marker size
    gridView.markerSize = ui->lcdMarker->value();

    // Set the marker size of elements
    gridView.setElementsMarkerSize();
}


void MainWindow::on_dialWidth_sliderReleased()
{
    // Set the new width of the grid
    gridView.widthGrid = ui->lcdWidth->value();

    // Resetting the gridview
    gridView.populateGridMap(gridView.getCurrentArrangement(), true);
}


void MainWindow::on_dialHeight_sliderReleased()
{
    // Set the new height of the grid
    gridView.heightGrid = ui->lcdHeight->value();

    // Resetting the gridview
    gridView.populateGridMap(gridView.getCurrentArrangement(), true);

}




void MainWindow::on_speedSpinBox_valueChanged(int arg1)
{
    pathAlgorithm.setSpeedVizualization(ui->speedSpinBox->maximum() / arg1);
}
void MainWindow::updateElapsedTime()
{
    qint64 elapsed = pausedTimeOffset + elapsedTimer.elapsed(); // Get elapsed time in milliseconds
    timeDisplayLabel->setText(QString("Time: %1 s").arg(elapsed / 1000.0, 0, 'f', 3)); // Display in seconds with 3 decimal places
}

//  Slot to clear all comparison data
void MainWindow::on_clearComparisonButton_clicked()
{
    comparisonDataList.clear(); // Clear the internal list
    ui->comparisonTable->setRowCount(0); // Clear the table widget rows
    QMessageBox::information(this, "Comparison Table", "All comparison data cleared.");
}

// NEW: Slot to delete a selected row
void MainWindow::on_deleteSelectedRowButton_clicked()
{
    QModelIndexList selectedRows = ui->comparisonTable->selectionModel()->selectedRows();
    if (selectedRows.isEmpty()) {
        QMessageBox::information(this, "Delete Row", "Please select a row to delete.");
        return;
    }

    // Iterate in reverse order to avoid issues with changing row indices
    for (int i = selectedRows.count() - 1; i >= 0; --i) {
        int rowToDelete = selectedRows.at(i).row();
        ui->comparisonTable->removeRow(rowToDelete);
        comparisonDataList.removeAt(rowToDelete); // Also remove from internal data list
    }
    QMessageBox::information(this, "Delete Row", "Selected row(s) deleted.");
}
