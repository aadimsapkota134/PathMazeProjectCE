#include <iostream>
#include <QChartView>
#include <QMessageBox>
#include <QLabel>         
#include <QTime>
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
    // NEW: Connect signal for pathfinding search completion (for timer stop)
    connect(&pathAlgorithm, &PathAlgorithm::pathfindingSearchCompleted, this, &MainWindow::onPathfindingSearchCompleted);
    // --- Timer Setup ---
    animationTimer = new QTimer(this); // Initialize the QTimer
    // Connect the timer's timeout signal to our new slot for updating the display
    connect(animationTimer, &QTimer::timeout, this, &MainWindow::updateElapsedTime);
    animationTimer->setInterval(10); // Set update frequency to every 10 milliseconds (0.01 seconds)

    // Initialize the QLabel for time display
    timeDisplayLabel = new QLabel("Time: 0.000 s", this);
    timeDisplayLabel->setAlignment(Qt::AlignCenter);
    timeDisplayLabel->setStyleSheet("font-weight: bold; color: #34ace0;"); // Styling for visibility
    ui->hLayout->addWidget(timeDisplayLabel); // Add the label to your existing horizontal layout
    pausedTimeOffset = 0; //initialize paused time offset

    // NEW: Connect the clear comparison button
    connect(ui->clearComparisonButton, &QPushButton::clicked, this, &MainWindow::on_clearComparisonButton_clicked);

    QPushButton* deleteRowButton = new QPushButton("Delete Selected Row", this);
    ui->verticalLayout_2->addWidget(deleteRowButton); // Add to the comparisonTab's vertical layout
    connect(deleteRowButton, &QPushButton::clicked, this, &MainWindow::on_deleteSelectedRowButton_clicked);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete animationTimer; // Clean up timer
    // timeDisplayLabel is a child of centralWidget, so it will be deleted automatically
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

    // Adding first interation: BFS
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

    // CRITICAL: Now pass the updated grid data and dimensions to PathAlgorithm
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

// Setup the comparison table
void MainWindow::setupComparisonTable()
{
    ui->comparisonTable->setColumnCount(5); // Algorithm, Time, Nodes Visited, Path Length, Grid Size
    QStringList headers;
    headers << "Algorithm" << "Time (s)" << "Nodes Visited" << "Path Length" << "Grid Size";
    ui->comparisonTable->setHorizontalHeaderLabels(headers);
    ui->comparisonTable->horizontalHeader()->setStretchLastSection(true);
    ui->comparisonTable->setSelectionBehavior(QAbstractItemView::SelectRows); // Select entire rows
    ui->comparisonTable->setSelectionMode(QAbstractItemView::SingleSelection); // Allow single row selection
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

    // Update the UI's algorithm selection combobox
    // Assuming NOALGO corresponds to no selection or a specific "None" item.
    // If you have a "None" or "No Algorithm" item in your algorithmsBox,
    // find its index and set it. Otherwise, setting to -1 usually deselects.
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
// NEW: Slot to handle when the pathfinding search itself completes
void MainWindow::onPathfindingSearchCompleted(int nodesVisited, int pathLength) // <--- NEW SLOT IMPLEMENTATION
{
    animationTimer->stop(); // Stop the timer display
    qint64 finalElapsedTime = pausedTimeOffset + elapsedTimer.elapsed(); // Calculate total time
    timeDisplayLabel->setText(QString("Time: %1 s").arg(finalElapsedTime / 1000.0, 0, 'f', 3));

    // Only add to comparison table if it was a pathfinding algorithm, not maze generation
    if (gridView.getCurrentAlgorithm() != BACKTRACK) {
        AlgorithmComparisonData data;
        data.algorithmName = ui->algorithmsBox->currentText();
        data.timeElapsedMs = finalElapsedTime;
        data.nodesVisited = nodesVisited;
        data.pathLength = pathLength;
        data.gridSize = QString("%1x%2").arg(gridView.widthGrid).arg(gridView.heightGrid);
        comparisonDataList.append(data);
        qDebug() << "Data added to comparison list.";

        // Add data to the QTableWidget
        int row = ui->comparisonTable->rowCount();
        ui->comparisonTable->insertRow(row);

        ui->comparisonTable->setItem(row, 0, new QTableWidgetItem(data.algorithmName));
        ui->comparisonTable->setItem(row, 1, new QTableWidgetItem(QString::number(data.timeElapsedMs / 1000.0, 'f', 3)));
        ui->comparisonTable->setItem(row, 2, new QTableWidgetItem(QString::number(data.nodesVisited)));
        ui->comparisonTable->setItem(row, 3, new QTableWidgetItem(QString::number(data.pathLength)));
        ui->comparisonTable->setItem(row, 4, new QTableWidgetItem(data.gridSize));

        qDebug() << "Data displayed in table.";
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


void MainWindow::on_mazeButton_released()
{
// The main logic for maze generation is in on_mazeButton_clicked().
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

// NEW: Slot to clear all comparison data
void MainWindow::on_clearComparisonButton_clicked()
{
    comparisonDataList.clear(); // Clear the internal list
    ui->comparisonTable->setRowCount(0); // Clear the table widget rows
    QMessageBox::information(this, "Comparison Table", "All comparison data cleared.");
}

 Slot to delete a selected row
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
