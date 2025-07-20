#include "PlayerMazeWindow.h"
#include <QGridLayout>
#include <QWidget>
#include <QDebug>
#include <QBrush>
#include <QColor>
#include <QPen>

PlayerMazeWindow::PlayerMazeWindow(const grid& mazeGrid, int width, int height,
                                   int startIndex, int endIndex, QWidget *parent)
    : QMainWindow(parent),
    gridView(new GridView(width, height, 19, nullptr)), // Marker size 19, no parent for QChartView initially
    currentMazeGrid(mazeGrid),
    mazeWidth(width),
    mazeHeight(height),
    playerStartIndex(startIndex),
    playerEndIndex(endIndex),
    playerElement(new QScatterSeries()),
    playerPathLine(new QLineSeries()),
    gameTimer(new QTimer(this)),
    timerDisplayLabel(new QLabel("Time: 60 .000 s", this)),
    timeLimitSeconds(60),
     gameStarted(false)
{
    // Set up the main window properties
    setWindowTitle("Play Yourself Maze");
    setFixedSize(800, 850); // Fixed size for the game window

    // Central widget and layout
    QWidget *centralWidget = new QWidget(this);
    QGridLayout *mainLayout = new QGridLayout(centralWidget);
    setCentralWidget(centralWidget);

    // --- GridView Setup ---
    // Update gridView's internal gridNodes with the maze data
    gridView->gridNodes = currentMazeGrid;
    gridView->widthGrid = mazeWidth;
    gridView->heightGrid = mazeHeight;
    gridView->markerSize = 19; // Ensure marker size is set for display

    // Clear existing series and re-add them to the chart to ensure a clean state
    gridView->chart->removeAllSeries();

    // Re-add all series from gridView to its chart (important for display)
    gridView->chart->addSeries(gridView->freeElements);
    gridView->chart->addSeries(gridView->obstacleElements);
    gridView->chart->addSeries(gridView->visitedElements); // Not used in player mode, but good to have
    gridView->chart->addSeries(gridView->nextElements);    // Not used
    gridView->chart->addSeries(gridView->pathElements);    // Not used
    gridView->chart->addSeries(gridView->startElement);
    gridView->chart->addSeries(gridView->endElement);
    gridView->chart->addSeries(playerElement); // Add player element
    gridView->chart->addSeries(playerPathLine); // Add player path line

    // Set up the chart (axes, background, etc.)
    QChart* chart = gridView->createChart();
    gridView->setChart(chart);

    // Ensure start and end elements are correctly positioned based on the passed indices
    QPointF startPointF(currentMazeGrid.Nodes[playerStartIndex].xCoord, currentMazeGrid.Nodes[playerStartIndex].yCoord);
    QPointF endPointF(currentMazeGrid.Nodes[playerEndIndex].xCoord, currentMazeGrid.Nodes[playerEndIndex].yCoord);
    gridView->startElement->replace(0, startPointF);
    gridView->endElement->replace(0, endPointF);

    // Initial player position
    currentPlayerX = currentMazeGrid.Nodes[playerStartIndex].xCoord;
    currentPlayerY = currentMazeGrid.Nodes[playerStartIndex].yCoord;

    // --- Player Element Setup ---
    playerElement->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
    playerElement->setMarkerSize(gridView->markerSize);
    playerElement->setColor(QColor("#00a8ff")); // Blue for player
    playerElement->setBorderColor(QColorConstants::Black);
    playerElement->setOpacity(0.95);
    playerElement->append(QPointF(currentPlayerX, currentPlayerY)); // Initial player position

    // --- Player Path Line Setup ---
    playerPathLine->setPen(QPen(QColor("#ee9b10"), 3)); // Orange line with width 3

    playerPathLine->append(QPointF(currentPlayerX, currentPlayerY)); // Start path at player's initial position
    playerPathPoints.push_back(QPointF(currentPlayerX, currentPlayerY)); // Add to vector for backtracking

    // --- Timer Setup ---
    timerDisplayLabel->setAlignment(Qt::AlignCenter);
    timerDisplayLabel->setStyleSheet("font-weight: bold; color: #ff3f34; font-size: 24px;"); // Red, large font
    gameTimer->setInterval(10); // Update timer every 100ms
    connect(gameTimer, &QTimer::timeout, this, &PlayerMazeWindow::updateGameTimer);

    // Add GridView and timer label to the layout
    mainLayout->addWidget(gridView, 0, 0, 1, 2); // GridView takes more space
    mainLayout->addWidget(timerDisplayLabel, 1, 0, 1, 2); // Timer label below grid

    // Set focus policy to receive key events
    setFocusPolicy(Qt::StrongFocus);
    setFocus(); // Set initial focus to this window

    // Hide default elements not needed for player mode
    gridView->freeElements->setVisible(true); // Keep free elements visible
    gridView->obstacleElements->setVisible(true); // Keep obstacles visible
    gridView->visitedElements->setVisible(false);
    gridView->nextElements->setVisible(false);
    gridView->pathElements->setVisible(false);
    gridView->pathLine->setVisible(false); // Hide the algorithm-generated path line
}

PlayerMazeWindow::~PlayerMazeWindow()
{
    delete gridView;
    // playerElement, playerPathLine, gameTimer, timerDisplayLabel are children of this window or centralWidget,
    // so they will be deleted automatically.
}

void PlayerMazeWindow::startGame()
{
    qDebug() << "PlayerMazeWindow: Game started.";
    resetGame(); // Reset game state before starting
}

void PlayerMazeWindow::resetGame()
{
    gameTimer->stop();
    elapsedGameTimer.restart(); // Reset elapsed timer
    gameStarted=false;

    // Reset player position to start
    currentPlayerX = currentMazeGrid.Nodes[playerStartIndex].xCoord;
    currentPlayerY = currentMazeGrid.Nodes[playerStartIndex].yCoord;

    // Clear player element and path line, then add starting point
    playerElement->clear();
    playerElement->append(QPointF(currentPlayerX, currentPlayerY));
    playerPathLine->clear();
    playerPathPoints.clear();
    playerPathPoints.push_back(QPointF(currentPlayerX, currentPlayerY));
    updatePlayerPathLine(); // Update the QLineSeries from the vector

    // Reset timer display
    timerDisplayLabel->setText(QString("Time: %1.000 s").arg(timeLimitSeconds));

    qDebug() << "PlayerMazeWindow: Game reset.";
}

void PlayerMazeWindow::closeGame()
{
    gameTimer->stop();
    emit gameFinished(false); // Emit signal indicating game closed without winning
    this->close(); // Close the window
    qDebug() << "PlayerMazeWindow: Game closed.";
}

void PlayerMazeWindow::keyPressEvent(QKeyEvent *event)
{
// Check if the game timer has started. If not, start it on the first valid movement key press.
    if (!gameStarted) {
        if (event->key() == Qt::Key_W || event->key() == Qt::Key_Up ||
            event->key() == Qt::Key_S || event->key() == Qt::Key_Down ||
            event->key() == Qt::Key_A || event->key() == Qt::Key_Left ||
            event->key() == Qt::Key_D || event->key() == Qt::Key_Right)
        {
            gameStarted = true;
            elapsedGameTimer.start();
            gameTimer->start();
            qDebug() << "PlayerMazeWindow: Game timer started on first key press.";
        } else {
            QMainWindow::keyPressEvent(event); // Pass other non-movement events to base class
            return; // Don't process movement if timer hasn't started and it's not a movement key
        }
    }
    if (!gameTimer->isActive()) {
        // Ignore key presses if the game isn't active (e.g., after win/loss)
        return;
    }

    int newX = currentPlayerX;
    int newY = currentPlayerY;

    // Determine new coordinates based on key pressed
    if (event->key() == Qt::Key_W || event->key() == Qt::Key_Up) {
        newY++;
    } else if (event->key() == Qt::Key_S || event->key() == Qt::Key_Down) {
        newY--;
    } else if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
        newX--;
    } else if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
        newX++;
    } else {
        QMainWindow::keyPressEvent(event); // Pass other events to base class
        return;
    }

    // Check if new position is within bounds
    if (newX >= 1 && newX <= mazeWidth && newY >= 1 && newY <= mazeHeight) {
        int newIndex = coordToIndex(newX, newY, mazeWidth);
        const Node& targetNode = currentMazeGrid.Nodes[newIndex];

        if (!targetNode.obstacle) {
            // Valid move: update player position
            movePlayer(newX, newY);
        } else {
            qDebug() << "PlayerMazeWindow: Cannot move into obstacle at (" << newX << "," << newY << ")";
        }
    } else {
        qDebug() << "PlayerMazeWindow: Cannot move out of bounds.";
    }
}

void PlayerMazeWindow::movePlayer(int newX, int newY)
{
    QPointF newPlayerPos(newX, newY);
    QPointF currentPlayerPos(currentPlayerX, currentPlayerY);

    // Check for backtracking: if the new position is the second to last point in the path
    if (playerPathPoints.size() >= 2 && newPlayerPos == playerPathPoints[playerPathPoints.size() - 2]) {
        // Backtracking: remove the last point from the path
        playerPathPoints.pop_back();
        qDebug() << "PlayerMazeWindow: Backtracked to (" << newX << "," << newY << ")";
    } else {
        // Forward movement: add new position to path
        playerPathPoints.push_back(newPlayerPos);
    }

    currentPlayerX = newX;
    currentPlayerY = newY;

    playerElement->replace(0, newPlayerPos); // Update player's visual position
    updatePlayerPathLine(); // Update the QLineSeries

    checkWinCondition(); // Check if the player reached the end
}

void PlayerMazeWindow::updatePlayerPathLine()
{
    playerPathLine->clear();
    for (const QPointF& p : playerPathPoints) {
        playerPathLine->append(p);
    }
}

void PlayerMazeWindow::updateGameTimer()
{
    qint64 elapsedMs = elapsedGameTimer.elapsed();
    int remainingSeconds = timeLimitSeconds - (elapsedMs / 1000);

    if (remainingSeconds <= 0) {
        gameTimer->stop();
        timerDisplayLabel->setText("Time: 0.000 s");
        showGameOverDialog(false); // Player lost
    } else {
        timerDisplayLabel->setText(QString("Time: %1.%2 s")
                                       .arg(remainingSeconds)
                                       .arg((1000 - (elapsedMs % 1000)) / 100, 3, 10, QChar('0')));
    }
}

void PlayerMazeWindow::checkWinCondition()
{
    int currentPlayerIndex = coordToIndex(currentPlayerX, currentPlayerY, mazeWidth);
    if (currentPlayerIndex == playerEndIndex) {
        gameTimer->stop();
        showGameOverDialog(true); // Player won
    }
}

void PlayerMazeWindow::showGameOverDialog(bool won)
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle(won ? "You Won!" : "Time's Up!");
    msgBox.setText(won ? "Congratulations! You reached the goal within 60 seconds." : "Sorry, you ran out of time.");

    QPushButton *playAgainButton = msgBox.addButton(tr("Play Again"), QMessageBox::AcceptRole);
    QPushButton *backToMainButton = msgBox.addButton(tr("Back to Main Window"), QMessageBox::RejectRole);

    msgBox.exec();

    if (msgBox.clickedButton() == playAgainButton) {
        startGame(); // Restart the game
    } else if (msgBox.clickedButton() == backToMainButton) {
        emit gameFinished(won); // Notify MainWindow
        this->close(); // Close this window
    }
}
