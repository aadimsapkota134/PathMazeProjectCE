#ifndef PLAYERMAZEWINDOW_H
#define PLAYERMAZEWINDOW_H

#include "GridView.h"
#include <QMainWindow>
#include <QtCharts/QtCharts> // Include the main QtCharts header
#include <QTimer>
#include <QElapsedTimer>
#include <QLabel>
#include <QKeyEvent>
#include <QMessageBox>
#include <QPushButton> // For the buttons in the game over dialog



    class PlayerMazeWindow : public QMainWindow
{
    Q_OBJECT

public:
    // Constructor: Takes the initial grid data, dimensions, and start/end indices
    explicit PlayerMazeWindow(const grid& mazeGrid, int width, int height,
                              int startIndex, int endIndex, QWidget *parent = nullptr);
    ~PlayerMazeWindow();

    // Initializes and starts the game
    void startGame();
    // Resets the game state
    void resetGame();
    // Cleans up and closes the game window
    void closeGame();

signals:
    // Signal emitted when the game finishes (won or lost)
    void gameFinished(bool won);

protected:
    // Override keyPressEvent to handle player movement
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    // Slot to update the game timer display
    void updateGameTimer();
private:
    // Helper function to move the player
    void movePlayer(int newX, int newY);
    // Helper function to check if the player has won
    void checkWinCondition();
    // Helper function to display game over message
    void showGameOverDialog(bool won);
    // Helper function to update the player's path line
    void updatePlayerPathLine();

    GridView* gridView; // Using a pointer to GridView to manage the maze display
    grid currentMazeGrid; // Copy of the maze grid for game logic
    int mazeWidth;
    int mazeHeight;
    int playerStartIndex;
    int playerEndIndex;

    QScatterSeries* playerElement; // Represents the player's current position
    QLineSeries* playerPathLine;   // Traces the player's path
    std::vector<QPointF> playerPathPoints; // Stores points for playerPathLine

    QTimer* gameTimer;       // Timer for the 20-second countdown
    QElapsedTimer elapsedGameTimer; // To track actual elapsed time
    QLabel* timerDisplayLabel; // Label to show the countdown timer

    int currentPlayerX;
    int currentPlayerY;
    int timeLimitSeconds; // 20 seconds as per requirement
    bool gameStarted;
};

#endif // PLAYERMAZEWINDOW_H
