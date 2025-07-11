/********************************************************************************
** Form generated from reading UI file 'mainWindow.ui'
**
** Created by: Qt User Interface Compiler version 6.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCharts/QChartView>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtWidgets/QDial>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *centralwidget_2;
    QWidget *centralwidget_3;
    QWidget *centralwidget_4;
    QWidget *centralWidget;
    QGridLayout *gridLayout_3;
    QGridLayout *gridLayout;
    QHBoxLayout *hLayout;
    QCommandLinkButton *resetButton;
    QCommandLinkButton *runButton;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_4;
    QSpinBox *speedSpinBox;
    QPushButton *mazeButton;
    QHBoxLayout *horizontalLayout_4;
    QTabWidget *tabWidget;
    QWidget *simulationTab;
    QFormLayout *formLayout;
    QComboBox *interactionBox;
    QComboBox *algorithmsBox;
    QWidget *visualizationTab;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QDial *dialWidth;
    QLabel *label;
    QLCDNumber *lcdWidth;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QDial *dialHeight;
    QLabel *label_2;
    QLCDNumber *lcdHeight;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_3;
    QSlider *sliderMarker;
    QLabel *label_3;
    QLCDNumber *lcdMarker;
    QSpacerItem *verticalSpacer_3;
    QWidget *comparisonTab;
    QVBoxLayout *verticalLayout_2;
    QTableWidget *comparisonTable;
    QPushButton *clearComparisonButton;
    QChartView *gridView;
    QFrame *line;
    QStatusBar *statusbar_2;
    QStatusBar *statusbar_3;
    QStatusBar *statusbar_4;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1455, 769);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        centralwidget_2 = new QWidget(centralwidget);
        centralwidget_2->setObjectName("centralwidget_2");
        centralwidget_2->setGeometry(QRect(10, 10, 1421, 695));
        centralwidget_3 = new QWidget(centralwidget_2);
        centralwidget_3->setObjectName("centralwidget_3");
        centralwidget_3->setGeometry(QRect(0, 10, 1421, 675));
        centralwidget_4 = new QWidget(centralwidget_3);
        centralwidget_4->setObjectName("centralwidget_4");
        centralwidget_4->setGeometry(QRect(0, 0, 1421, 675));
        centralWidget = new QWidget(centralwidget_4);
        centralWidget->setObjectName("centralWidget");
        centralWidget->setGeometry(QRect(0, 0, 1421, 675));
        QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(centralWidget);
        gridLayout_3->setObjectName("gridLayout_3");
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        hLayout = new QHBoxLayout();
        hLayout->setSpacing(4);
        hLayout->setObjectName("hLayout");
        hLayout->setSizeConstraint(QLayout::SizeConstraint::SetMaximumSize);
        hLayout->setContentsMargins(0, -1, 0, -1);
        resetButton = new QCommandLinkButton(centralWidget);
        resetButton->setObjectName("resetButton");
        resetButton->setEnabled(true);

        hLayout->addWidget(resetButton);

        runButton = new QCommandLinkButton(centralWidget);
        runButton->setObjectName("runButton");
        runButton->setEnabled(true);
        runButton->setCheckable(false);

        hLayout->addWidget(runButton);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName("label_4");
        label_4->setScaledContents(true);
        label_4->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_4->setWordWrap(true);

        horizontalLayout_5->addWidget(label_4);

        speedSpinBox = new QSpinBox(centralWidget);
        speedSpinBox->setObjectName("speedSpinBox");
        speedSpinBox->setMinimum(1);
        speedSpinBox->setMaximum(10);

        horizontalLayout_5->addWidget(speedSpinBox);


        hLayout->addLayout(horizontalLayout_5);

        mazeButton = new QPushButton(centralWidget);
        mazeButton->setObjectName("mazeButton");
        mazeButton->setCheckable(true);
        mazeButton->setAutoRepeat(false);
        mazeButton->setAutoExclusive(false);

        hLayout->addWidget(mazeButton);


        gridLayout->addLayout(hLayout, 0, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName("tabWidget");
        tabWidget->setTabPosition(QTabWidget::TabPosition::North);
        tabWidget->setIconSize(QSize(16, 16));
        simulationTab = new QWidget();
        simulationTab->setObjectName("simulationTab");
        simulationTab->setMinimumSize(QSize(432, 581));
        simulationTab->setAutoFillBackground(true);
        formLayout = new QFormLayout(simulationTab);
        formLayout->setObjectName("formLayout");
        interactionBox = new QComboBox(simulationTab);
        interactionBox->setObjectName("interactionBox");

        formLayout->setWidget(0, QFormLayout::ItemRole::SpanningRole, interactionBox);

        algorithmsBox = new QComboBox(simulationTab);
        algorithmsBox->setObjectName("algorithmsBox");
        algorithmsBox->setEnabled(true);
        algorithmsBox->setEditable(false);
        algorithmsBox->setCurrentText(QString::fromUtf8(""));
        algorithmsBox->setInsertPolicy(QComboBox::InsertPolicy::NoInsert);
        algorithmsBox->setSizeAdjustPolicy(QComboBox::SizeAdjustPolicy::AdjustToContents);

        formLayout->setWidget(1, QFormLayout::ItemRole::FieldRole, algorithmsBox);

        tabWidget->addTab(simulationTab, QString());
        visualizationTab = new QWidget();
        visualizationTab->setObjectName("visualizationTab");
        visualizationTab->setAutoFillBackground(true);
        gridLayout_2 = new QGridLayout(visualizationTab);
        gridLayout_2->setObjectName("gridLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        dialWidth = new QDial(visualizationTab);
        dialWidth->setObjectName("dialWidth");
        dialWidth->setMinimum(5);
        dialWidth->setMaximum(30);
        dialWidth->setSliderPosition(10);
        dialWidth->setOrientation(Qt::Orientation::Vertical);
        dialWidth->setWrapping(false);
        dialWidth->setNotchesVisible(true);

        horizontalLayout->addWidget(dialWidth);

        label = new QLabel(visualizationTab);
        label->setObjectName("label");
        label->setScaledContents(true);
        label->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout->addWidget(label);

        lcdWidth = new QLCDNumber(visualizationTab);
        lcdWidth->setObjectName("lcdWidth");
        lcdWidth->setFrameShape(QFrame::Shape::StyledPanel);
        lcdWidth->setProperty("intValue", QVariant(20));

        horizontalLayout->addWidget(lcdWidth);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        dialHeight = new QDial(visualizationTab);
        dialHeight->setObjectName("dialHeight");
        dialHeight->setMinimum(5);
        dialHeight->setMaximum(30);
        dialHeight->setValue(10);
        dialHeight->setNotchesVisible(true);

        horizontalLayout_2->addWidget(dialHeight);

        label_2 = new QLabel(visualizationTab);
        label_2->setObjectName("label_2");
        label_2->setScaledContents(true);
        label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);

        horizontalLayout_2->addWidget(label_2);

        lcdHeight = new QLCDNumber(visualizationTab);
        lcdHeight->setObjectName("lcdHeight");
        lcdHeight->setFrameShape(QFrame::Shape::StyledPanel);
        lcdHeight->setProperty("intValue", QVariant(20));

        horizontalLayout_2->addWidget(lcdHeight);


        verticalLayout->addLayout(horizontalLayout_2);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        sliderMarker = new QSlider(visualizationTab);
        sliderMarker->setObjectName("sliderMarker");
        sliderMarker->setMinimum(1);
        sliderMarker->setMaximum(50);
        sliderMarker->setValue(25);
        sliderMarker->setOrientation(Qt::Orientation::Horizontal);

        horizontalLayout_3->addWidget(sliderMarker);

        label_3 = new QLabel(visualizationTab);
        label_3->setObjectName("label_3");
        label_3->setAutoFillBackground(false);
        label_3->setScaledContents(true);
        label_3->setAlignment(Qt::AlignmentFlag::AlignCenter);
        label_3->setWordWrap(true);

        horizontalLayout_3->addWidget(label_3);

        lcdMarker = new QLCDNumber(visualizationTab);
        lcdMarker->setObjectName("lcdMarker");
        lcdMarker->setFrameShape(QFrame::Shape::StyledPanel);
        lcdMarker->setDigitCount(5);
        lcdMarker->setProperty("value", QVariant(25.000000000000000));

        horizontalLayout_3->addWidget(lcdMarker);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);


        gridLayout_2->addLayout(verticalLayout, 0, 0, 1, 1);

        tabWidget->addTab(visualizationTab, QString());
        comparisonTab = new QWidget();
        comparisonTab->setObjectName("comparisonTab");
        verticalLayout_2 = new QVBoxLayout(comparisonTab);
        verticalLayout_2->setObjectName("verticalLayout_2");
        comparisonTable = new QTableWidget(comparisonTab);
        if (comparisonTable->columnCount() < 5)
            comparisonTable->setColumnCount(5);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        comparisonTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        comparisonTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        comparisonTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        comparisonTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        comparisonTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        comparisonTable->setObjectName("comparisonTable");

        verticalLayout_2->addWidget(comparisonTable);

        clearComparisonButton = new QPushButton(comparisonTab);
        clearComparisonButton->setObjectName("clearComparisonButton");

        verticalLayout_2->addWidget(clearComparisonButton);

        tabWidget->addTab(comparisonTab, QString());

        horizontalLayout_4->addWidget(tabWidget);

        gridView = new QChartView(centralWidget);
        gridView->setObjectName("gridView");
        gridView->setSizeAdjustPolicy(QAbstractScrollArea::SizeAdjustPolicy::AdjustToContents);

        horizontalLayout_4->addWidget(gridView);


        gridLayout->addLayout(horizontalLayout_4, 1, 0, 1, 1);


        gridLayout_3->addLayout(gridLayout, 0, 0, 1, 1);

        line = new QFrame(centralWidget);
        line->setObjectName("line");
        line->setFrameShape(QFrame::Shape::VLine);
        line->setFrameShadow(QFrame::Shadow::Sunken);

        gridLayout_3->addWidget(line, 0, 1, 1, 1);

        statusbar_2 = new QStatusBar(centralwidget_3);
        statusbar_2->setObjectName("statusbar_2");
        statusbar_2->setGeometry(QRect(0, 0, 3, 25));
        statusbar_3 = new QStatusBar(centralwidget_2);
        statusbar_3->setObjectName("statusbar_3");
        statusbar_3->setGeometry(QRect(0, 10, 3, 25));
        statusbar_4 = new QStatusBar(centralwidget);
        statusbar_4->setObjectName("statusbar_4");
        statusbar_4->setGeometry(QRect(10, 10, 3, 25));
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);
        interactionBox->setCurrentIndex(-1);
        algorithmsBox->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        resetButton->setText(QCoreApplication::translate("MainWindow", "Reset", nullptr));
        runButton->setText(QCoreApplication::translate("MainWindow", "Start PathFinding", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "SPEED SIMULATION", nullptr));
        mazeButton->setText(QCoreApplication::translate("MainWindow", "Generate Maze", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(simulationTab), QCoreApplication::translate("MainWindow", "Simulation", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "GRID WIDTH", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "GRID HEIGHT", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "MARKER SIZE", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(visualizationTab), QCoreApplication::translate("MainWindow", "Visualization", nullptr));
        QTableWidgetItem *___qtablewidgetitem = comparisonTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "Algorithm", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = comparisonTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "Time (s)", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = comparisonTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "Nodes Visited", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = comparisonTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "Path Length", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = comparisonTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("MainWindow", "Grid Size", nullptr));
        clearComparisonButton->setText(QCoreApplication::translate("MainWindow", "Clear Comparison Data", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(comparisonTab), QCoreApplication::translate("MainWindow", "Comparison", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
