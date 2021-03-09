#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMenuBar>
#include <QToolBar>
#include <QDebug>
#include <QPushButton>
#include <QLabel>
#include <QDockWidget>
#include <QTextEdit>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ①set MenuBar，只有一个菜单栏
    QMenuBar* bar = menuBar();
    setMenuBar(bar);  // 将菜单栏放入窗口中

    // 创建菜单
    QMenu* fileMenu = bar->addMenu("File");
    QMenu* editMenu = bar->addMenu("Edit");

    // 创建菜单项
    fileMenu->addAction("New file");
    fileMenu->addSeparator();
    fileMenu->addAction("Open file");

    editMenu->addAction("Cut");
    editMenu->addAction("Past");
    editMenu->addAction("Copy");
    editMenu->addSeparator();
    editMenu->addAction("Select All");


    //②set toolbar，可以有多个
    QToolBar* toolbar = new QToolBar();
    addToolBar(Qt::LeftToolBarArea, toolbar);  // 添加一个工具栏，并设置默认的位置
    toolbar->setFloatable(false);
    toolbar->setAllowedAreas(Qt::LeftToolBarArea | Qt::BottomToolBarArea); // 后面需要移动的位置

    // 向toolbar中添加控件
    QPushButton* btn = new QPushButton("Welcome", this);
    btn->resize(20, 40);
    toolbar->addWidget(btn);

    // 向toolbar中添加内容
    toolbar->addAction("Edit");
    toolbar->addAction("Design");
    toolbar->addSeparator();      // 添加分割线
    toolbar->addAction("Debug");

    // ③设置状态栏，最多只能有一个
    QStatusBar* statBar = new QStatusBar();
    setStatusBar(statBar);   // 将状态栏添加到窗口中
    QLabel* label = new QLabel("label", this);
    statBar->addWidget(label);// 状态栏的位置，放左侧
    QLabel* rlabel = new QLabel("Rlabel", this);
    statBar->addPermanentWidget(rlabel);// 状态栏的位置，放右侧

    // ④设置浮动窗口，可以有多个
    QDockWidget* dockwidget = new QDockWidget("flow", this);
    addDockWidget(Qt::TopDockWidgetArea, dockwidget); // 默认浮动窗口的位置
//    dockwidget->setAllowedAreas(Qt::RightDockWidgetArea | Qt::TopDockWidgetArea);  // 后面可能移动的位置

    // ⑤设置核心部件，只能有一个
    QTextEdit* edit = new QTextEdit();
    setCentralWidget(edit);
}

MainWindow::~MainWindow()
{
    delete ui;
}

