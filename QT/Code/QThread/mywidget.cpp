#include "mywidget.h"
#include "ui_mywidget.h"
#include <QDebug>
#include <QThread>

MyWidget::MyWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MyWidget)
{
    ui->setupUi(this);
    myTimer = new QTimer(this);
    connect(myTimer, &QTimer::timeout, this, &MyWidget::deal);
}

// 需要处理的槽函数
void MyWidget::deal()
{
    static int num = 0;
    num ++;
    ui->lcdNumber->display(num);
}

MyWidget::~MyWidget()
{
    delete ui;
}


void MyWidget::on_pushButton_clicked()
{
    if (myTimer->isActive() == false)   // 定时器没有工作
    {
        myTimer->start(1000);
    }

//    QThread::sleep(3);
    qDebug() << "定时器处理过程";
//    tim->stop();
}
