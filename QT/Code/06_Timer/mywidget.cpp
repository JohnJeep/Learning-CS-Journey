#include "mywidget.h"
#include "ui_mywidget.h"
#include <QDebug>

MyWidget::MyWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MyWidget)
{
    ui->setupUi(this);

    // QT库中封装好的QTimer定时器库，优先选择使用
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

void MyWidget::on_pushButtonStart_clicked()
{
    if (myTimer->isActive() == false)   // 定时器没有激活才会启动定时器
    {
        myTimer->start(500);
    }
    qDebug() << "定时器工作中...";
}

void MyWidget::on_pushButtonStop_clicked()
{
    if (myTimer->isActive() == true)
    {
        myTimer->stop(); // 定时器只有激活了才会关闭定时器
    }
}
