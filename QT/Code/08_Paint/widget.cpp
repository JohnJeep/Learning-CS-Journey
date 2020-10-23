#include "widget.h"
#include "ui_widget.h"
#include <QPainter>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::paintEvent(QPaintEvent *e)
{

//    QPainter(this);   // 第一种方法：指定当前设备为画图对象

    // 第二种方法
    QPainter(p);
    p.begin(this);
    p.drawPixmap(0, 0, width(), height(), QPixmap("../Image/guess.ico"));  // 画背景图片

    p.end();
}

