#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QTimer>
#include <QString>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    connect(ui->btn_set, &QPushButton::clicked, this, [=](){
        ui->widget->setNum(66);
    });
    connect(ui->btn_get, &QPushButton::clicked, this, [=](){
        qDebug() << ui->widget->getNum();
    });

    // 启动定时器
    timerId1 = startTimer(1000);   // 时间间隔，单位为ms
    timerId2 = startTimer(500);

    // 推荐使用：第一种方法创建定时器
    QTimer *tm = new QTimer(this);   // 传递this的作用：将timer对象挂载到对象树上，不需要手动释放内存
    tm->start(2000);
    static int m = 10;
    connect(tm, &QTimer::timeout, [=](){
        ui->label_3->setText(QString::number(m++));
        if (m == 20)
        {
            tm->stop();  // 停止定时器
        }
    });
}

// 定时器使用  第二种方法
void Widget::timerEvent(QTimerEvent *tev)
{
    if (tev->timerId() == timerId1){
        static int num = 1;   // 从0开始每次加一
        ui->label->setText(QString::number(num++));
    }
    if (tev->timerId() == timerId2){
        static int tmp = 100;   // 从0开始每次减一
        ui->label_2->setText(QString::number(tmp--));
    }
}

// 键盘事件
void Widget::keyPressEvent(QKeyEvent* ev)
{
    qDebug() << ev->key();
}


Widget::~Widget()
{
    delete ui;
}

