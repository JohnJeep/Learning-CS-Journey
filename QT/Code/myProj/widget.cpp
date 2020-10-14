#include "widget.h"
#include "ui_widget.h"
#include <QPushButton>
#include <QString>
#include <QtDebug>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    qDebug() << "执行QDebug";
    ui->setupUi(this);

    // set button1
    QPushButton* btn1 = new QPushButton;
    btn1->setParent(this);
    QString str1 = "按钮1";
    btn1->setText(str1);

    // set button2
    QPushButton* btn2 = new QPushButton;
    btn2->setParent(this);
    QString str2 = "按钮2";
    btn2->setText(str2);
    btn2->move(100, 0);

    QPushButton* btn3 = new QPushButton;
    btn3->setParent(this);
    QString str3 = "退出";
    btn3->setText(str3);
    btn3->move(200, 0);

    resize(600, 400);             // 重新设置窗口的大小，窗口可以缩放
///    setFixedSize(600, 400); // 设置固定大小的窗口
///    QString wt = "google";
///    setWindowTitle(wt);          // 重新设置窗口的名称

    connect(btn3, &QPushButton::clicked, this, &Widget::close); // 点击按钮退出

    // 创建对象
    this->te = new Teacher();
    this->stu = new Student();

    // 实现没有函数重载时，自定义的信号与槽函数，只执行一次
//    connect(te, &Teacher::speaking, stu, &Student::listening);
//    attendClass();

    // 实现有重载时，自定义的信号与槽函数，
    // 需要将信号与槽函数分别定义为函数指针，再传入函数的地址
    void (Teacher:: *teacherSignal)(QString) = &Teacher::speaking;
    void (Student:: *studentSlot)(QString) = &Student::listening;
    connect(te, teacherSignal, stu, studentSlot);  // 信号直接连接槽
    attendClass();  // 默认执行一次

    // ①信号连接槽
    QPushButton* attendBtn = new QPushButton("上课", this);
    attendBtn->move(0, 100);
//    connect(attendBtn, &QPushButton::clicked, this, &Widget::attendClass);   // error，要注意参数类型匹配的问题

    // ②信号连接信号，无参信号与槽函数连接
    void (Teacher:: *teacherSignal2)(void) = &Teacher::speaking;
    void (Student:: *studentSlot2)(void) = &Student::listening;
    connect(te, teacherSignal2, stu, studentSlot2);
    connect(btn1, &QPushButton::clicked, te, teacherSignal2);

    // 信号与槽函数的参数类型一致，都为bool类型
//    void (Teacher:: *teacherSignal3)(bool) = &Teacher::speaking;
//    void (Student:: *studentSlot3)(bool) = &Student::listening;
//    connect(te, teacherSignal3, stu, studentSlot3);
//    connect(btn2, &QPushButton::clicked, te, teacherSignal3);

    // 利用lambda表达式实现关闭按钮
    QPushButton* btn4 = new QPushButton("close", this);
    btn4->move(300, 0);
    connect(btn4, &QPushButton::clicked, this, [=](){
        emit te->speaking("西班牙语");
        this->close();
    });
}

Widget::~Widget()
{
    delete ui;
}

void Widget::attendClass()
{
    emit te->speaking();        // 触发老师讲课的信号
    emit te->speaking("英语");   // 触发老师讲课的信号
}
