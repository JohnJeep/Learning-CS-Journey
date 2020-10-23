#include "mywidget.h"
#include "ui_mywidget.h"
#include "mybutton.h"
#include <QtDebug>
#include <QMessageBox>
#include <QEvent>

MyWidget::MyWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MyWidget)
{
    ui->setupUi(this);

    // Qt的样式表
    ui->pushButton->setStyleSheet("color: red;"
                                  "background-color: yellow;"
                                  "selection-color: purple;"
                                  "selection-background-color: blue;");

    connect(ui->pushButton_2, &MyButton::clicked, this, [=](){
       qDebug() << "MyWidget: clicked event";
    });
}

MyWidget::~MyWidget()
{
    delete ui;
}

void MyWidget::keyPressEvent(QKeyEvent *ev)
{
    qDebug() << ev->key();
}

void MyWidget::mousePressEvent(QMouseEvent *ev)
{
    qDebug() << "hello";
}

// 事件函数使用
bool MyWidget::event(QEvent *ev)
{
    QKeyEvent* event = static_cast<QKeyEvent*>(ev);
    if (event->key() == Qt::Key_Q)
    {
        return QWidget::event(ev);
    }
    return true;
}

bool MyWidget::eventFilter(QObject *obj, QEvent *event)
{

}


// 关闭与忽略事件，常常用作窗口的关闭与确认
void MyWidget::closeEvent(QCloseEvent* event)
{
    int ret = QMessageBox::question(this, "提示窗口", "确认要关闭窗口吗？", QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret == QMessageBox::Yes)
    {
        event->accept(); // 接收事件：事件接收后，不再往下传递
    }
    else
    {
        event->ignore();  // 忽略事件：当前事件不接收，实现继续向父组件传递，而不是父类（基类）
    }
}


