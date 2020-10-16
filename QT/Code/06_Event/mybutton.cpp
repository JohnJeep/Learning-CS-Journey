/*自定义Button实现信号的接收：accept()和忽略：ignore()*/
#include "mybutton.h"
#include <QDebug>


MyButton::MyButton(QWidget *parent) : QPushButton(parent)
{

}

void MyButton::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton)
    {
        qDebug() << "Mouse left clicked";
//        ev->ignore();   // 忽略事件：当前事件不接收，实现继续向父组件传递，而不是父类（基类:MyButton,QPushButton）
    }
    else
    {
        qDebug() << "not left cliked";
    }
}


