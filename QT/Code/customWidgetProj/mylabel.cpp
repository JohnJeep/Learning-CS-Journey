#include "mylabel.h"
#include <QMouseEvent>

MyLabel::MyLabel(QWidget *parent) : QLabel(parent)
{
    setMouseTracking(true);    // 设置鼠标是否追踪，鼠标直要移动，就会显示信息
}

// 鼠标事件的处理
void MyLabel::enterEvent(QEvent* event)
{
//    qDebug() << "鼠标点击";
}

void MyLabel::leaveEvent(QEvent* event)
{
//    qDebug() << "鼠标离开";
}


void MyLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)  // 鼠标左键按下生效
    {
        qDebug() << "鼠标按下";
    }
}

void MyLabel::mouseMoveEvent(QMouseEvent *event)
{
//    if (event->buttons() & Qt::LeftButton)     // 只有鼠标左键按下生效
    {
        QString str = QString("鼠标移动 x = %1 y = %2").arg(event->x()).arg(event->y());
        qDebug() << str;
    }
}

void MyLabel::mouseReleaseEvent(QMouseEvent *event)
{
    qDebug() << "鼠标释放";
}
