#include "subwindow.h"
#include <QPushButton>

SubWindow::SubWindow(QWidget *parent) : QWidget(parent)
{
    QPushButton* btnSub = new QPushButton;

    btnSub->setParent(this);
    btnSub->setText("返回主窗口");
    btnSub->move(50, 50);
    setWindowTitle("子窗口");
    resize(600, 400);
    connect(btnSub, &QPushButton::clicked, this, &SubWindow::mySlot);  // 子窗口发送一个点击的信号
}

void SubWindow::mySlot()
{
    emit mySignal();
}
