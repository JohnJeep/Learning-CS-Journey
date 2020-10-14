#include "widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    return a.exec();  // 让程序对象进入消息循环，代码阻塞到当前行，不会让程序一闪而过
}
