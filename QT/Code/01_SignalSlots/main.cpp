#include "widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);   // 有且仅有一个应用程序类的对象
    Widget w;
    w.show();                     // 窗口创建默认是隐藏的，需要手动去显示
    return a.exec();              // 让程序对象进入消息循环，代码阻塞到当前行，不会让程序一闪而过
}
