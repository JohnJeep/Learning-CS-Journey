#include "widget.h"
#include "udpwidget.h"
#include "udpmultiwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    UdpWidget u;
    u.show();
    UdpMultiWidget m;
    m.show();

    return a.exec();
}
