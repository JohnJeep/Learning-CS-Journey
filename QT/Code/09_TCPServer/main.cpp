#include "widget.h"
#include "clientwidget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();

    ClientWidget cw;
    cw.show();
    return a.exec();
}
