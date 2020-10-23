#include "widget.h"
#include "clientfilewidget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    ClientFileWidget c;
    c.show();
    return a.exec();
}
