#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "teacher.h"
#include "student.h"


QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT  // 是一个宏，允许类中使用信号和槽

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    void attendClass();

private:
    Ui::Widget *ui;
    Teacher* te;
    Student* stu;
};
#endif // WIDGET_H
