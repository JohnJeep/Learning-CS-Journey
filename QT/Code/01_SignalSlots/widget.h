#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include "teacher.h"
#include "student.h"
#include "subwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT  // 是一个宏，允许类中使用信号和槽

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    // 自定义的槽函数
    void attendClass();
    void myMainWindow();
    void dealSubWindow();

private:
    Ui::Widget *ui;
    Teacher* te;
    Student* stu;
    QPushButton btnNew;
    SubWindow btnSubWin;
};
#endif // WIDGET_H
