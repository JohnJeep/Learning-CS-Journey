#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QKeyEvent>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void timerEvent(QTimerEvent *tev);

    // 键盘捕获事件
    void keyPressEvent(QKeyEvent* ev);

private:
    Ui::Widget *ui;
    int timerId1;
    int timerId2;
};
#endif // WIDGET_H
