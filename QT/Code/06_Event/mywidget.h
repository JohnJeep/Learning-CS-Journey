#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>
#include <QKeyEvent>

QT_BEGIN_NAMESPACE
namespace Ui { class MyWidget; }
QT_END_NAMESPACE

class MyWidget : public QWidget
{
    Q_OBJECT

public:
    MyWidget(QWidget *parent = nullptr);
    ~MyWidget();

    void keyPressEvent(QKeyEvent* ev);
    void closeEvent(QCloseEvent* event);
    void mousePressEvent(QMouseEvent *ev);
    bool event(QEvent* ev);
    bool eventFilter(QObject* obj, QEvent* event);   // 重写事件过滤器
private:
    Ui::MyWidget *ui;
};
#endif // MYWIDGET_H
