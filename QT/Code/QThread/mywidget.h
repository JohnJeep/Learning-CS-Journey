#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>
#include <QTimer>


QT_BEGIN_NAMESPACE
namespace Ui { class MyWidget; }
QT_END_NAMESPACE

class MyWidget : public QWidget
{
    Q_OBJECT

public:
    MyWidget(QWidget *parent = nullptr);
    ~MyWidget();
    void deal();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MyWidget *ui;
    QTimer* myTimer;
};
#endif // MYWIDGET_H
