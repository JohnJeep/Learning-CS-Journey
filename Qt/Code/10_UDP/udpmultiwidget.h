#ifndef UDPMULTIWIDGET_H
#define UDPMULTIWIDGET_H

#include <QWidget>
#include <QUdpSocket>
#include <QPushButton>

namespace Ui {
class UdpMultiWidget;
}

class UdpMultiWidget : public QWidget
{
    Q_OBJECT

public:
    explicit UdpMultiWidget(QWidget *parent = nullptr);
    ~UdpMultiWidget();

private slots:
    void on_btnSend_clicked();

    void on_btnClose_clicked();

private:
    Ui::UdpMultiWidget *ui;
    QUdpSocket* udpSocket_3;
    QPushButton* btn;
};

#endif // UDPMULTIWIDGET_H
