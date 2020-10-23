#ifndef UDPWIDGET_H
#define UDPWIDGET_H

#include <QWidget>
#include <QUdpSocket>

namespace Ui {
class UdpWidget;
}

class UdpWidget : public QWidget
{
    Q_OBJECT

public:
    explicit UdpWidget(QWidget *parent = nullptr);
    ~UdpWidget();

private slots:
    void on_btnSend_clicked();

    void on_btnClose_clicked();

private:
    Ui::UdpWidget *ui;
    QUdpSocket* udpSocket_2;
};

#endif // UDPWIDGET_H
