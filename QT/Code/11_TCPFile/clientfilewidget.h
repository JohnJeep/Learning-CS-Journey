#ifndef CLIENTFILEWIDGET_H
#define CLIENTFILEWIDGET_H

#include <QWidget>
#include <QTcpSocket>
#include <QFile>

namespace Ui {
class ClientFileWidget;
}

class ClientFileWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ClientFileWidget(QWidget *parent = nullptr);
    ~ClientFileWidget();

private slots:
    void on_btnConnect_clicked();

private:
    Ui::ClientFileWidget *ui;
    QTcpSocket* tcpSocket;
    QFile clientFile;
    QString cname;
    qint64 csize;
    qint64 receiveSize;      // 已经接收文件的大小
    bool isStart;          //标志位，是否为头部信息
};

#endif // CLIENTFILEWIDGET_H
