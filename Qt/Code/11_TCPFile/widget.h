#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QTcpSocket>
#include <QTcpServer>
#include <QFile>
#include <QTimer>
#include <QDebug>
#include <QFileInfo>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void clientSendData();

private slots:
    void on_pushButtonSend_clicked();

    void on_pushButtonSelect_clicked();

private:
    Ui::Widget *ui;
    QTcpSocket* tSocket; // 监听套接字
    QTcpServer* tServer; // 通信套接字
    QFile myFile;        // 文件对象
    QString fname;
    qint64 fsize;
    qint64 fsend;      // 已经发送文件的大小
    QTimer timer;
};
#endif // WIDGET_H
