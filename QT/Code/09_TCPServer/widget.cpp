#include "widget.h"
#include "ui_widget.h"


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle("Server");
    server = nullptr;
    sock = nullptr;

    server = new QTcpServer(this);
    server->listen(QHostAddress::Any, 6666);
    connect(server, &QTcpServer::newConnection, this, [=](){
        sock = server->nextPendingConnection();   // 取出建立好的连接套接字

        // 获取client的IP和端口号
        QString ip = sock->peerAddress().toString();   // 将QHostAddress类型转化为QString类型
        qint16 port = sock->peerPort();
        QString tmp = QString("[%1, %2]:连接成功").arg(ip).arg(port);     // 显示IP和Port
        ui->textEditRead->setText(tmp);

        // 接收来自客户端发送的消息;注意：只有当sock取出来时，才能执行这步
        connect(sock, &QTcpSocket::readyRead, this, [=](){
            QByteArray arr = sock->readAll();            // 获取客户端的中的内容，然后再发送
            ui->textEditRead->append(arr);
        });
    });
}

Widget::~Widget()
{
    delete ui;
}

// 服务器向客户端发送消息
void Widget::on_pushButtonSend_clicked()
{
    if (sock == nullptr)
    {
        return;  // 做出错判断
    }
    QString str = ui->textEditWrite->toPlainText();   // 获取服务器端编辑区的内容
    sock->write(str.toUtf8().data());                 // 向客户端发送消息
}

// 服务器端与客户端主动断开连接
void Widget::on_pushButtonClose_clicked()
{
    if (sock == nullptr)
    {
        return;
    }
    sock->disconnectFromHost();
    sock->close();
    sock = nullptr;
    ui->textEditRead->setText("Server主动关闭");
}
