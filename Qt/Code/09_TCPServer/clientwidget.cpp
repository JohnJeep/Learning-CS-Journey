#include "clientwidget.h"
#include "ui_clientwidget.h"
#include <QHostAddress>

ClientWidget::ClientWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ClientWidget)
{
    ui->setupUi(this);
    setWindowTitle("Client");
    clientSocket = nullptr;

    clientSocket = new QTcpSocket(this);

    // client显示连接成功信息
    connect(clientSocket, &QTcpSocket::connected, this, [=](){
        ui->textEditRead->setText("成功和服务器建立连接");
    });

    // 收到来自服务器发送的信息
    connect(clientSocket, &QTcpSocket::readyRead, this, [=](){
        QByteArray array = clientSocket->readAll();
//        ui->textEditRead->setText(array);
        ui->textEditRead->append(array);
    });
}

ClientWidget::~ClientWidget()
{
    delete ui;
}

void ClientWidget::on_brnClientCon_clicked()
{
    // 获取服务器的IP和port
    QString ip = ui->lineEditIP->text();
    qint16 port = ui->lineEditPort->text().toInt();
    // client主动与server建立连接
    clientSocket->connectToHost(QHostAddress(ip), port);
}

// 客户端向服务器端发送数据
void ClientWidget::on_btnClientSend_clicked()
{
    // 获取要发送的信息
    QString clientStr = ui->textEditWrite->toPlainText();
    // 然后再发数据
    clientSocket->write(clientStr.toUtf8().data());
}

// 客户端主动关闭服务器
void ClientWidget::on_btnClientClose_clicked()
{
    clientSocket->disconnectFromHost();
    clientSocket->close();
//    ui->textEditRead->setText("Client主动关闭");
    ui->textEditRead->append("Client主动关闭");
}
