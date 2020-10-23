#include "udpwidget.h"
#include "ui_udpwidget.h"

UdpWidget::UdpWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UdpWidget)
{
    ui->setupUi(this);
    setWindowTitle("UDP Window 2");

    udpSocket_2 = new QUdpSocket(this);
    udpSocket_2->bind(8082);  // 绑定端口号

    // 对方成功发送数据过来，自动触发readyRead()
    connect(udpSocket_2, &QUdpSocket::readyRead, this, [=](){
        //读对方发送的内容
        char buf[1024] = {0};
        QHostAddress cliAddr;
        quint16 port;
        qint64 len = udpSocket_2->readDatagram(buf, sizeof(buf), &cliAddr, &port);
        if (len > 0)
        {
            QString str = QString("[%1:%2] %3").arg(cliAddr.toString())
                                               .arg(port)
                                               .arg(buf);
            ui->textEdit->append(str); // 将结果显示在文本区内
        }
    });
}

UdpWidget::~UdpWidget()
{
    delete ui;
}

void UdpWidget::on_btnSend_clicked()
{
    // 1. 获取对方IP和port
    QString wip = ui->lineEditIP->text();
    quint16 wport = ui->lineEditPort->text().toUInt();

    // 2. 获取编辑区内容
    QString str = ui->textEdit->toPlainText();

    // 3. 给指定的IP发送数据
    udpSocket_2->writeDatagram(str.toUtf8(), QHostAddress(wip), wport);
}

void UdpWidget::on_btnClose_clicked()
{

}
