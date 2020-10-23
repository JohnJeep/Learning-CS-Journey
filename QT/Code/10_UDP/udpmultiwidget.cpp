#include "udpmultiwidget.h"
#include "ui_udpmultiwidget.h"

UdpMultiWidget::UdpMultiWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::UdpMultiWidget)
{
    ui->setupUi(this);
    setWindowTitle("UDP Window 3");
    udpSocket_3 = new QUdpSocket(this);
    udpSocket_3->bind(8083);  // 给当前窗口绑定端口号，IP为Any

    // 加入某个广播组播地址;组播地址必须是D类IP地址
//    udpSocket_3->joinMulticastGroup(QHostAddress("224.0.0.15"));
//    udpSocket_3->bind(QHostAddress::AnyIPv4, 8083);
//    udpSocket_3->leaveMulticastGroup(IP); // 退出组播: 指定哪个IP退出当前组播

    // 对方成功发送数据过来，自动触发readyRead()
    connect(udpSocket_3, &QUdpSocket::readyRead, this, [=](){
        //读对方发送的内容
        char buf[1024] = {0};
        QHostAddress cliAddr;
        quint16 port;
        qint64 len = udpSocket_3->readDatagram(buf, sizeof(buf), &cliAddr, &port);
        if (len > 0)
        {
            QString str = QString("[%1:%2] %3").arg(cliAddr.toString())
                                               .arg(port)
                                               .arg(buf);
            ui->textEdit->append(str); // 将结果显示在文本区内
        }
    });
}

UdpMultiWidget::~UdpMultiWidget()
{
    delete ui;
}

void UdpMultiWidget::on_btnSend_clicked()
{
    QString mip = ui->lineEditIP->text();
    quint16 mport = ui->lineEditPort->text().toInt();
    QString mstr = ui->textEdit->toPlainText();
    udpSocket_3->writeDatagram(mstr.toUtf8(), QHostAddress(mip), mport);
}

void UdpMultiWidget::on_btnClose_clicked()
{
//    connect(btn, &QPushButton::clicked, this, &UdpMultiWidget::close);
}
