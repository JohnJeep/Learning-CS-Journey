#include "clientfilewidget.h"
#include "ui_clientfilewidget.h"
#include <QMessageBox>
#include <QDebug>
#include <QHostAddress>

ClientFileWidget::ClientFileWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ClientFileWidget)
{
    ui->setupUi(this);
    setWindowTitle("文件客户端");
    tcpSocket = new QTcpSocket(this);
    isStart = true;
    ui->progressBar->setValue(0);

    connect(tcpSocket, &QTcpSocket::readyRead, this, [=](){
        QByteArray array = tcpSocket->readAll(); //取出接收的内容
        if (isStart == true)
        {
            isStart = false;
            // 解析头部信息：name, size
            cname = QString(array).section("#", 0, 0);  // 按照 # 解析，开始为0 ，结束为0
            csize = QString(array).section("#", 1, 1).toInt();
            receiveSize = 0;
            clientFile.setFileName(cname);  // 关联文件名字

            //只写方式方式打开文件
            bool isOpen = clientFile.open(QIODevice::WriteOnly);
            if (isOpen == false)
            {
                qDebug() << "只写方式方式打开文件失败";
                tcpSocket->disconnectFromHost();
                tcpSocket->close();
                return ; //如果打开文件失败，中断函数
            }
            //弹出对话框，显示接收文件的信息
            QString str = QString("接收的文件：%1, %2KB").arg(cname).arg(csize/1024);  // 将接收的文件信息显示出来
            QMessageBox::information(this, "文件信息", str);
//            QMessageBox::resize();

            //设置进度条
            ui->progressBar->setMinimum(0);
            ui->progressBar->setMaximum(csize/1024);
            ui->progressBar->setValue(0);
        }
        else
        {
            // 文件的内容信息
            qint64 len = clientFile.write(array);
            if (len > 0)
            {
                receiveSize += len;
                qDebug() << "file len = " << len;
            }

            ui->progressBar->setValue(receiveSize/1024); // 更新进度条

            // 文件接收完毕
            if (receiveSize == csize)
            {
                tcpSocket->write("file done"); //先给服务发送(接收文件完成的信息)
                QMessageBox::information(this, "完成", "文件完成接收");
                clientFile.close();
                tcpSocket->disconnectFromHost();
                tcpSocket->close();
            }
        }
    });
}

ClientFileWidget::~ClientFileWidget()
{
    delete ui;
}

void ClientFileWidget::on_btnConnect_clicked()
{
    // 获取服务器的IP和port
    QString cli_ip =  ui->lineEditIP->text();
    quint16 cli_port = ui->lineEditPort->text().toInt();
    tcpSocket->connectToHost(QHostAddress(cli_ip), cli_port); // client主动和服务器连接
    isStart = true;
    ui->progressBar->setValue(0);
}
