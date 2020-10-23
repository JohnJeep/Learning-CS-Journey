#include "widget.h"
#include "ui_widget.h"
#include <QFileDialog>


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle("文件服务器");
    ui->pushButtonSelect->setEnabled(false);
    ui->pushButtonSend->setEnabled(false);

    tServer = new QTcpServer(this);
    tServer->listen(QHostAddress::Any, 777);
    connect(tServer, &QTcpServer::newConnection, this, [=](){
        tSocket = tServer->nextPendingConnection();   // 取出已经建立好连接的套接字
        // 获取对方的IP和port
        QString ip = tSocket->peerAddress().toString();
        quint16 port = tSocket->peerPort();
        QString str = QString("[%1:%2] client连接server成功").arg(ip).arg(port);
        ui->textEdit->append(str); // 将IP和port显示在文本区内

        // client与server连接成功后才可操作按钮
        ui->pushButtonSelect->setEnabled(true);
        ui->pushButtonSend->setEnabled(true);

//        connect(tSocket, &QTcpSocket::readyRead, this, [=](){
//           QByteArray arr = tSocket->readAll();  // 获取客户端的信息
//           if (QString(arr) == "file done")
//           {
//               //文件接收完毕
//               ui->textEdit->append("文件发送完成");
//               myFile.close();

//               // 关闭客户端
//               tSocket->disconnectFromHost();
//               tSocket->close();
//           }
//        });
    });

    // 定时器操作，当前只做启动
    connect(&timer, &QTimer::timeout, this, [=](){
        timer.stop();
        clientSendData();
    });
}

Widget::~Widget()
{
    delete ui;
}

// 选择打开文件
void Widget::on_pushButtonSelect_clicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "open", "../");  // server选择要打开的文件路径
    if (filePath.isEmpty() == false)
    {
        // 文件信息初始化
        fname.clear();
        fsize = 0;
        fsend = 0;

        QFileInfo info;
        fsize = info.size();// 获取文件大小
        fname = info.fileName(); // 获取文件名称

        // 只读方式打开指定的文件
        myFile.setFileName(filePath);
        bool isOpen =  myFile.open(QIODevice::ReadOnly);
        if (isOpen == false)
        {
            qDebug() << "只读方式打开文件失败";
        }
        ui->textEdit->append(filePath); // 文本区提示打开的文件路径
        ui->pushButtonSelect->setEnabled(false);
        ui->pushButtonSend->setEnabled(true);
    }
    else
    {
        qDebug() << "打开文件路径错误";
    }
}

// 发送文件信息
void Widget::on_pushButtonSend_clicked()
{

   // 1. 先发送头部信息：name, size
    QString head = QString("(%1#%2)").arg(fname).arg(fsize);
    qint64 len = tSocket->write(head.toUtf8());
     if (len > 0)  // 头部信息发送成功
     {
         // 保证头部发完后，再发送数据，防止TCP黏包
         timer.start(10);
     }
     else
     {
         qDebug() << "头部信息发送失败";
         ui->pushButtonSelect->setEnabled(true);
         ui->pushButtonSend->setEnabled(false);
     }
}

// 发送文件数据函数
void Widget::clientSendData()
{
    ui->textEdit->append("正在发送数据...");
    char buf[4*1024] = {0};   //每次发送数据的大小
    qint64 len = 0;

    do
    {
        // 从文件中读数据
        len = myFile.read(buf, sizeof(buf));
        len = tSocket->write(buf, len); // server 发送多少数据，client就读多少数据
        fsend += len;
    }while(len > 0);     // 发送数据的长度大于0，则一直发送
    if(fsend == fsize)
    {
        ui->textEdit->append("文件发送完成");
        myFile.close();

        // 关闭客户端
        tSocket->disconnectFromHost();
        tSocket->close();
    }
}
