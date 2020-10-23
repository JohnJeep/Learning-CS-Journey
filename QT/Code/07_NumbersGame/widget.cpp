#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QTime>
#include <QString>
#include <QMessageBox>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle("猜数字游戏");
    ui->stackedWidget->setCurrentIndex(0);   // 设置首页

    // 初始化数据，失败动画页面
    lossMove.setFileName(":/res/over.gif");
    ui->labelLoss->setMovie(&lossMove);      // 给标签设置动画
    ui->labelLoss->setScaledContents(true); // //让动画自动适应标签大小

    // 初始化数据，胜利动画页面
    winMove.setFileName(":/res/win.gif");
    ui->labelWin->setMovie(&winMove);
    ui->labelWin->setScaledContents(true);

    //猜数字游戏界面相应设置
    connect(ui->pushButton_0, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_1, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_2, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_3, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_4, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_6, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_7, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_8, &QPushButton::clicked, this, &Widget::dealNumberKey);
    connect(ui->pushButton_9, &QPushButton::clicked, this, &Widget::dealNumberKey);
}

Widget::~Widget()
{
    delete ui;
}


// 定时器事件处理
void Widget::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == gameTimerID)   // 游戏事件处理
    {
        gameTime--;
        ui->progressBar->setValue(gameTime); // 设置进度条
        if (gameTime == 0)
        {
            killTimer(gameTimerID);
            QMessageBox::information(this, "游戏结束", "时间已到了！！！");
            lossMove.start(); //启动动画
            ui->stackedWidget->setCurrentWidget(ui->pageLoss);  //切换失败动画页面
            gameLossID = startTimer(3000);  // 游戏失败动画页面维持多久
        }
    }
    else if (e->timerId() == gameLossID)  // 游戏失败动画事件处理
    {
        //停止动画，停止定时器，回到游戏设置页面
        lossMove.stop();
        killTimer(gameLossID);
        ui->stackedWidget->setCurrentWidget(ui->pageGame);
    }
    else if (e->timerId() == gameWinID)  // 游戏胜利动画事件处理
    {
        //停止动画，停止定时器，回到游戏设置页面
        winMove.stop();
        killTimer(gameWinID);
        ui->stackedWidget->setCurrentWidget(ui->pageGame);
    }
}

//数字键处理
void Widget::dealNumberKey()
{
    QObject* mySender = sender();      //获取信号接收者
    QPushButton* btn = (QPushButton*)mySender;
    if (btn != nullptr)
    {
       // 获取按钮内容
        QString numStr = btn->text();
        qDebug() << "numstr = " << numStr;
        resultStr += numStr;

        //数字不能以0开始
        if (resultStr.size() == 1 && resultStr == "0")
        {
            resultStr.clear();
        }
        if (resultStr.size() <= 4) //保证显示结果为4位
        {
            ui->textEdit->setText(resultStr);
            if (resultStr.size() == 4) // 输入的输入为四位数，才能出结果
            {
                if (resultStr > randStr)
                {
                    ui->textEdit->append("你猜的数字大于原数");
                }
                else if (resultStr < randStr)
                {
                    ui->textEdit->append("你猜的数字小于原数");
                }
                else
                {
                    ui->textEdit->append("恭喜你！你猜对了");
                    killTimer(gameTimerID);
                    QMessageBox::information(this, "胜利", "猜对了");

                    //切换到成功动画，启动定时器
                    ui->stackedWidget->setCurrentWidget(ui->pageWin);
                    gameWinID = startTimer(3000);
                }
                resultStr.clear(); // 清空界面中已有的结果
            }
        }
    }
}

// 开始游戏处理
void Widget::on_pushButton_start_clicked()
{
    // ①获取下拉框的时间; toInt()将字符串转换为int
    gameTime = ui->comboBox->currentText().toInt();
    qDebug() << gameTime;

    // ②切换到游戏界面
    ui->stackedWidget->setCurrentWidget(ui->pageGame);

    int num;
    qsrand(QTime(0, 0, 0).secsTo(QTime::currentTime())); //以从0时0分0秒到现在的秒数为种子
    while((num = qrand() % 10000) < 999)
    {
        NULL;
    }
    randStr = QString::number(num);
    qDebug() << randStr;

    // ③设置进度条
    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(gameTime);
    ui->progressBar->setValue(gameTime);

    // 启动定时器
    gameTimerID = 0;
    gameTimerID = startTimer(500); //启动定时器，以 500 毫秒作为时间间隔

    resultStr.clear();
    ui->textEdit->clear();
}

// 退出游戏
void Widget::on_pushButton_exit_clicked()
{
    this->close();  // 退出时直接关闭窗口
}

// 退格按钮
void Widget::on_pushButton_back_clicked()
{
    if (resultStr.size() == 1)
    {
        resultStr.clear();
        ui->textEdit->clear();
    }
    else
    {
        resultStr.chop(1); //截断最后一位字符
        ui->textEdit->setText(resultStr);
    }
}

// 提示按钮
void Widget::on_pushButton_hint_clicked()
{
    resultStr.clear();
    QString str = "rand number:" + randStr;
    ui->textEdit->setText(str);
}


void Widget::on_pushButton_clicked()
{
//    connect(ui->pushButton_return, &QPushButton::clicked, this, &Widget::gameReturnButton);

}

void Widget::gameReturnButton()
{
    ui->setCurrentIndex(0);
//    this->show(); // 主窗口
//    hide();
}
