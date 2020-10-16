#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QFileDialog>
#include <QFontDialog>
#include <QTreeWidget>
#include <QStringList>
#include <QTableWidget>
#include <QLabel>
#include <QMovie>
#include <QCompleter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->actionNew_File->setIcon(QIcon(":/Pictures/alert.png"));      // 添加图标
    ui->actionOpen_File->setIcon(QIcon(":/Pictures/category.gif"));

    // 设置LineEdit显示为密码显示的方式
    ui->lineEdit_2->setEchoMode(QLineEdit::Password);

   // 设置行编辑时，输入有提示
    QStringList list;
    list << "I love you" << "very good" << "pretty girl";
    QCompleter* cp = new QCompleter(list, this);
    cp->setCaseSensitivity(Qt::CaseInsensitive);  //不区分大小写
    ui->lineEdit->setCompleter(cp);

    // ①创建一个对话框，点击后弹出一个新的窗口
    connect(ui->actionNew_File, &QAction::triggered, this, [=](){
        // 创建非模态的对话框
        QDialog dlg(this);
        dlg.setWindowTitle("新窗口");
        dlg.resize(300, 200);
        dlg.exec();         // 窗口程序阻塞在此处

    });

    connect(ui->actionOpen_File, &QAction::triggered, this, [=](){
        //  创建非模态的对话框
        QDialog *dg = new QDialog(this);
        dg->setWindowTitle("新非模态窗口");
        dg->resize(400, 200);
        dg->show();
        dg->setAttribute(Qt::WA_DeleteOnClose);  // 每次关闭窗口时，释放创建窗口的资源
    });


    // ②消息对话框
    connect(ui->actionCritical, &QAction::triggered, this, [=](){
        // 错误消息
        QMessageBox::critical(this, "critical", "错误信息");
    });

    connect(ui->actionInformation, &QAction::triggered, this, [=](){
        // 信息对话框
        QMessageBox::information(this, "information", "正常消息");
    });

    connect(ui->actionQuestion, &QAction::triggered, this, [=](){
        // 选择提问
        QMessageBox::question(this, "question", "选择信息", QMessageBox::Ok | QMessageBox::No, QMessageBox::No);
    });

    connect(ui->actionWaring, &QAction::triggered, this, [=](){
        // 警告
        QMessageBox::warning(this, "warning", "警告信息");
    });


    // ③其它对话框
    // 颜色对话框：QColorDialog::getColor()
    connect(ui->actionColor, &QAction::triggered, this, [=](){
        QColor color = QColorDialog::getColor();
       qDebug() << "color RGB:" << color.red() << color.green() << color.blue();
    });

    // 文件对话框：QFileDialog::getOpenFileName()
    connect(ui->actionFilePath, &QAction::triggered, this, [=](){
        QString fl = QFileDialog::getOpenFileName(this, "打开文件", "./main.cpp");
        qDebug() << fl;
    });

    // 字体对话框：QFontDialog::getFont()
    connect(ui->actionFont, &QAction::triggered, this, [=](){
    bool flag;
    QFont font = QFontDialog::getFont(&flag, QFont("宋体", 12));
    qDebug() << "字体：" << font.family().toUtf8().data()
             << "字号：" << font.pointSize()
             << "是否是斜体：" << font.italic();
    });


    // ④单选与多选框操作
    ui->radioButton_A->setChecked(true); // 设置默认选项的值

    connect(ui->checkBoxFirst, &QCheckBox::stateChanged, this, [=](int state){
        qDebug() << state;    // 打印选项输出的值：2-选中 1-办选 0-未选
    });


    // ⑤利用代码的方式插入树对象控件
    ui->treeWidget->setHeaderLabels(QStringList() << "英雄" << "英雄介绍");
    QTreeWidgetItem* dema = new QTreeWidgetItem(QStringList() << "德玛");

    // 添加上层结点
    ui->treeWidget->addTopLevelItem(dema);

    // 添加子结点
    QStringList intro;
    intro << "坦克" << "坦克需要吸收力量";   // 将两个数据添加到QStringList中，库中重写了 << 操作符
    QTreeWidgetItem* childIntro = new QTreeWidgetItem(intro);
    dema->addChild(childIntro);

    // 下拉框 Combo box
    // QLabel可以显示文字和图片（静态图和动态图）
    ui->label_img_static->setPixmap(QPixmap(":/Pictures/category.gif"));
    ui->label_img->setScaledContents(true);        // 让图片自动适应Label内容的大小

    // 利用QLabel显示gif动态图片
    QMovie* mv = new QMovie(":/Pictures/waiticon.gif");
    ui->label_img->setMovie(mv);
    mv->start(); // 播放动图

    // Label与HTML标签结合
    ui->label_html->setText("<h1><a href=\"https://www.baidu.com\">百度一下 </a></h1>");
    ui->label_html->setOpenExternalLinks(true);   // 在外部打开连接
}

MainWindow::~MainWindow()
{
    delete ui;
}

