#include "widget.h"
#include "ui_widget.h"
#include <QStringList>
#include <QTableWidget>
#include <QDebug>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    // TableWidget控件
    ui->tableWidget->setColumnCount(3);  // 设置列数
    ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "姓名" << "性别" << "分数");  // 设置水平表头
    ui->tableWidget->setRowCount(4); // 设置行数

    // 设置表内的内容
    QStringList nameList;
    nameList << "李云龙" << "赵云" << "关羽" << "刘备";
    QList<QString> sexList;
    sexList << "男" << "男" << "男" << "男";
    int arr[] = {100, 98, 95, 99};

    for (int i = 0; i < 4; i++) {
        int col = 0;
        ui->tableWidget->setItem(i, col++, new QTableWidgetItem(nameList[i])); //  nameList[i])传入的类型为QString
        ui->tableWidget->setItem(i, col++, new QTableWidgetItem(sexList.at(i)));
        ui->tableWidget->setItem(i, col++, new QTableWidgetItem(QString::number(arr[i])));
    }
}

Widget::~Widget()
{
    delete ui;
}

