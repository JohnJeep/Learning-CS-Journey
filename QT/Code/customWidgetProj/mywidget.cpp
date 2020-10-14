#include "mywidget.h"
#include "ui_mywidget.h"
#include <QSlider>

MyWidget::MyWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MyWidget)
{
    ui->setupUi(this);

    // 自定义控件的实现
    void (QSpinBox:: *spSignal)(int) = &QSpinBox::valueChanged;
    connect(ui->spinBox, spSignal, ui->horizontalScrollBar, &QSlider::setValue);
    connect(ui->horizontalScrollBar, &QSlider::valueChanged, ui->spinBox, &QSpinBox::setValue);
}

void MyWidget::setNum(int num)
{
    ui->spinBox->setValue(num);
}
int MyWidget::getNum()
{
    return ui->spinBox->value();
}

MyWidget::~MyWidget()
{
    delete ui;
}
