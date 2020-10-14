#include "student.h"
#include <QDebug>

Student::Student(QObject *parent) : QObject(parent)
{

}

// 学生听课
void Student::listening()
{
   qDebug() << "Press btn1, void type, student listening";
}

void Student::listening(QString str)
{
    // toUtf8().data() 将QString数据类型转换为char *类型
    qDebug() << "QString type, student listent:" << str.toUtf8().data();
}

void Student::listening(bool)
{
    qDebug() << "Press btn2, bool type, student listening";
}
