#ifndef TEACHER_H
#define TEACHER_H

#include <QObject>

class Teacher : public QObject
{
    Q_OBJECT
public:
    explicit Teacher(QObject *parent = nullptr);

signals:
    // 自定义信号
    // 返回值是void，只需要声明，不需要实现
    // 函数中可以有参数，可以重载
    void speaking();
    void speaking(QString str);
    //void speaking(bool pt);
};

#endif // TEACHER_H
