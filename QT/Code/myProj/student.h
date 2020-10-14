#ifndef STUDENT_H
#define STUDENT_H

#include <QObject>

class Student : public QObject
{
    Q_OBJECT
public:
    explicit Student(QObject *parent = nullptr);

signals:

public slots:
    // 自定义槽函数，早期Qt版本必须写到public slots下，现在高级版本可以直接写在public下
    // 返回值为void，需要声明，也需要实现
    // 可以有参数，可以重载
    void listening();
    void listening(QString str);
    void listening(bool);
};

#endif // STUDENT_H
