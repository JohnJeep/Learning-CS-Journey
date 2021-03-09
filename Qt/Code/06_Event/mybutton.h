#ifndef MYBUTTON_H
#define MYBUTTON_H
#include <QMouseEvent>
#include <QPushButton>

class MyButton : public QPushButton
{
    Q_OBJECT
public:
    explicit MyButton(QWidget *parent = nullptr);
    void mousePressEvent(QMouseEvent* ev);

signals:

};

#endif // MYBUTTON_H
