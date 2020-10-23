#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QMovie>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    void dealNumberKey();
    void timerEvent(QTimerEvent* e);

private slots:
    void on_pushButton_start_clicked();

    void on_pushButton_exit_clicked();

    void on_pushButton_back_clicked();

    void on_pushButton_hint_clicked();

    void on_pushButton_clicked();

    void gameReturnButton();

private:
    Ui::Widget *ui;
    QMovie lossMove;  // 游戏失败显示动画
    QMovie winMove;   // 游戏获胜显示动画
    QString randStr;
    int gameTime;      // 游戏时间
    int gameTimerID;   // 游戏时间定时器ID
    int gameLossID;    // 失败动画定时器ID
    int gameWinID;     // 成功动画定时器ID
    QString resultStr;        // 结果数
};
#endif // WIDGET_H
