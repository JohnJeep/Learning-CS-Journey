#ifndef CLIENTWIDGET_H
#define CLIENTWIDGET_H

#include <QWidget>
#include <QTcpSocket>

namespace Ui {
class ClientWidget;
}

class ClientWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ClientWidget(QWidget *parent = nullptr);
    ~ClientWidget();

private slots:
    void on_brnClientCon_clicked();

    void on_btnClientSend_clicked();

    void on_btnClientClose_clicked();

private:
    Ui::ClientWidget *ui;
    QTcpSocket* clientSocket;
};

#endif // CLIENTWIDGET_H
