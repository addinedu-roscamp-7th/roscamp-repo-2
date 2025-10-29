#ifndef DOBBYLOAN_H
#define DOBBYLOAN_H

#include <QDialog>
#include <qnetworkaccessmanager.h>

namespace Ui {
class DobbyLoan;
}

class DobbyLoan : public QDialog
{
    Q_OBJECT

public:
    explicit DobbyLoan(QWidget *parent = nullptr);
    ~DobbyLoan();


private slots:
    void on_initWindowbtnClick();

private:
    Ui::DobbyLoan *ui;
    QNetworkAccessManager* manager;
    void send_pickup();
};

#endif // DOBBYLOAN_H
