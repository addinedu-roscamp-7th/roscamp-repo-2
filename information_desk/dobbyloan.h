#ifndef DOBBYLOAN_H
#define DOBBYLOAN_H

#include <QDialog>

namespace Ui {
class DobbyLoan;
}

class DobbyLoan : public QDialog
{
    Q_OBJECT

public:
    explicit DobbyLoan(QWidget *parent = nullptr);
    ~DobbyLoan();

private:
    Ui::DobbyLoan *ui;
};

#endif // DOBBYLOAN_H
