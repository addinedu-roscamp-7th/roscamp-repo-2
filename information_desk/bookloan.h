#ifndef BOOKLOAN_H
#define BOOKLOAN_H

#include <QDialog>

namespace Ui {
class BookLoan;
}

class BookLoan : public QDialog
{
    Q_OBJECT

public:
    explicit BookLoan(QWidget *parent = nullptr);
    ~BookLoan();

private:
    Ui::BookLoan *ui;
};

#endif // BOOKLOAN_H
