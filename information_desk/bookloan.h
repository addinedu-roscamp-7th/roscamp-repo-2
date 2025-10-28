#ifndef BOOKLOAN_H
#define BOOKLOAN_H

#include <QDialog>
#include <qnetworkaccessmanager.h>

namespace Ui {
class BookLoan;
}

class BookLoan : public QDialog
{
    Q_OBJECT

public:
    explicit BookLoan(QWidget *parent = nullptr);
    ~BookLoan();
    void setText(const int row);
    void set_book_info(const QList<QList<QString>>* text); //로컬 도서 정보 가져오기


private slots:
    void onCancelBtn();
    void onTestBtn();

private:
    Ui::BookLoan *ui;
    const QList<QList<QString>>* book_info = nullptr; //로컬 도서 정보

};

#endif // BOOKLOAN_H
