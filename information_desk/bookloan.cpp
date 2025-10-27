#include "bookloan.h"
#include "ui_bookloan.h"

BookLoan::BookLoan(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::BookLoan)
{
    ui->setupUi(this);
}

BookLoan::~BookLoan()
{
    delete ui;
}
