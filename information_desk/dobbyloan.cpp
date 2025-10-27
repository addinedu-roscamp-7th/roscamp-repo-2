#include "dobbyloan.h"
#include "ui_dobbyloan.h"

DobbyLoan::DobbyLoan(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DobbyLoan)
{
    ui->setupUi(this);
}

DobbyLoan::~DobbyLoan()
{
    delete ui;
}
