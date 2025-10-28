#include "bookloan.h"
#include "ui_bookloan.h"
#include "dobbyloan.h"


BookLoan::BookLoan(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::BookLoan)
{
    ui->setupUi(this);

    connect(ui->cancelBtn, &QPushButton::clicked, this, &BookLoan::onCancelBtn);
    connect(ui->testBtn, &QPushButton::clicked, this, &BookLoan::onTestBtn);
}

BookLoan::~BookLoan()
{
    delete ui;
}

void BookLoan::setText(const int row){

    if(!BookLoan::book_info) return;

    const QList<QString>& rowdata = (*BookLoan::book_info)[row];

    ui->title->setText(QString("제목: ") + rowdata[1]);
    ui->author->setText(QString("저자: ") + rowdata[2]);
    ui->publisher->setText(QString("출판사: ") + rowdata[3]);
    ui->isbn->setText(QString("표준번호: ") + rowdata[0]);

}

void BookLoan::set_book_info(const QList<QList<QString>>* text){
    BookLoan::book_info = text;
}



void BookLoan::onCancelBtn(){
    this->close();
}

void BookLoan::onTestBtn(){
    this->close();
    DobbyLoan dlg(this);
    dlg.setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    dlg.exec();

}

