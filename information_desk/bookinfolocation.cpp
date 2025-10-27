#include "bookinfolocation.h"
#include "ui_bookinfolocation.h"

BookInfoLocation::BookInfoLocation(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::BookInfoLocation)
{
    ui->setupUi(this);
}

BookInfoLocation::~BookInfoLocation()
{
    delete ui;
}
void BookInfoLocation::setText(const QString &text)
{
    QStringList list = text.split(",", Qt::SkipEmptyParts); //도서 정도 + 도서 위치
    qDebug() << list;
    QString bookLocation;
    if(list.size() == 8){
        bookLocation = QString(" 제목: %1\n 저자: %2\n 출판사: %3\n 표준번호: %4\n 위치: %5")
                                   .arg(list[4]) .arg(list[5]). arg(list[6]) .arg(list[3]) //도서 정보
                                   .arg("해당 도서는 없습니다."); //도서 위치

    }

    if(list.size() == 9){
        bookLocation = QString(" 제목: %1\n 저자: %2\n 출판사: %3\n 표준번호: %4\n 위치: %5")
                                   .arg(list[5]) .arg(list[6]). arg(list[7]) .arg(list[4]) //도서 정보
                                   .arg(list[0] + list[3] + list[1] + list[2]); //도서 위치

    }

    ui->locationInfoEdit->setPlainText(bookLocation); // 도서 정보 및 위치 표시
}

void BookInfoLocation::errorInfo(const QString & errorInfo){
    qDebug() << errorInfo; //ISBN 없을 떄 오류 처리 (보류)
}
