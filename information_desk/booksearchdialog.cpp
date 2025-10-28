#include "booksearchdialog.h" //도서 조회
#include "ui_booksearchdialog.h"
#include "bookinfolocation.h" //도서 위치 정보
#include "bookloan.h" //도서 대여
#include <QStandardItem>
#include <QVBoxLayout>
#include "buttonDelegate.h"
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrl>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>




BookSearchDialog::BookSearchDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::BookSearchDialog), model(new QStandardItemModel(this)), searchText("")
{
    ui->setupUi(this);  // 여기서 setupUi 사용

    //검색창 설정
    ui->bookSearchWindow->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->bookSearchWindow->setLineWrapMode(QTextEdit::NoWrap);
    ui->backBtn->setFlat(true);


    //검색결과 창
    ui->bookInfoTable->setMinimumSize(400, 100);  // 최소 크기
    ui->bookInfoTable->horizontalHeader()->setStretchLastSection(true); // 마지막 컬럼 자동 늘리기

    //NetworkManager 생성
    manager = new QNetworkAccessManager(this);

    localBookInfo = new QList<QList<QString>>;

    setupTable();
    connect(ui->backBtn, &QPushButton::clicked, this, &BookSearchDialog::on_backBtn);

}

BookSearchDialog::~BookSearchDialog()
{
    delete ui;
}


void BookSearchDialog::on_backBtn(){ //뒤로가기 버튼
    this->close();
}

void BookSearchDialog::setupTable()
{
    model->setHorizontalHeaderLabels({"ISBN","도서명", "저자", "출판사","위치","대출"});
    ui->bookInfoTable->setModel(model);
    ui->bookInfoTable->verticalHeader()->setVisible(false); //인덱스 삭제
    ui->bookInfoTable->setSortingEnabled(true); // 정렬 가능


    //위치 버튼 기능 구현
    ButtonDelegate* delegatLOC = new ButtonDelegate(this);
    ui->bookInfoTable->setItemDelegateForColumn(4, delegatLOC);
    connect(delegatLOC, &ButtonDelegate::buttonClicked, this, [this](int row){
        int currentRow = row;
        QString isbn = model->item(row, 0)->text(); // 위치를 조회하기 위한 ISBN

        QString urlString = QString("http://127.0.0.1:8000/infodesk/books/pickup").arg(isbn);
        QNetworkRequest request((QUrl(urlString)));
        request.setRawHeader("accept", "application/json");

        QNetworkReply* reply = manager->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply,currentRow](){
            BookInfoLocation dlg(this); //위치 창 객체

            if(reply->error() == QNetworkReply::NoError){
                QByteArray responseData = reply->readAll();
                QJsonDocument doc = QJsonDocument::fromJson(responseData);

                if(doc.isObject()){
                    QJsonObject obj = doc.object();

                    QString detail = QString(
                                         "%1, %2, %3, %4")
                                         .arg(obj["locationName"].toString())
                                         .arg(obj["locType"].toString())
                                         .arg(obj["zoneName"].toString())
                                         .arg(obj["floor"].toString());


                    QString bookInfo = NULL;
                    for(int col = 0; col < model->columnCount()-1; ++col){
                        bookInfo.append(",");
                        bookInfo.append(model->item(currentRow, col)->text());
                    }
                    qDebug() << "[" << bookInfo << "]";

                    detail.append(bookInfo);
                    dlg.setText(detail);   // 여기서 BookInfoLocation에 넣어줌
                    dlg.exec();
                }
            } else {
                qDebug() << "HTTP 요청 오류:" << reply->errorString();
                dlg.errorInfo(reply->errorString());
            }
            reply->deleteLater();
        });
    });



    //대출 버튼 기능 구현
    ButtonDelegate* delegateLOAN = new ButtonDelegate(this);
    ui->bookInfoTable->setItemDelegateForColumn(5, delegateLOAN); // 5번 col에 버튼 생성
    connect(delegateLOAN, &ButtonDelegate::buttonClicked, this, [this](int row){

        QString status = model->item(row, 5)->text();
        if(status == "대출"){                     // 대출 가능한 경우만 실행
            BookLoan dlg(this);
            dlg.set_book_info(localBookInfo);
            dlg.setText(row);
            dlg.setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
            dlg.exec();
        } else {
            qDebug() << "대출 불가 도서입니다!";
        }
    });
}



void BookSearchDialog::on_searchBtn_clicked() // 검색 버튼
{
    QString encodeKeyword = QUrl::toPercentEncoding(searchText);
    QString urlString = QString("http://127.0.0.1:8000/infodesk/books?keyword=%1&page=1&per_page=5")
                            .arg(encodeKeyword);
    QUrl url(urlString);

    QNetworkRequest request(url);
    request.setRawHeader("accept", "application/json");
    QNetworkReply* reply = manager->get(request);

    // 응답 처리
    connect(reply, &QNetworkReply::finished, this, [this, reply]() {

        if(reply->error() == QNetworkReply::NoError){
            QByteArray responseData = reply->readAll();

            // JSON 파싱
            QJsonDocument doc = QJsonDocument::fromJson(responseData);
            if(doc.isObject()){ // 최상위가 객체
                QJsonObject rootObj = doc.object();

                if(rootObj.contains("result") && rootObj["result"].isArray()){
                    QJsonArray array = rootObj["result"].toArray();

                    // 기존 테이블 초기화
                    model->removeRows(0, model->rowCount());

                    // JSON → 테이블 반영 (테이블에 버튼을 그림)
                    for(const QJsonValue &val : array){
                        QJsonObject obj = val.toObject();
                        QList<QStandardItem*> rowItems;
                        rowItems.append(new QStandardItem(obj["isbn"].toString()));
                        rowItems.append(new QStandardItem(obj["title"].toString()));
                        rowItems.append(new QStandardItem(obj["author"].toString()));
                        rowItems.append(new QStandardItem(obj["publisher"].toString()));
                        rowItems.append(new QStandardItem("보기")); // 위치 버튼

                        QString status = obj["status"].toString(); //대출 버튼
                        QString buttonLabel;
                        if(status == "대출가능"){
                            buttonLabel = "대출";}
                        else
                            buttonLabel = "불가";

                        rowItems.append(new QStandardItem(buttonLabel));
                        model->appendRow(rowItems);

                        //도서 조회 정보 로컬 저장(대출 여부 확인을 위해서)
                        QList<QString> localItems;
                        localItems.append(obj["isbn"].toString());
                        localItems.append(obj["title"].toString());
                        localItems.append(obj["author"].toString());
                        localItems.append(obj["publisher"].toString());
                        localItems.append(obj["status"].toString());
                        localBookInfo->append(localItems);
                    }
                    ui->bookInfoTable->resizeColumnsToContents();

                }
            }
        } else {
            qDebug() << "HTTP 요청 오류:" << reply->errorString();
        }

        reply->deleteLater();
    });
}



void BookSearchDialog::on_bookSearchWindow_textChanged()// 검색창 속성 설정
{
    searchText = ui->bookSearchWindow->toPlainText();
    ui->searchBtn->setEnabled(!searchText.isEmpty());

}

