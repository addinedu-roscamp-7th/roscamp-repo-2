#include "dobbyloan.h"
#include "ui_dobbyloan.h"
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

DobbyLoan::DobbyLoan(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DobbyLoan)
{
    ui->setupUi(this);
    manager = new QNetworkAccessManager(this);
    ui->guidL1->setAlignment(Qt::AlignCenter);
    send_pickup();
    connect(ui->initWindowbtn,&QPushButton::clicked, this, &DobbyLoan::on_initWindowbtnClick);
}

DobbyLoan::~DobbyLoan()
{
    delete ui;
}

void DobbyLoan::send_pickup(){
    //픽업 정보 전송 (회원정보 스캔 정보는 임시 데이터를 사용)
    int member_id = 1;
    QString barcode = "B00003";

    QString urlString = QString("http://192.168.0.132:4069/infodesk/books/pickup?memberID=%1&barcode=%2")
                            .arg(member_id)
                            .arg(barcode);

    QUrl url(urlString);
    QNetworkRequest request(url);

    QNetworkReply* reply = manager->post(request, QByteArray());  // 빈 바디
    connect(reply, &QNetworkReply::finished, this, [reply, this]() {
        if(reply->error() == QNetworkReply::NoError){
            QByteArray responseData = reply->readAll();
            QJsonDocument doc = QJsonDocument::fromJson(responseData);

            if(doc.isObject()){
                QJsonObject obj = doc.object();
                QString locationName = obj["locationName"].toString();
                if(locationName != "자리없음"){
                    // 정규식으로 숫자 추출
                    QRegularExpression re("(\\d+)$"); // 문자열 끝의 숫자 추출
                    QRegularExpressionMatch match = re.match(locationName);
                    QString boxNum = match.hasMatch() ? match.captured(1) : locationName;
                    ui->boxNumL->setText(boxNum + QString("번"));
                    ui->boxNumL->setStyleSheet("color: red; font-weight: bold;");
                    qDebug() << "위치 번호:" << boxNum; // 여기서 1만 출력됨


                    // 먼저 모든 라벨 색 초기화
                    ui->label_1->setStyleSheet("color: black; border: 1px solid black;");
                    ui->label_2->setStyleSheet("color: black; border: 1px solid black;");
                    ui->label_3->setStyleSheet("color: black; border: 1px solid black;");
                    ui->label_4->setStyleSheet("color: black; border: 1px solid black;");

                    // boxNum에 따라 해당 라벨 색 변경
                    int num = boxNum.toInt();
                    switch(num){
                    case 1:
                        ui->label_1->setStyleSheet("color: red; font-weight: bold; border: 1px solid black;");
                        break;
                    case 2:
                        ui->label_2->setStyleSheet("color: red; font-weight: bold; border: 1px solid black;");
                        break;
                    case 3:
                        ui->label_3->setStyleSheet("color: red; font-weight: bold; border: 1px solid black;");
                        break;
                    case 4:
                        ui->label_4->setStyleSheet("color: red; font-weight: bold; border: 1px solid black;");
                        break;
                    default:
                        // 해당 없는 경우는 아무것도 안함
                        break;
                    }
                }
                else{
                    ui->boxNumL->setVisible(false);
                    ui->guidL1->resize(490,171);
                    ui->guidL1->setText("현재 사용가능한 픽업대가 없습니다. \n불편을 끼쳐드려 죄송합니다.");
                    ui->guidL1->setAlignment(Qt::AlignCenter);
                    ui->guidL2->setVisible(false);
                     ui->guidL3->setVisible(false);
                    ui->boxTimeL->setVisible(false);

                    qDebug() << "자리가 없음";
                }

            }
        } else {
            qDebug() << "HTTP 오류:" << reply->errorString();
        }
        reply->deleteLater();
    });

}

void DobbyLoan::on_initWindowbtnClick(){
    foreach (QWidget *w, QApplication::topLevelWidgets()) {
        // "infodeskmain"이 아니라면 닫기
        if (w->objectName() != "InfoDeskMain") {
            w->close();
        }
    }
}
