#include "infodeskmain.h"
#include "ui_infodeskmain.h"
#include "booksearchdialog.h"

InfoDeskMain::InfoDeskMain(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::InfoDeskMain)
{
    ui->setupUi(this);
    connect(ui->book_search_btn, &QPushButton::clicked, this, &InfoDeskMain::onBookSearchClicked);

}

InfoDeskMain::~InfoDeskMain()
{
    delete ui;
}

void InfoDeskMain::onBookSearchClicked()
{
    BookSearchDialog dlg(this);
    dlg.setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    dlg.exec();
}
