#ifndef BOOKSEARCHDIALOG_H
#define BOOKSEARCHDIALOG_H

#include <QDialog>
#include <QTableView>
#include <QStandardItemModel>
#include <QNetworkAccessManager>

namespace Ui {
class BookSearchDialog;
}

class BookSearchDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BookSearchDialog(QWidget *parent = nullptr);
    ~BookSearchDialog();

private slots:
    void on_searchBtn_clicked();

    void on_bookSearchWindow_textChanged();

private:
    Ui::BookSearchDialog *ui;

    QTableView* tableView;
    QStandardItemModel* model;
    QString searchText;
    QNetworkAccessManager* manager;
    void setupTable();
    QList<QList<QString>>* localBookInfo;
};

#endif // BOOKSEARCHDIALOG_H
