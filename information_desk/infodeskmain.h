#ifndef INFODESKMAIN_H
#define INFODESKMAIN_H

#include <QMainWindow>
#include <QDialog>
#include <ui_booksearchdialog.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class InfoDeskMain;
}
QT_END_NAMESPACE

class InfoDeskMain : public QMainWindow
{
    Q_OBJECT

public:
    InfoDeskMain(QWidget *parent = nullptr);
    ~InfoDeskMain();

private slots:
    void onBookSearchClicked();

private:
    Ui::InfoDeskMain *ui;
};
#endif // INFODESKMAIN_H
