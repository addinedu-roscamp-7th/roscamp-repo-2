#ifndef BOOKINFOLOCATION_H
#define BOOKINFOLOCATION_H

#include <QDialog>

namespace Ui {
class BookInfoLocation;
}

class BookInfoLocation : public QDialog
{
    Q_OBJECT

public:
    explicit BookInfoLocation(QWidget *parent = nullptr);
    ~BookInfoLocation();
    void setText(const QString &text);
    void errorInfo(const QString &erroinfo );


private:
    Ui::BookInfoLocation *ui;
};

#endif // BOOKINFOLOCATION_H
