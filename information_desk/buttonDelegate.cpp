#include "buttonDelegate.h"
#include <QPainter>
#include <QPixmap>

//버튼 생성 부에서 자동 호출
void ButtonDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option,
                           const QModelIndex& index) const
{

    QString buttonText = index.model()->data(index, Qt::DisplayRole).toString();
    if(buttonText.isEmpty())
        buttonText = btnText;

    QPushButton button;
    button.setText(buttonText);
    button.resize(option.rect.size());

    // 버튼 비활성화/색상 처리
    if(buttonText == "불가"){
        button.setEnabled(false); // 클릭 안 되도록
        button.setStyleSheet("background-color: red;");
    }
    else{
        button.setEnabled(true);
        button.setStyleSheet("background-color: white;");
    }
    // optional: 색상 변경
    QPalette pal = button.palette();
    if(!button.isEnabled())
        pal.setColor(QPalette::Button, Qt::lightGray);
    button.setPalette(pal);

    QPixmap pixmap(option.rect.size());
    button.render(&pixmap);
    painter->drawPixmap(option.rect.topLeft(), pixmap);
}

bool ButtonDelegate::editorEvent(QEvent* event, QAbstractItemModel* model,
                                 const QStyleOptionViewItem& option,
                                 const QModelIndex& index)
{
    if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        if (option.rect.contains(mouseEvent->pos())) {
            emit buttonClicked(index.row());
        }
    }
    return true;
}
