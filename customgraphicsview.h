#ifndef CUSTOMGRAPHICSVIEW_H
#define CUSTOMGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QMouseEvent>
#include <QPointF>

class CustomGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit CustomGraphicsView(QWidget *parent = nullptr);

signals:
    void pointClicked(const QPointF &point, Qt::MouseButton button);

protected:
    void mousePressEvent(QMouseEvent *event) override;
};

#endif // CUSTOMGRAPHICSVIEW_H
