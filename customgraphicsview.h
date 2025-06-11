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
    void mouseMoved(const QPointF &point);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override {
        QPointF scenePoint = mapToScene(event->pos());
        emit mouseMoved(scenePoint);
        QGraphicsView::mouseMoveEvent(event);
    }
};

#endif // CUSTOMGRAPHICSVIEW_H
