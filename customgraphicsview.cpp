#include "customgraphicsview.h"
#include <QMouseEvent>

CustomGraphicsView::CustomGraphicsView(QWidget *parent)
    : QGraphicsView(parent)
{
    setMouseTracking(true);
}

void CustomGraphicsView::mousePressEvent(QMouseEvent *event)
{
    QPointF scenePos = mapToScene(event->pos());
    emit pointClicked(scenePos, event->button());
    QGraphicsView::mousePressEvent(event);
}
