#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "customgraphicsview.h"
#include <QGraphicsScene>
#include <QPushButton>


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);

private:
    QPushButton *toggleRedLinesButton;
    bool redLinesVisible = false;
    QList<QGraphicsLineItem*> redLines;
    QGraphicsEllipseItem *targetCircle;
    QGraphicsScene *scene;
    CustomGraphicsView *view;
    QList<QGraphicsLineItem*> allYellowRays;
    QList<QGraphicsLineItem*> bestYellowRay;
    qreal bestYellowLength = std::numeric_limits<qreal>::max();
    void toggleRedLinesVisibility();
    void onSceneClicked(const QPointF &point, Qt::MouseButton button);
    void addTestComponents();
    void launchRayTracing(const QPointF &start, double angleDeg, int maxBounces);
    bool rayIntersectsCircle(const QPointF &rayStart, const QVector2D &direction, const QPointF &circleCenter, double radius, double &t);
};
#endif // MAINWINDOW_H
