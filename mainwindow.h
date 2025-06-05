#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "customgraphicsview.h"
#include <QGraphicsScene>
#include <QPushButton>
#include <QVector2D>


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
    QList<QGraphicsLineItem*> allBlueRays;
    QList<QGraphicsLineItem*> bestBlueRay;
    qreal bestBlueLength = std::numeric_limits<qreal>::max();
    void toggleRedLinesVisibility();
    void onSceneClicked(const QPointF &point, Qt::MouseButton button);
    void addTestComponents();
    void launchRayTracing(const QPointF &start, double angleDeg, int maxBounces);
    bool rayIntersectsCircle(const QPointF &rayStart, const QVector2D &direction, const QPointF &circleCenter, double radius, double &t);

    struct TraceVertex {
        QPointF point;
        int segmentBefore; // индекс сегмента перед вершиной
        int segmentAfter;  // индекс сегмента после вершины
    };

    QPushButton *optimizeButton;
    QList<TraceVertex> getVerticesFromTrace(const QList<QGraphicsLineItem*>& trace);
    bool isDirectPathClear(const QPointF& start, const QPointF& end);
    QList<QLineF> optimizeTraceSegments(const QList<QGraphicsLineItem*>& originalTrace);
    void performOptimization();

};
#endif // MAINWINDOW_H
