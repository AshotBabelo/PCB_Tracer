#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QPointF>
#include <QList>
#include <QLineF>
#include <QDebug>
#include <QMouseEvent>
#include "customgraphicsview.h"
#include "ui.h"


class MainWindow : public QMainWindow
{
    Q_OBJECT

    friend class UI;
public:
    MainWindow(QWidget *parent = nullptr);

    struct TraceVertex {
        QPointF point;
        int segmentBefore;
        int segmentAfter;
    };

    // Структура для хранения набора лучей из одной точки
    struct RaySet {
        QPointF startPoint;                        // Точка старта лучей
        QList<QGraphicsLineItem*> redLines;        // Красные лучи (не попавшие в цель)
        QList<QGraphicsLineItem*> allBlueRays;     // Все синие лучи (попавшие в цель)
        QList<QGraphicsLineItem*> bestBlueRay;     // Лучший синий луч
        qreal bestBlueLength;                      // Длина лучшего пути
        RaySet() : bestBlueLength(std::numeric_limits<qreal>::max()) {}
    };


    struct IntersectionResult {
        bool hasIntersection = false;
        QPointF point;
        QVector2D normal;
        double distance = std::numeric_limits<double>::max();
        QGraphicsItem* item = nullptr;
    };

    bool isInTargetMode() const { return targetMode; }
    bool isInObstacleMode() const { return obstacleMode; }
    bool isInAddMode() const { return addMode; }
    void setAddMode() { addMode = true; }
    void setReplaceMode() { addMode = false; }

private slots:
    void toggleTargetMode();
    void toggleObstacleMode();
    void clearAllRays();
    void toggleLinesVisibility();
    void onSceneClicked(const QPointF &point, Qt::MouseButton button);
    void performOptimization();
    void onMouseMoved(const QPointF &point);

private:

    UI* m_uiManager;

    void updateStatusLabel();

    void createObstacle(const QPointF &startPoint, const QPointF &endPoint);
    void setTargetPoint(const QPointF &point);
    void addTestComponents(const QString& filename);
    bool intersectRayWithExistingTraces(const QPointF &rayStart, const QVector2D &direction,
                                        double rayLength, QPointF &intersectionPoint,
                                        QVector2D &reflectionNormal, double &distance);
    bool isDirectPathClear(const QPointF& start, const QPointF& end);
    QList<QLineF> optimizeTraceSegments(const QList<QGraphicsLineItem*>& originalTrace);

    void performSBR(const QPointF &start, RaySet& raySet);
    QList<QLineF> traceRayWithBounces(const QPointF &start, double angleDeg, int maxBounces, bool& hitTarget);

    IntersectionResult findClosestObstacleIntersection(
        const QPointF& rayStart, const QVector2D& direction, double rayLength);

    QGraphicsScene *scene;
    CustomGraphicsView *view;
    QGraphicsEllipseItem *targetCircle;

    // Хранение всех наборов лучей
    QList<RaySet> allRaySets;

    // Состояние видимости и режима
    bool redLinesVisible;
    bool addMode;
    bool targetMode;
    bool obstacleMode;

    //Переменные для создания препядствия
    QPointF obstacleStartPoint;
    QGraphicsRectItem* tempObstacleRect = nullptr;
};


#endif // MAINWINDOW_H
