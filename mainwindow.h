#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QPushButton>
#include <QPointF>
#include <QList>
#include <QLineF>
#include <QDebug>
#include <QMouseEvent>
#include <QLabel>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include "customgraphicsview.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
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

private slots:
    void toggleTargetMode();
    void toggleObstacleMode();
    void clearAllRays();
    void toggleRedLinesVisibility();
    void onSceneClicked(const QPointF &point, Qt::MouseButton button);
    void performOptimization();
    void onMouseMoved(const QPointF &point);

    void setAddMode();
    void setReplaceMode();
    void showTracingModeMenu();

private:
    void updateStatusLabel();
    void updateTargetButtonText();
    void updateObstacleButtonText();
    void createObstacle(const QPointF &startPoint, const QPointF &endPoint);
    void setTargetPoint(const QPointF &point);
    void addTestComponents(const QString& filename);
    bool intersectRayWithExistingTraces(const QPointF &rayStart, const QVector2D &direction,
                                        double rayLength, QPointF &intersectionPoint,
                                        QVector2D &reflectionNormal, double &distance);
    void launchRayTracing(const QPointF &start, double angleDeg, int maxBounces, RaySet& raySet);
    bool isDirectPathClear(const QPointF& start, const QPointF& end);
    QList<QLineF> optimizeTraceSegments(const QList<QGraphicsLineItem*>& originalTrace);
    QGraphicsScene *scene;
    CustomGraphicsView *view;
    QGraphicsEllipseItem *targetCircle;

    // Кнопки управления
    QPushButton *toggleRedLinesButton;
    QPushButton *optimizeButton;
    QPushButton *clearAllButton;
    QPushButton *setTargetButton;
    QPushButton *createObstacleButton;

    QLabel* statusLabel;          // Информативное поле с текущим режимом
    QMenu* tracingModeMenu;       // Всплывающее меню для режимов трассировки
    QAction* addModeAction;       // Действие для режима добавления
    QAction* replaceModeAction;

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
    QList<QGraphicsRectItem*> createdObstacles;
};

#endif // MAINWINDOW_H
