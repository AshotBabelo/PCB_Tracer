#include "mainwindow.h"
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QtMath>
#include <QVector2D>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent) {
    setWindowTitle("PCB Designer");
    resize(900, 700);

    // Создаем сцену и view
    scene = new QGraphicsScene(this);
    view = new CustomGraphicsView(this);
    view->setScene(scene);
    setCentralWidget(view);

    connect(view, &CustomGraphicsView::pointClicked, this, &MainWindow::onSceneClicked);

    addTestComponents();
    toggleRedLinesButton = new QPushButton("Скрыть красные лучи", this);
    toggleRedLinesButton->setGeometry(700, 20, 180, 30);
    connect(toggleRedLinesButton, &QPushButton::clicked, this, &MainWindow::toggleRedLinesVisibility);
}
void MainWindow::toggleRedLinesVisibility() {
    redLinesVisible = !redLinesVisible;

    // Показываем/скрываем красные
    for (QGraphicsLineItem *line : redLines) {
        line->setVisible(redLinesVisible);
    }

    // Показываем/скрываем все жёлтые, КРОМЕ лучшего
    for (QGraphicsLineItem *line : allYellowRays) {
        line->setVisible(redLinesVisible);
    }

    toggleRedLinesButton->setText(redLinesVisible ? "Скрыть красные лучи" : "Показать красные лучи");

}

void MainWindow::onSceneClicked(const QPointF &point, Qt::MouseButton button)
{
    if (button == Qt::LeftButton) {
        // Удаляем все старые красные лучи
        for (QGraphicsLineItem *line : redLines) {
            scene->removeItem(line);
            delete line;
        }
        redLines.clear();

        // Удаляем старый лучший желтый луч
        for (QGraphicsLineItem *line : bestYellowRay) {
            scene->removeItem(line);
            delete line;
        }
        bestYellowRay.clear();
        bestYellowLength = std::numeric_limits<qreal>::max();
        // Очистка всех желтых
        for (QGraphicsLineItem *line : allYellowRays) {
            scene->removeItem(line);
            delete line;
        }
        allYellowRays.clear();

        // Параметры трассировки
        int maxBounces = 15;
        double angleStep = 5.0;

        // Запускаем лучи под всеми углами
        for (double angle = 0.0; angle < 360.0; angle += angleStep) {
            launchRayTracing(point, angle, maxBounces);
        }

        // После трассировки скрываем все красные лучи (по умолчанию)
        for (QGraphicsLineItem *line : redLines) {
            line->setVisible(false);
        }

        redLinesVisible = false;
        toggleRedLinesButton->setText("Показать красные лучи");
    }
}

bool intersectRayCircle(const QPointF &rayStart, const QVector2D &direction,
                        const QPointF &circleCenter, double radius, double &t) {
    QVector2D m = QVector2D(rayStart - circleCenter);
    double b = QVector2D::dotProduct(m, direction);
    double c = QVector2D::dotProduct(m, m) - radius*radius;

    double discriminant = b*b - c;
    if (discriminant < 0) {
        return false; // пересечений нет
    }

    double sqrtDisc = qSqrt(discriminant);
    double t1 = -b - sqrtDisc;
    double t2 = -b + sqrtDisc;

    if (t1 >= 0) {
        t = t1;
        return true;
    }
    if (t2 >= 0) {
        t = t2;
        return true;
    }
    return false;
}


void MainWindow::launchRayTracing(const QPointF &start, double angleDeg, int maxBounces) {
    QPointF currentPos = start;
    double angleRad = qDegreesToRadians(angleDeg);
    QVector2D direction(qCos(angleRad), qSin(angleRad));
    direction.normalize();

    int bounces = 0;
    const double rayLength = 1000;
    const double epsilon = 0.01;

    QVector<QLineF> segments;
    bool hitCircle = false;

    // Параметры окружности
    QPointF circleCenter = targetCircle->sceneBoundingRect().center();
    double radius = targetCircle->rect().width() / 2;

    while (bounces < maxBounces) {
        QLineF ray(currentPos, currentPos + direction.toPointF() * rayLength);

        QGraphicsItem *closestItem = nullptr;
        QPointF closestPoint;
        QVector2D normal;
        qreal minDist = 1e9;

        for (QGraphicsItem *item : scene->items()) {
            if (item == targetCircle) continue;

            if (auto rectItem = qgraphicsitem_cast<QGraphicsRectItem*>(item)) {
                QPolygonF poly = rectItem->mapToScene(rectItem->rect());
                for (int i = 0; i < poly.size(); ++i) {
                    QPointF p1 = poly[i];
                    QPointF p2 = poly[(i + 1) % poly.size()];
                    QLineF edge(p1, p2);

                    QPointF intersectPoint;
                    if (ray.intersects(edge, &intersectPoint) == QLineF::BoundedIntersection) {
                        qreal dist = QLineF(currentPos, intersectPoint).length();
                        if (dist < minDist) {
                            minDist = dist;
                            closestPoint = intersectPoint;
                            closestItem = item;

                            QVector2D edgeVec = QVector2D(p2 - p1).normalized();
                            normal = QVector2D(-edgeVec.y(), edgeVec.x());
                        }
                    }
                }
            }
        }

        double tCircle;
        bool circleHit = intersectRayCircle(currentPos, direction, circleCenter, radius, tCircle);

        if (circleHit && tCircle < minDist) {
            QPointF intersectPoint = currentPos + direction.toPointF() * tCircle;
            segments.append(QLineF(currentPos, intersectPoint));
            hitCircle = true;
            break;
        }

        if (closestItem) {
            segments.append(QLineF(currentPos, closestPoint));

            // Отражаем луч
            direction = direction - 2 * QVector2D::dotProduct(direction, normal) * normal;
            direction.normalize();

            currentPos = closestPoint + direction.toPointF() * epsilon;
            bounces++;
        } else {
            segments.append(ray);
            break;
        }
    }

    // Рисуем линии
    if (hitCircle) {
        qreal totalLength = 0;
        for (const QLineF &seg : segments) {
            totalLength += seg.length();
        }

        // СОЗДАЁМ отдельные линии для allYellowRays
        QList<QGraphicsLineItem*> currentYellow;
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::yellow, 2));
            line->setVisible(false);  // скрыты по умолчанию
            allYellowRays.append(line);
            currentYellow.append(line);
        }

        // Если этот путь — лучший
        if (totalLength < bestYellowLength) {
            // Удаляем старый лучший путь
            for (QGraphicsLineItem *line : bestYellowRay) {
                scene->removeItem(line);
                delete line;
            }
            bestYellowRay.clear();

            // СОЗДАЁМ КОПИИ ЛИНИЙ ДЛЯ ЛУЧШЕГО ПУТИ (чтобы не пересекались с allYellowRays)
            for (const QLineF &seg : segments) {
                QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::yellow, 2));
                line->setVisible(true);  // показываем лучший путь всегда
                bestYellowRay.append(line);
            }

            bestYellowLength = totalLength;
        }
    } else {
        // Обычные красные лучи
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::red, 2));
            line->setVisible(false);  // скрыты по умолчанию
            redLines.append(line);
        }
    }

}


void MainWindow::addTestComponents() {

    const double boardWidth = 700;
    const double boardHeight = 500;
    QGraphicsRectItem *boardBorder = new QGraphicsRectItem(0, 0, boardWidth, boardHeight);
    boardBorder->setPos(50, 50);  // Отступ от края окна
    boardBorder->setPen(QPen(Qt::black, 3, Qt::SolidLine));  // Толщина 3px, черный цвет
    boardBorder->setBrush(Qt::NoBrush);  // Без заливки (только рамка)
    scene->addItem(boardBorder);



    QGraphicsRectItem *rectItem = new QGraphicsRectItem(0, 0, 120, 120);
    rectItem->setPos(100, 350);
    rectItem->setBrush(Qt::gray);
    scene->addItem(rectItem);

    QGraphicsRectItem *rectItem2 = new QGraphicsRectItem(0, 0, 120, 120);
    rectItem2->setPos(500, 330);
    rectItem2->setBrush(Qt::gray);
    scene->addItem(rectItem2);

    QGraphicsRectItem *rectItem3 = new QGraphicsRectItem(0, 0, 180, 120);
    rectItem3->setPos(570, 210);
    rectItem3->setBrush(Qt::gray);
    scene->addItem(rectItem3);

    QGraphicsRectItem *rectItem4 = new QGraphicsRectItem(0, 0, 80, 50);
    rectItem4->setPos(490, 210);
    rectItem4->setBrush(Qt::gray);
    scene->addItem(rectItem4);

    QGraphicsRectItem *rectItem5 = new QGraphicsRectItem(0, 0, 80, 90);
    rectItem5->setPos(600, 50);
    rectItem5->setBrush(Qt::gray);
    scene->addItem(rectItem5);

    QGraphicsRectItem *rectItem6 = new QGraphicsRectItem(0, 0, 80, 50);
    rectItem6->setPos(300, 110);
    rectItem6->setBrush(Qt::gray);
    scene->addItem(rectItem6);

    QGraphicsRectItem *rectItem7 = new QGraphicsRectItem(0, 0, 80, 40);
    rectItem7->setPos(300, 300);
    rectItem7->setBrush(Qt::gray);
    scene->addItem(rectItem7);

    const double radius = 20.0;
    targetCircle = new QGraphicsEllipseItem(-radius, -radius, 2*radius, 2*radius);
    targetCircle->setBrush(Qt::green);
    targetCircle->setPen(QPen(Qt::NoPen));
    targetCircle->setPos(715, 100);  // Позиция окружности (центр)
    scene->addItem(targetCircle);
}
