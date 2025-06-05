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

    optimizeButton = new QPushButton("Оптимизировать трассу", this);
    optimizeButton->setGeometry(700, 60, 180, 30);
    connect(optimizeButton, &QPushButton::clicked, this, &MainWindow::performOptimization);


}
void MainWindow::toggleRedLinesVisibility() {
    redLinesVisible = !redLinesVisible;

    // Показываем/скрываем красные
    for (QGraphicsLineItem *line : redLines) {
        line->setVisible(redLinesVisible);
    }

    // Показываем/скрываем все жёлтые, КРОМЕ лучшего
    for (QGraphicsLineItem *line : allBlueRays) {
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
        for (QGraphicsLineItem *line : bestBlueRay) {
            scene->removeItem(line);
            delete line;
        }
        bestBlueRay.clear();
        bestBlueLength = std::numeric_limits<qreal>::max();
        // Очистка всех желтых
        for (QGraphicsLineItem *line : allBlueRays) {
            scene->removeItem(line);
            delete line;
        }
        allBlueRays.clear();

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

        // СОЗДАЁМ отдельные линии для allBlueRays
        QList<QGraphicsLineItem*> currentBlue;
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 2));
            line->setVisible(false);  // скрыты по умолчанию
            allBlueRays.append(line);
            currentBlue.append(line);
        }

        // Если этот путь — лучший
        if (totalLength < bestBlueLength) {
            // Удаляем старый лучший путь
            for (QGraphicsLineItem *line : bestBlueRay) {
                scene->removeItem(line);
                delete line;
            }
            bestBlueRay.clear();

            // СОЗДАЁМ КОПИИ ЛИНИЙ ДЛЯ ЛУЧШЕГО ПУТИ (чтобы не пересекались с allBlueRays)
            for (const QLineF &seg : segments) {
                QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 2));
                line->setVisible(true);  // показываем лучший путь всегда
                bestBlueRay.append(line);
            }

            bestBlueLength = totalLength;
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


QList<MainWindow::TraceVertex> MainWindow::getVerticesFromTrace(const QList<QGraphicsLineItem*>& trace) {
    QList<TraceVertex> vertices;

    if (trace.size() < 2) return vertices;

    // Создаем вершины только в точках соединения сегментов (точки отражения)
    for (int i = 0; i < trace.size() - 1; ++i) {
        QPointF connectionPoint = trace[i]->line().p2();
        QPointF nextStartPoint = trace[i + 1]->line().p1();

        // Проверяем, что точки действительно соединяются
        if (QLineF(connectionPoint, nextStartPoint).length() < 2.0) {
            TraceVertex vertex;
            vertex.point = connectionPoint;
            vertex.segmentBefore = i;
            vertex.segmentAfter = i + 1;
            vertices.append(vertex);
        }
    }

    return vertices;
}

bool MainWindow::isDirectPathClear(const QPointF& start, const QPointF& end) {
    QLineF testPath(start, end);
    const double epsilon = 1.5;

    // Проверяем пересечение только с препятствиями
    for (QGraphicsItem *item : scene->items()) {
        if (item == targetCircle) continue;

        // Пропускаем все линии трасс
        if (qgraphicsitem_cast<QGraphicsLineItem*>(item)) continue;

        if (auto rectItem = qgraphicsitem_cast<QGraphicsRectItem*>(item)) {
            QRectF rect = rectItem->mapRectToScene(rectItem->rect());

            // Проверяем пересечение с границами прямоугольника
            QLineF edges[4] = {
                QLineF(rect.topLeft(), rect.topRight()),
                QLineF(rect.topRight(), rect.bottomRight()),
                QLineF(rect.bottomRight(), rect.bottomLeft()),
                QLineF(rect.bottomLeft(), rect.topLeft())
            };

            for (const QLineF& edge : edges) {
                QPointF intersection;
                if (testPath.intersects(edge, &intersection) == QLineF::BoundedIntersection) {
                    double distFromStart = QLineF(start, intersection).length();
                    double distFromEnd = QLineF(end, intersection).length();

                    if (distFromStart > epsilon && distFromEnd > epsilon) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

QList<QLineF> MainWindow::optimizeTraceSegments(const QList<QGraphicsLineItem*>& originalTrace) {
    if (originalTrace.isEmpty()) return {};

    // Получаем все сегменты из исходной трассы
    QList<QLineF> segments;
    for (auto* item : originalTrace) {
        segments.append(item->line());
    }

    if (segments.size() <= 1) return segments;

    // ВАЖНО: сохраняем начальную и конечную точки
    QPointF startPoint = segments.first().p1();
    QPointF endPoint = segments.last().p2();

    QList<TraceVertex> vertices = getVerticesFromTrace(originalTrace);
    bool optimizationMade = true;
    int maxIterations = 10000;
    int iteration = 0;

    while (optimizationMade && iteration < maxIterations) {
        optimizationMade = false;
        iteration++;

        // Пересоздаем вершины на основе текущих сегментов
        vertices.clear();
        for (int i = 0; i < segments.size() - 1; ++i) {
            QPointF connectionPoint = segments[i].p2();
            TraceVertex vertex;
            vertex.point = connectionPoint;
            vertex.segmentBefore = i;
            vertex.segmentAfter = i + 1;
            vertices.append(vertex);
        }

        // Для каждой вершины пытаемся найти оптимизацию
        for (int v = 0; v < vertices.size(); ++v) {
            TraceVertex vertex = vertices[v];

            // Получаем точки предыдущего и следующего сегментов
            QPointF prevStart = segments[vertex.segmentBefore].p1();
            QPointF nextEnd = segments[vertex.segmentAfter].p2();

            // Вычисляем направления
            QVector2D dirToPrev = QVector2D(prevStart - vertex.point).normalized();
            QVector2D dirToNext = QVector2D(nextEnd - vertex.point).normalized();

            const double minDist = 10.0;
            double distToPrevStart = QLineF(prevStart, vertex.point).length();
            double distToNextEnd = QLineF(vertex.point, nextEnd).length();
            double maxDist = std::min(distToPrevStart, distToNextEnd) * 0.8;
            const double step = maxDist / 10.0;


            double bestDist = 0.0;
            QLineF bestModifiedBefore, bestShortcut, bestModifiedAfter;
            bool foundBetterPath = false;

            for (double dist = minDist; dist <= maxDist; dist += step) {
                QPointF point1 = vertex.point + dirToPrev.toPointF() * dist;
                QPointF point2 = vertex.point + dirToNext.toPointF() * dist;

                double distToPrevStart = QLineF(prevStart, vertex.point).length();
                double distToNextEnd = QLineF(vertex.point, nextEnd).length();

                if (dist >= distToPrevStart * 0.99 || dist >= distToNextEnd * 0.99)
                    break;

                if (isDirectPathClear(point1, point2)) {
                    bestDist = dist;
                    bestModifiedBefore = QLineF(prevStart, point1);
                    bestShortcut = QLineF(point1, point2);
                    bestModifiedAfter = QLineF(point2, nextEnd);
                    foundBetterPath = true;
                    // ⚠️ НЕ прерываем, продолжаем поиск лучшего варианта
                }
            }

            if (foundBetterPath) {
                QList<QLineF> newSegments;
                for (int i = 0; i < vertex.segmentBefore; ++i)
                    newSegments.append(segments[i]);

                newSegments.append(bestModifiedBefore);
                newSegments.append(bestShortcut);
                newSegments.append(bestModifiedAfter);

                for (int i = vertex.segmentAfter + 1; i < segments.size(); ++i)
                    newSegments.append(segments[i]);

                segments = newSegments;
                optimizationMade = true;
                break; // Выходим из обработки этой итерации, т.к. сегменты обновлены
            }

            if (optimizationMade) break; // Начинаем новую итерацию
        }
    }

    // КРИТИЧЕСКИ ВАЖНО: проверяем и восстанавливаем связность
    if (!segments.isEmpty()) {
        // Убеждаемся, что первый сегмент начинается с правильной точки
        if (QLineF(segments.first().p1(), startPoint).length() > 2.0) {
            segments.first().setP1(startPoint);
        }

        // Убеждаемся, что последний сегмент заканчивается в правильной точке
        if (QLineF(segments.last().p2(), endPoint).length() > 2.0) {
            segments.last().setP2(endPoint);
        }

        // Проверяем связность между сегментами и исправляем при необходимости
        for (int i = 0; i < segments.size() - 1; ++i) {
            QPointF endOfCurrent = segments[i].p2();
            QPointF startOfNext = segments[i + 1].p1();

            double gap = QLineF(endOfCurrent, startOfNext).length();
            if (gap > 2.0) {
                // Исправляем разрыв
                segments[i + 1].setP1(endOfCurrent);
            }
        }
    }

    return segments;
}

void MainWindow::performOptimization() {
    if (bestBlueRay.isEmpty()) {
        qDebug() << "Нет трассы для оптимизации";
        return;
    }

    qDebug() << "Начинаем оптимизацию. Исходное количество сегментов:" << bestBlueRay.size();

    // Сохраняем исходную длину
    double originalLength = bestBlueLength;

    // Получаем оптимизированные сегменты
    QList<QLineF> optimizedSegments = optimizeTraceSegments(bestBlueRay);

    if (optimizedSegments.isEmpty()) {
        qDebug() << "Ошибка: оптимизация вернула пустой результат";
        return;
    }

    // Удаляем старую трассу
    for (auto* item : bestBlueRay) {
        scene->removeItem(item);
        delete item;
    }
    bestBlueRay.clear();

    // Создаем новую оптимизированную трассу
    QPen optimizedPen(Qt::blue, 3);
    for (const QLineF& segment : optimizedSegments) {
        QGraphicsLineItem* line = scene->addLine(segment, optimizedPen);
        line->setVisible(true);
        bestBlueRay.append(line);
    }

    // Пересчитываем длину
    bestBlueLength = 0;
    for (const QLineF& segment : optimizedSegments) {
        bestBlueLength += segment.length();
    }

    qDebug() << "Оптимизация завершена:";
    qDebug() << "стало:" << optimizedSegments.size();
    qDebug() << "Длина была:" << originalLength << "стала:" << bestBlueLength;
    qDebug() << "Сокращение:" << (originalLength - bestBlueLength) << "("
             << ((originalLength - bestBlueLength) / originalLength * 100) << "%)";
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
