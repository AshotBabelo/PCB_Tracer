#include "mainwindow.h"
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QtMath>
#include <QVector2D>

namespace {


void deleteGraphicsLines(QGraphicsScene* scene, QList<QGraphicsLineItem*>& lines) {
    for (QGraphicsLineItem* line : lines) {
        scene->removeItem(line);
        delete line;
    }
    lines.clear();
}

QGraphicsRectItem* addObstacle(QGraphicsScene* scene, double x, double y, double w, double h) {
    auto* item = new QGraphicsRectItem(0, 0, w, h);
    item->setPos(x, y);
    item->setBrush(Qt::gray);
    scene->addItem(item);
    return item;
}

}


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

    // Кнопка для переключения видимости красных лучей
    toggleRedLinesButton = new QPushButton("Скрыть красные лучи", this);
    toggleRedLinesButton->setGeometry(500, 20, 180, 30);
    connect(toggleRedLinesButton, &QPushButton::clicked, this, &MainWindow::toggleRedLinesVisibility);

    // Кнопка оптимизации
    optimizeButton = new QPushButton("Оптимизировать трассу", this);
    optimizeButton->setGeometry(500, 60, 180, 30);
    connect(optimizeButton, &QPushButton::clicked, this, &MainWindow::performOptimization);

    // Кнопка очистки всех лучей
    clearAllButton = new QPushButton("Очистить все лучи", this);
    clearAllButton->setGeometry(700, 20, 180, 30);
    connect(clearAllButton, &QPushButton::clicked, this, &MainWindow::clearAllRays);

    // Кнопка переключения режима
    toggleModeButton = new QPushButton("Режим: Замена", this);
    toggleModeButton->setGeometry(700, 60, 180, 30);
    connect(toggleModeButton, &QPushButton::clicked, this, &MainWindow::toggleMode);

    // По умолчанию режим добавления
    addMode = true;
    updateModeButtonText();
}

void MainWindow::toggleMode() {
    addMode = !addMode;
    updateModeButtonText();
}

void MainWindow::updateModeButtonText() {
    toggleModeButton->setText(addMode ? "Режим: Добавление" : "Режим: Замена");
}

void MainWindow::clearAllRays() {
    for (auto& raySet : allRaySets) {
        deleteGraphicsLines(scene, raySet.redLines);
        deleteGraphicsLines(scene, raySet.allBlueRays);
        deleteGraphicsLines(scene, raySet.bestBlueRay);
    }
    allRaySets.clear();
    currentRaySetIndex = -1;
    qDebug() << "Все лучи очищены";
}

void MainWindow::toggleRedLinesVisibility() {
    redLinesVisible = !redLinesVisible;

    // Переключаем видимость для всех наборов лучей
    for (auto& raySet : allRaySets) {
        // Показываем/скрываем красные линии
        for (QGraphicsLineItem *line : raySet.redLines) {
            line->setVisible(redLinesVisible);
        }

        // Показываем/скрываем все синие лучи (кроме лучших)
        for (QGraphicsLineItem *line : raySet.allBlueRays) {
            line->setVisible(redLinesVisible);
        }
    }

    toggleRedLinesButton->setText(redLinesVisible ? "Скрыть красные лучи" : "Показать красные лучи");
}

void MainWindow::onSceneClicked(const QPointF &point, Qt::MouseButton button)
{
    if (button == Qt::LeftButton) {
        if (!addMode) {
            // Режим замены - очищаем все лучи
            clearAllRays();
        }

        // Создаем новый набор лучей
        RaySet newRaySet;
        newRaySet.startPoint = point;
        newRaySet.bestBlueLength = std::numeric_limits<qreal>::max();

        // Параметры трассировки
        int maxBounces = 15;
        double angleStep = 5.0;

        // Запускаем лучи под всеми углами для нового набора
        for (double angle = 0.0; angle < 360.0; angle += angleStep) {
            launchRayTracing(point, angle, maxBounces, newRaySet);
        }

        // Добавляем набор в список
        allRaySets.append(newRaySet);
        currentRaySetIndex = allRaySets.size() - 1;

        // После трассировки скрываем красные лучи (по умолчанию)
        for (QGraphicsLineItem *line : newRaySet.redLines) {
            line->setVisible(false);
        }

        for (QGraphicsLineItem *line : newRaySet.allBlueRays) {
            line->setVisible(false);
        }

        redLinesVisible = false;
        toggleRedLinesButton->setText("Показать красные лучи");

        qDebug() << "Создан набор лучей #" << currentRaySetIndex << "из точки" << point;
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

// Новая функция для проверки пересечения луча с существующими трассами
bool MainWindow::intersectRayWithExistingTraces(const QPointF &rayStart, const QVector2D &direction,
                                                double rayLength, QPointF &intersectionPoint,
                                                QVector2D &reflectionNormal, double &distance) {

    QLineF ray(rayStart, rayStart + direction.toPointF() * rayLength);
    double minDistance = std::numeric_limits<double>::max();
    bool foundIntersection = false;

    const double epsilon = 1e-6;

    // Проверяем пересечение с лучшими путями всех существующих наборов лучей
    for (const auto& raySet : allRaySets) {
        for (const auto* traceLine : raySet.bestBlueRay) {
            QLineF existingSegment = traceLine->line();
            QPointF intersection;

            if (ray.intersects(existingSegment, &intersection) == QLineF::BoundedIntersection) {
                double dist = QLineF(rayStart, intersection).length();

                // Проверяем, что пересечение не слишком близко к началу луча
                if (dist > epsilon && dist < minDistance) {
                    minDistance = dist;
                    intersectionPoint = intersection;
                    foundIntersection = true;

                    // Вычисляем нормаль для отражения
                    QVector2D segmentDir = QVector2D(existingSegment.p2() - existingSegment.p1()).normalized();
                    reflectionNormal = QVector2D(-segmentDir.y(), segmentDir.x());

                    // Убеждаемся, что нормаль направлена в сторону падающего луча
                    if (QVector2D::dotProduct(reflectionNormal, -direction) < 0) {
                        reflectionNormal = -reflectionNormal;
                    }
                }
            }
        }
    }

    if (foundIntersection) {
        distance = minDistance;
        return true;
    }

    return false;
}

void MainWindow::launchRayTracing(const QPointF &start, double angleDeg, int maxBounces, RaySet& raySet) {
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

        // Проверяем пересечение с препятствиями (прямоугольниками)
        for (QGraphicsItem *item : scene->items()) {
            if (item == targetCircle) continue;

            // Пропускаем все линии (трассы)
            if (qgraphicsitem_cast<QGraphicsLineItem*>(item)) continue;

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

        // Проверяем пересечение с существующими трассами
        QPointF traceIntersection;
        QVector2D traceNormal;
        double traceDistance;
        bool hitExistingTrace = intersectRayWithExistingTraces(currentPos, direction, rayLength,
                                                               traceIntersection, traceNormal, traceDistance);

        // Проверяем пересечение с целевой окружностью
        double tCircle;
        bool circleHit = intersectRayCircle(currentPos, direction, circleCenter, radius, tCircle);

        // Определяем ближайшее пересечение
        double circleDistance = circleHit ? tCircle : std::numeric_limits<double>::max();
        double obstacleDistance = closestItem ? minDist : std::numeric_limits<double>::max();
        double traceDistanceValue = hitExistingTrace ? traceDistance : std::numeric_limits<double>::max();

        // Находим минимальное расстояние
        double minDistance = std::min({circleDistance, obstacleDistance, traceDistanceValue});

        if (circleHit && circleDistance == minDistance) {
            // Попали в целевую окружность
            QPointF intersectPoint = currentPos + direction.toPointF() * tCircle;
            segments.append(QLineF(currentPos, intersectPoint));
            hitCircle = true;
            break;
        }
        else if (hitExistingTrace && traceDistanceValue == minDistance) {
            // Отскочили от существующей трассы
            segments.append(QLineF(currentPos, traceIntersection));

            // Отражаем луч от трассы
            direction = direction - 2 * QVector2D::dotProduct(direction, traceNormal) * traceNormal;
            direction.normalize();

            currentPos = traceIntersection + direction.toPointF() * epsilon;
            bounces++;
        }
        else if (closestItem && obstacleDistance == minDistance) {
            // Отскочили от препятствия
            segments.append(QLineF(currentPos, closestPoint));

            // Отражаем луч
            direction = direction - 2 * QVector2D::dotProduct(direction, normal) * normal;
            direction.normalize();

            currentPos = closestPoint + direction.toPointF() * epsilon;
            bounces++;
        }
        else {
            // Ничего не задели, луч уходит в бесконечность
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

        // Создаём отдельные линии для allBlueRays
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 2));
            line->setVisible(false);  // скрыты по умолчанию
            raySet.allBlueRays.append(line);
        }

        // Если этот путь — лучший для данного набора
        if (totalLength < raySet.bestBlueLength) {
            // Удаляем старый лучший путь
            for (QGraphicsLineItem *line : raySet.bestBlueRay) {
                scene->removeItem(line);
                delete line;
            }
            raySet.bestBlueRay.clear();

            // Создаём копии линий для лучшего пути
            for (const QLineF &seg : segments) {
                QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 3));
                line->setVisible(true);  // показываем лучший путь всегда
                raySet.bestBlueRay.append(line);
            }

            raySet.bestBlueLength = totalLength;
        }
    } else {
        // Обычные красные лучи
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::red, 2));
            line->setVisible(false);  // скрыты по умолчанию
            raySet.redLines.append(line);
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

    // Проверяем пересечение с препятствиями
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

    // НОВОЕ: Проверяем пересечение с существующими трассами
    for (const auto& raySet : allRaySets) {
        for (const auto* traceLine : raySet.bestBlueRay) {
            QLineF existingSegment = traceLine->line();
            QPointF intersection;

            if (testPath.intersects(existingSegment, &intersection) == QLineF::BoundedIntersection) {
                double distFromStart = QLineF(start, intersection).length();
                double distFromEnd = QLineF(end, intersection).length();

                if (distFromStart > epsilon && distFromEnd > epsilon) {
                    return false;
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

    bool optimizationMade = true;
    int maxIterations = 1;
    int iteration = 0;

    while (optimizationMade && iteration < maxIterations) {
        optimizationMade = false;
        iteration++;

        // Пересоздаем вершины на основе текущих сегментов
        QList<TraceVertex> vertices;
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

            // Получаем начальную и конечную точки сегментов до и после вершины
            QPointF segmentBeforeStart = segments[vertex.segmentBefore].p1();
            QPointF segmentAfterEnd = segments[vertex.segmentAfter].p2();

            // Параметры для поиска точек соединения
            const double minDistFromEnds = 1.0;  // Минимальное расстояние от концов сегментов
            const double maxSteps = 20;          // Максимальное количество шагов поиска

            // Длины сегментов
            double beforeLength = segments[vertex.segmentBefore].length();
            double afterLength = segments[vertex.segmentAfter].length();

            // Максимальные расстояния для поиска (не доходим до концов)
            double maxDistBefore = beforeLength - minDistFromEnds;
            double maxDistAfter = afterLength - minDistFromEnds;

            if (maxDistBefore <= minDistFromEnds || maxDistAfter <= minDistFromEnds) {
                continue; // Сегменты слишком короткие для оптимизации
            }

            // Направления сегментов
            QVector2D dirBefore = QVector2D(vertex.point - segmentBeforeStart).normalized();
            QVector2D dirAfter = QVector2D(segmentAfterEnd - vertex.point).normalized();

            bool foundBetterPath = false;
            QLineF bestDirectConnection;
            QLineF bestModifiedBefore, bestModifiedAfter;
            double bestDistBefore = 0, bestDistAfter = 0;

            // Пробуем разные расстояния от концов сегментов к вершине отскока
            for (int stepBefore = 1; stepBefore <= maxSteps && !foundBetterPath; ++stepBefore) {
                double distFromStartBefore = minDistFromEnds + (maxDistBefore - minDistFromEnds) * stepBefore / maxSteps;
                QPointF pointOnBefore = segmentBeforeStart + dirBefore.toPointF() * distFromStartBefore;

                for (int stepAfter = 1; stepAfter <= maxSteps; ++stepAfter) {
                    double distFromVertexAfter = minDistFromEnds + (maxDistAfter - minDistFromEnds) * stepAfter / maxSteps;
                    QPointF pointOnAfter = vertex.point + dirAfter.toPointF() * distFromVertexAfter;

                    // Проверяем, можно ли провести прямую линию между этими точками
                    if (isDirectPathClear(pointOnBefore, pointOnAfter)) {
                        // Нашли возможное соединение, сохраняем его
                        bestDirectConnection = QLineF(pointOnBefore, pointOnAfter);
                        bestModifiedBefore = QLineF(segmentBeforeStart, pointOnBefore);
                        bestModifiedAfter = QLineF(pointOnAfter, segmentAfterEnd);
                        bestDistBefore = distFromStartBefore;
                        bestDistAfter = distFromVertexAfter;
                        foundBetterPath = true;

                        // Продолжаем поиск лучшего варианта (ближе к вершине отскока)
                        // но не прерываем цикл, чтобы найти оптимальный вариант
                    }
                }
            }

            // Если нашли лучший путь, применяем оптимизацию
            if (foundBetterPath) {
                QList<QLineF> newSegments;

                // Добавляем все сегменты до оптимизируемого
                for (int i = 0; i < vertex.segmentBefore; ++i) {
                    newSegments.append(segments[i]);
                }

                // Добавляем оптимизированные сегменты
                newSegments.append(bestModifiedBefore);
                newSegments.append(bestDirectConnection);
                newSegments.append(bestModifiedAfter);

                // Добавляем все сегменты после оптимизируемого
                for (int i = vertex.segmentAfter + 1; i < segments.size(); ++i) {
                    newSegments.append(segments[i]);
                }

                segments = newSegments;
                optimizationMade = true;

                qDebug() << "Оптимизация на итерации" << iteration
                         << ": соединили точки на расстояниях" << bestDistBefore
                         << "и" << bestDistAfter << "от вершины отскока";

                break; // Выходим из обработки этой итерации, т.к. сегменты обновлены
            }
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
    if (allRaySets.isEmpty()) {
        qDebug() << "Нет наборов лучей для оптимизации";
        return;
    }

    // Оптимизируем все наборы лучей
    for (int i = 0; i < allRaySets.size(); ++i) {
        RaySet& raySet = allRaySets[i];

        if (raySet.bestBlueRay.isEmpty()) {
            qDebug() << "Набор лучей #" << i << "не имеет трассы для оптимизации";
            continue;
        }

        qDebug() << "Оптимизируем набор лучей #" << i << ". Исходное количество сегментов:" << raySet.bestBlueRay.size();

        // Сохраняем исходную длину
        double originalLength = raySet.bestBlueLength;

        // Получаем оптимизированные сегменты
        QList<QLineF> optimizedSegments = optimizeTraceSegments(raySet.bestBlueRay);

        if (optimizedSegments.isEmpty()) {
            qDebug() << "Ошибка: оптимизация набора #" << i << "вернула пустой результат";
            continue;
        }

        // Удаляем старую трассу
        for (auto* item : raySet.bestBlueRay) {
            scene->removeItem(item);
            delete item;
        }
        raySet.bestBlueRay.clear();

        // Создаем новую оптимизированную трассу
        QPen optimizedPen(Qt::blue, 3);
        for (const QLineF& segment : optimizedSegments) {
            QGraphicsLineItem* line = scene->addLine(segment, optimizedPen);
            line->setVisible(true);
            raySet.bestBlueRay.append(line);
        }

        // Пересчитываем длину
        raySet.bestBlueLength = 0;
        for (const QLineF& segment : optimizedSegments) {
            raySet.bestBlueLength += segment.length();
        }

        qDebug() << "Оптимизация набора #" << i << "завершена:";
        qDebug() << "стало:" << optimizedSegments.size();
        qDebug() << "Длина была:" << originalLength << "стала:" << raySet.bestBlueLength;
        qDebug() << "Сокращение:" << (originalLength - raySet.bestBlueLength) << "("
                 << ((originalLength - raySet.bestBlueLength) / originalLength * 100) << "%)";
    }
}

void MainWindow::addTestComponents() {
    QGraphicsRectItem *boardBorder = new QGraphicsRectItem(0, 0, 700, 500);
    boardBorder->setPos(50, 50);
    boardBorder->setPen(QPen(Qt::black, 3));
    boardBorder->setBrush(Qt::NoBrush);
    scene->addItem(boardBorder);

    addObstacle(scene, 100, 350, 120, 120);
    addObstacle(scene, 500, 330, 120, 120);
    addObstacle(scene, 570, 210, 180, 120);
    addObstacle(scene, 490, 210, 80, 50);
    addObstacle(scene, 600, 50, 80, 90);
    addObstacle(scene, 300, 110, 80, 50);
    addObstacle(scene, 300, 300, 80, 40);

    const double radius = 20.0;
    targetCircle = new QGraphicsEllipseItem(-radius, -radius, 2*radius, 2*radius);
    targetCircle->setBrush(Qt::green);
    targetCircle->setPen(Qt::NoPen);
    targetCircle->setPos(715, 100);
    scene->addItem(targetCircle);
}
