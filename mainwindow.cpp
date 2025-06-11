#include "mainwindow.h"
#include <QGraphicsRectItem>
#include <QGraphicsLineItem>
#include <QtMath>
#include <QVector2D>
#include <QFile>
#include <QDir>

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
    connect(view, &CustomGraphicsView::mouseMoved, this, &MainWindow::onMouseMoved);
    addTestComponents(QCoreApplication::applicationDirPath() + "/obstacles.txt");

    // Кнопки управления
    toggleRedLinesButton = new QPushButton("Скрыть красные лучи", this);
    toggleRedLinesButton->setGeometry(300, 20, 180, 30);
    connect(toggleRedLinesButton, &QPushButton::clicked, this, &MainWindow::toggleRedLinesVisibility);

    optimizeButton = new QPushButton("Оптимизировать трассу", this);
    optimizeButton->setGeometry(300, 60, 180, 30);
    connect(optimizeButton, &QPushButton::clicked, this, &MainWindow::performOptimization);

    clearAllButton = new QPushButton("Очистить все лучи", this);
    clearAllButton->setGeometry(500, 20, 180, 30);
    connect(clearAllButton, &QPushButton::clicked, this, &MainWindow::clearAllRays);

    toggleModeButton = new QPushButton("Режим: Замена", this);
    toggleModeButton->setGeometry(500, 60, 180, 30);
    connect(toggleModeButton, &QPushButton::clicked, this, &MainWindow::toggleMode);

    // Новая кнопка для задания конечной точки
    setTargetButton = new QPushButton("Задать конечную точку", this);
    setTargetButton->setGeometry(700, 20, 180, 30);
    connect(setTargetButton, &QPushButton::clicked, this, &MainWindow::toggleTargetMode);

    // Новая кнопка для создания препятствий
    createObstacleButton = new QPushButton("Создать препятствие", this);
    createObstacleButton->setGeometry(700, 60, 180, 30);
    connect(createObstacleButton, &QPushButton::clicked, this, &MainWindow::toggleObstacleMode);

    addMode = true;
    targetMode = false; // Новый режим для задания целевой точки
    obstacleMode = false;
    obstacleStartPoint = QPointF();
    updateModeButtonText();
}

void MainWindow::toggleMode() {
    if (!targetMode) { // Переключаем обычные режимы только если не в режиме задания цели
        addMode = !addMode;
        updateModeButtonText();
    }
}

void MainWindow::toggleTargetMode() {
    if (!obstacleMode) { // Можно включить режим цели только если не создаем препятствие
        targetMode = !targetMode;
        updateTargetButtonText();
        updateModeButtonText();

        if (targetMode) {
            view->setCursor(Qt::CrossCursor);
        } else {
            view->setCursor(Qt::ArrowCursor);
        }
    }
}

void MainWindow::toggleObstacleMode() {
    if (!targetMode) { // Можно включить режим препятствий только если не задаем цель
        obstacleMode = !obstacleMode;
        updateObstacleButtonText();
        updateModeButtonText();

        if (obstacleMode) {
            view->setCursor(Qt::CrossCursor);
            obstacleStartPoint = QPointF(); // Сбрасываем точку начала
        } else {
            view->setCursor(Qt::ArrowCursor);
            // Если отменяем создание препятствия в процессе, удаляем временный прямоугольник
            if (tempObstacleRect) {
                scene->removeItem(tempObstacleRect);
                delete tempObstacleRect;
                tempObstacleRect = nullptr;
            }
        }
    }
}
void MainWindow::updateModeButtonText() {
    if (targetMode) {
        toggleModeButton->setText("Режим: Задание цели");
    } else if (obstacleMode) {
        toggleModeButton->setText("Режим: Создание препятствий");
    } else {
        toggleModeButton->setText(addMode ? "Режим: Добавление" : "Режим: Замена");
    }
}

void MainWindow::updateTargetButtonText() {
    setTargetButton->setText(targetMode ? "Отменить задание цели" : "Задать конечную точку");
}
void MainWindow::updateObstacleButtonText() {
    createObstacleButton->setText(obstacleMode ? "Отменить создание" : "Создать препятствие");
}

void MainWindow::clearAllRays() {
    for (auto& raySet : allRaySets) {
        deleteGraphicsLines(scene, raySet.redLines);
        deleteGraphicsLines(scene, raySet.allBlueRays);
        deleteGraphicsLines(scene, raySet.bestBlueRay);
    }
    allRaySets.clear();
    currentRaySetIndex = -1;
}

void MainWindow::toggleRedLinesVisibility() {
    redLinesVisible = !redLinesVisible;

    for (auto& raySet : allRaySets) {
        for (QGraphicsLineItem *line : raySet.redLines) {
            line->setVisible(redLinesVisible);
        }
        for (QGraphicsLineItem *line : raySet.allBlueRays) {
            line->setVisible(redLinesVisible);
        }
    }

    toggleRedLinesButton->setText(redLinesVisible ? "Скрыть красные лучи" : "Показать красные лучи");
}

void MainWindow::onSceneClicked(const QPointF &point, Qt::MouseButton button) {
    if (button == Qt::LeftButton) {
        if (targetMode) {
            // Режим задания конечной точки
            setTargetPoint(point);
            targetMode = false;
            updateTargetButtonText();
            updateModeButtonText();
            view->setCursor(Qt::ArrowCursor);
            return;
        }

        if (obstacleMode) {
            // Режим создания препятствий
            if (obstacleStartPoint.isNull()) {
                // Первый клик - задаем начальную точку
                obstacleStartPoint = point;

                // Создаем временный прямоугольник размером 1x1 пиксель
                tempObstacleRect = new QGraphicsRectItem(0, 0, 1, 1);
                tempObstacleRect->setPos(point);
                tempObstacleRect->setBrush(QBrush(Qt::gray, Qt::DiagCrossPattern)); // Паттерн для отличия от готовых препятствий
                tempObstacleRect->setPen(QPen(Qt::darkGray, 2, Qt::DashLine));
                scene->addItem(tempObstacleRect);
            } else {
                // Второй клик - создаем препятствие
                createObstacle(obstacleStartPoint, point);

                // Удаляем временный прямоугольник
                if (tempObstacleRect) {
                    scene->removeItem(tempObstacleRect);
                    delete tempObstacleRect;
                    tempObstacleRect = nullptr;
                }

                // Выходим из режима создания препятствий
                obstacleMode = false;
                obstacleStartPoint = QPointF();
                updateObstacleButtonText();
                updateModeButtonText();
                view->setCursor(Qt::ArrowCursor);
            }
            return;
        }

        // Обычный режим трассировки
        if (!addMode) {
            clearAllRays();
        }

        RaySet newRaySet;
        newRaySet.startPoint = point;
        newRaySet.bestBlueLength = std::numeric_limits<qreal>::max();

        int maxBounces = 15;
        double angleStep = 5.0;

        for (double angle = 0.0; angle < 360.0; angle += angleStep) {
            launchRayTracing(point, angle, maxBounces, newRaySet);
        }

        allRaySets.append(newRaySet);
        currentRaySetIndex = allRaySets.size() - 1;

        // Скрываем красные лучи по умолчанию
        for (QGraphicsLineItem *line : newRaySet.redLines) {
            line->setVisible(false);
        }
        for (QGraphicsLineItem *line : newRaySet.allBlueRays) {
            line->setVisible(false);
        }

        redLinesVisible = false;
        toggleRedLinesButton->setText("Показать красные лучи");
    }
}

void MainWindow::onMouseMoved(const QPointF &point) {
    if (obstacleMode && !obstacleStartPoint.isNull() && tempObstacleRect) {
        // Обновляем размер временного прямоугольника
        double x = qMin(obstacleStartPoint.x(), point.x());
        double y = qMin(obstacleStartPoint.y(), point.y());
        double width = qAbs(point.x() - obstacleStartPoint.x());
        double height = qAbs(point.y() - obstacleStartPoint.y());

        tempObstacleRect->setRect(0, 0, width, height);
        tempObstacleRect->setPos(x, y);
    }
}

void MainWindow::createObstacle(const QPointF &startPoint, const QPointF &endPoint) {
    // Вычисляем параметры прямоугольника
    double x = qMin(startPoint.x(), endPoint.x());
    double y = qMin(startPoint.y(), endPoint.y());
    double width = qAbs(endPoint.x() - startPoint.x());
    double height = qAbs(endPoint.y() - startPoint.y());

    // Минимальный размер препятствия
    const double minSize = 10.0;
    if (width < minSize) width = minSize;
    if (height < minSize) height = minSize;

    // Создаем новое препятствие
    QGraphicsRectItem* obstacle = addObstacle(scene, x, y, width, height);

    // Добавляем в список созданных препятствий для возможности удаления
    createdObstacles.append(obstacle);

    qDebug() << "Создано препятствие:" << x << y << width << height;
}

void MainWindow::setTargetPoint(const QPointF &point) {
    // Удаляем старую целевую точку
    if (targetCircle) {
        scene->removeItem(targetCircle);
        delete targetCircle;
    }

    // Создаем новую целевую точку
    const double radius = 5.0;
    targetCircle = new QGraphicsEllipseItem(-radius, -radius, 2*radius, 2*radius);
    targetCircle->setBrush(Qt::green);
    targetCircle->setPen(Qt::NoPen);
    targetCircle->setPos(point);
    scene->addItem(targetCircle);

    // Очищаем все существующие трассы, так как цель изменилась
    clearAllRays();
}

bool intersectRayCircle(const QPointF &rayStart, const QVector2D &direction,
                        const QPointF &circleCenter, double radius, double &t) {
    QVector2D m = QVector2D(rayStart - circleCenter);
    double b = QVector2D::dotProduct(m, direction);
    double c = QVector2D::dotProduct(m, m) - radius*radius;

    double discriminant = b*b - c;
    if (discriminant < 0) {
        return false;
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

bool MainWindow::intersectRayWithExistingTraces(const QPointF &rayStart, const QVector2D &direction,
                                                double rayLength, QPointF &intersectionPoint,
                                                QVector2D &reflectionNormal, double &distance) {
    QLineF ray(rayStart, rayStart + direction.toPointF() * rayLength);
    double minDistance = std::numeric_limits<double>::max();
    bool foundIntersection = false;

    const double epsilon = 1e-6;

    for (const auto& raySet : allRaySets) {
        for (const auto* traceLine : raySet.bestBlueRay) {
            QLineF existingSegment = traceLine->line();
            QPointF intersection;

            if (ray.intersects(existingSegment, &intersection) == QLineF::BoundedIntersection) {
                double dist = QLineF(rayStart, intersection).length();

                if (dist > epsilon && dist < minDistance) {
                    minDistance = dist;
                    intersectionPoint = intersection;
                    foundIntersection = true;

                    QVector2D segmentDir = QVector2D(existingSegment.p2() - existingSegment.p1()).normalized();
                    reflectionNormal = QVector2D(-segmentDir.y(), segmentDir.x());

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

    QPointF circleCenter = targetCircle->sceneBoundingRect().center();
    double radius = targetCircle->rect().width() / 2;

    while (bounces < maxBounces) {
        QLineF ray(currentPos, currentPos + direction.toPointF() * rayLength);

        QGraphicsItem *closestItem = nullptr;
        QPointF closestPoint;
        QVector2D normal;
        qreal minDist = 1e9;

        // Проверяем пересечение с препятствиями
        for (QGraphicsItem *item : scene->items()) {
            if (item == targetCircle || qgraphicsitem_cast<QGraphicsLineItem*>(item)) continue;

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

        double circleDistance = circleHit ? tCircle : std::numeric_limits<double>::max();
        double obstacleDistance = closestItem ? minDist : std::numeric_limits<double>::max();
        double traceDistanceValue = hitExistingTrace ? traceDistance : std::numeric_limits<double>::max();

        double minDistance = std::min({circleDistance, obstacleDistance, traceDistanceValue});

        if (circleHit && circleDistance == minDistance) {
            QPointF intersectPoint = currentPos + direction.toPointF() * tCircle;
            segments.append(QLineF(currentPos, intersectPoint));
            hitCircle = true;
            break;
        }
        else if (hitExistingTrace && traceDistanceValue == minDistance) {
            segments.append(QLineF(currentPos, traceIntersection));
            direction = direction - 2 * QVector2D::dotProduct(direction, traceNormal) * traceNormal;
            direction.normalize();
            currentPos = traceIntersection + direction.toPointF() * epsilon;
            bounces++;
        }
        else if (closestItem && obstacleDistance == minDistance) {
            segments.append(QLineF(currentPos, closestPoint));
            direction = direction - 2 * QVector2D::dotProduct(direction, normal) * normal;
            direction.normalize();
            currentPos = closestPoint + direction.toPointF() * epsilon;
            bounces++;
        }
        else {
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

        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 2));
            line->setVisible(false);
            raySet.allBlueRays.append(line);
        }

        if (totalLength < raySet.bestBlueLength) {
            for (QGraphicsLineItem *line : raySet.bestBlueRay) {
                scene->removeItem(line);
                delete line;
            }
            raySet.bestBlueRay.clear();

            for (const QLineF &seg : segments) {
                QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::blue, 3));
                line->setVisible(true);
                raySet.bestBlueRay.append(line);
            }

            raySet.bestBlueLength = totalLength;
        }
    } else {
        for (const QLineF &seg : segments) {
            QGraphicsLineItem *line = scene->addLine(seg, QPen(Qt::red, 2));
            line->setVisible(false);
            raySet.redLines.append(line);
        }
    }
}

bool MainWindow::isDirectPathClear(const QPointF& start, const QPointF& end) {
    QLineF testPath(start, end);
    const double epsilon = 0.01;

    // Проверяем пересечение с препятствиями
    for (QGraphicsItem *item : scene->items()) {
        if (item == targetCircle || qgraphicsitem_cast<QGraphicsLineItem*>(item)) continue;

        if (auto rectItem = qgraphicsitem_cast<QGraphicsRectItem*>(item)) {
            QRectF rect = rectItem->mapRectToScene(rectItem->rect());

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

    // Проверяем пересечение с существующими трассами
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
    int maxIterations = 100;
    int iteration = 0;
    double minSegmentLength = 1;

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

            // Длины сегментов
            double beforeLength = segments[vertex.segmentBefore].length();
            double afterLength = segments[vertex.segmentAfter].length();

            if (beforeLength < minSegmentLength * 2 || afterLength < minSegmentLength * 2) {
                continue;
            }

            // Направления сегментов
            QVector2D dirBefore = QVector2D(vertex.point - segmentBeforeStart).normalized();
            QVector2D dirAfter = QVector2D(segmentAfterEnd - vertex.point).normalized();

            bool foundBetterPath = false;
            int maxSteps = 30;
            QLineF bestDirectConnection;
            QLineF bestModifiedBefore, bestModifiedAfter;
            double bestDistBefore = 0, bestDistAfter = 0;

            // Пробуем разные расстояния от концов сегментов к вершине отскока
            for (int stepBefore = 0; stepBefore <= maxSteps && !foundBetterPath; ++stepBefore) {
                double distFromStartBefore = beforeLength * stepBefore / maxSteps;

                if (distFromStartBefore < minSegmentLength && stepBefore != 0) continue;
                if (distFromStartBefore > beforeLength - minSegmentLength) continue;


                QPointF pointOnBefore = segmentBeforeStart + dirBefore.toPointF() * distFromStartBefore;

                for (int stepAfter = 0; stepAfter <= maxSteps; ++stepAfter) {
                    double distFromVertexAfter = afterLength * stepAfter / maxSteps;

                    if (distFromVertexAfter < minSegmentLength || distFromVertexAfter > afterLength - minSegmentLength)
                        continue;

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
                bool localMergeMade = true;
                while (localMergeMade) {
                    localMergeMade = false;

                    for (int i = 0; i < segments.size() - 2; ++i) {
                        int maxMergeCount = 3;

                        for (int count = 3; count <= maxMergeCount && i + count <= segments.size(); ++count) {
                            QPointF start = segments[i].p1();
                            QPointF end = segments[i + count - 1].p2();

                            QLineF directLine(start, end);
                            double directLength = directLine.length();

                            double originalLength = 0.0;
                            for (int k = 0; k < count; ++k) {
                                originalLength += segments[i + k].length();
                            }

                            if (directLength + 0.01 < originalLength && isDirectPathClear(start, end)) {
                                segments[i] = directLine;
                                for (int k = 0; k < count - 1; ++k) {
                                    segments.removeAt(i + 1);
                                }

                                qDebug() << "Внутри итерации объединено" << count << "сегментов в прямую от" << start << "до" << end;

                                localMergeMade = true;
                                break;
                            }
                        }

                        if (localMergeMade) break;
                    }
                }
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
        return;
    }

    for (int i = 0; i < allRaySets.size(); ++i) {
        RaySet& raySet = allRaySets[i];

        if (raySet.bestBlueRay.isEmpty()) {
            continue;
        }

        QList<QLineF> optimizedSegments = optimizeTraceSegments(raySet.bestBlueRay);

        if (optimizedSegments.isEmpty()) {
            continue;
        }

        // Удаляем старую трассу
        for (auto* item : raySet.bestBlueRay) {
            scene->removeItem(item);
            delete item;
        }
        raySet.bestBlueRay.clear();

        // Создаем новую оптимизированную трассу
        for (const QLineF& segment : optimizedSegments) {
            QGraphicsLineItem* line = scene->addLine(segment, QPen(Qt::blue, 3));
            line->setVisible(true);
            raySet.bestBlueRay.append(line);
        }

        // Пересчитываем длину
        raySet.bestBlueLength = 0;
        for (const QLineF& segment : optimizedSegments) {
            raySet.bestBlueLength += segment.length();
        }
    }
}

void MainWindow::addTestComponents(const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Cannot open file:" << filename;
        return;
    }

    // Добавим рамку как раньше
    QGraphicsRectItem *boardBorder = new QGraphicsRectItem(0, 0, 700, 500);
    boardBorder->setPos(50, 50);
    boardBorder->setPen(QPen(Qt::black, 3));
    boardBorder->setBrush(Qt::NoBrush);
    scene->addItem(boardBorder);

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.trimmed().isEmpty()) continue; // пропустить пустые строки
        QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() != 4) continue; // некорректная строка

        bool okX, okY, okW, okH;
        double x = parts[0].toDouble(&okX);
        double y = parts[1].toDouble(&okY);
        double w = parts[2].toDouble(&okW);
        double h = parts[3].toDouble(&okH);

        if (okX && okY && okW && okH) {
            addObstacle(scene, x, y, w, h);
        }
    }

    const double radius = 5.0;
    targetCircle = new QGraphicsEllipseItem(-radius, -radius, 2*radius, 2*radius);
    targetCircle->setBrush(Qt::green);
    targetCircle->setPen(Qt::NoPen);
    targetCircle->setPos(715, 100);
    scene->addItem(targetCircle);

    file.close();
}
