#include "ray.h"
#include <QGraphicsRectItem>
#include <QtMath>

Ray::Ray(QPointF startPoint, double angle, QGraphicsItem* parent)
    : QGraphicsLineItem(parent), startPoint(startPoint), angle(angle) {
    setPen(QPen(Qt::red, 1, Qt::SolidLine));
}

QPointF Ray::findIntersection(const QLineF& ray, const QGraphicsItem* obstacle) {
    if (const QGraphicsRectItem* rect = dynamic_cast<const QGraphicsRectItem*>(obstacle)) {
        QPolygonF rectPolygon = rect->rect().translated(rect->pos());
        QPointF intersectionPoint;
        for (int i = 0; i < rectPolygon.size(); ++i) {
            QLineF edge(rectPolygon[i], rectPolygon[(i + 1) % rectPolygon.size()]);
            if (ray.intersects(edge, &intersectionPoint) == QLineF::BoundedIntersection) {
                return intersectionPoint;
            }
        }
    }
    return QPointF(); // Нет пересечения
}
void Ray::calculatePath(const QList<QGraphicsItem*>& obstacles, int maxReflections) {
    pathSegments.clear();
    QPointF currentPos = startPoint;
    double currentAngle = angle;
    int reflections = 0;

    while (reflections <= maxReflections) {
        // Создаем луч из текущей позиции под текущим углом
        QLineF rayLine(currentPos,
                       currentPos + QPointF(1000 * qCos(qDegreesToRadians(currentAngle)),
                                            1000 * qSin(qDegreesToRadians(currentAngle))));

        // Ищем ближайшее пересечение
        QPointF nearestIntersection;
        QGraphicsItem* collidedItem = nullptr;
        for (QGraphicsItem* item : obstacles) {
            QPointF intersection = findIntersection(rayLine, item);
            if (!intersection.isNull()) {
                if (nearestIntersection.isNull() ||
                    QLineF(currentPos, intersection).length() < QLineF(currentPos, nearestIntersection).length()) {
                    nearestIntersection = intersection;
                    collidedItem = item;
                }
            }
        }

        // Если нет пересечений — рисуем луч до границы сцены
        if (nearestIntersection.isNull()) {
            pathSegments.append(QLineF(currentPos, rayLine.p2()));
            break;
        }

        // Добавляем отрезок до точки столкновения
        pathSegments.append(QLineF(currentPos, nearestIntersection));
        currentPos = nearestIntersection;

        // Если достигли максимума отражений — выходим
        if (reflections++ >= maxReflections) break;

        // Рассчитываем отраженный угол (угол падения = углу отражения)
        QLineF normalLine; // Нормаль к поверхности в точке столкновения
        // (Здесь нужно определить нормаль для collidedItem, упрощенный вариант ниже)
        currentAngle = -currentAngle; // Простое отражение (для вертикальных/горизонтальных стен)
    }

    // Обновляем графическое представление
    if (!pathSegments.isEmpty()) {
        setLine(pathSegments.first());
    }
}
