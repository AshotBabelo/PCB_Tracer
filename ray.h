#include <QGraphicsLineItem>
#include <QVector>

class Ray : public QGraphicsLineItem {
public:
    Ray(QPointF startPoint, double angle, QGraphicsItem* parent = nullptr);
    void calculatePath(const QList<QGraphicsItem*>& obstacles, int maxReflections);
    const QVector<QLineF>& getPathSegments() const;

private:
    QPointF startPoint;
    double angle; // в градусах
    QVector<QLineF> pathSegments; // Сохраняет все отрезки пути луча

    QPointF findIntersection(const QLineF& ray, const QGraphicsItem* obstacle);
    void reflect(const QLineF& segment, QPointF collisionPoint);
};
