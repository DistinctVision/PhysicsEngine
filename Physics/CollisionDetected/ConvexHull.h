#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <QVector3D>
#include <QMatrix4x4>
#include "QScrollEngine/QBoundingBox.h"

namespace CD {

class ConvexHull
{
public:
    ConvexHull();

    void setCountVerties(std::size_t count);
    void setVertex(std::size_t index, const QVector3D& vertex);
    QVector3D& vertex(std::size_t index);
    const QVector3D& vertex(std::size_t index) const;
    float supportValue(const QVector3D& dir) const;
    float supportValue(QVector3D& supportVertex, const QVector3D& dir) const;
    void setTransformedVertex(std::size_t index, const QVector3D& vertex);
    QVector3D& transformedVertex(std::size_t index);
    const QVector3D& transformedVertex(std::size_t index) const;
    float transformedSupprtValue(const QVector3D& dir) const;
    float transformedSupprtValue(QVector3D& transformedSupprtVertex, const QVector3D& dir) const;

    QMatrix4x4 transformMatrix() const;
    void transformVertices(const QMatrix4x4& transform);

    const QScrollEngine::QBoundingBox& boundingBox() const;
    void computeBoundingBox();

private:
    std::vector<QVector3D> m_vertices;
    std::vector<QVector3D> m_transformedVertices;
    QMatrix4x4 m_transformMatrix;
    QScrollEngine::QBoundingBox m_boundingBox;

};

}

#endif // CONVEXHULL_H
