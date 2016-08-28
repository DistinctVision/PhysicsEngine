#include "ConvexHull.h"
#include <cassert>

namespace CD {

ConvexHull::ConvexHull()
{
    m_vertices.resize(1);
    m_transformedVertices.resize(m_vertices.size());
}

void ConvexHull::setCountVerties(std::size_t count)
{
    assert(count > 0);
    m_vertices.resize(count);
    m_transformedVertices.resize(count);
}

void ConvexHull::setVertex(std::size_t index, const QVector3D& vertex)
{
    m_vertices[index] = vertex;
}

QVector3D& ConvexHull::vertex(std::size_t index)
{
    return m_vertices[index];
}

const QVector3D& ConvexHull::vertex(std::size_t index) const
{
    return m_vertices[index];
}

float ConvexHull::supportValue(const QVector3D& dir) const
{
    float sV = QVector3D::dotProduct(m_vertices[0], dir);
    for (std::size_t i=1; i<m_vertices.size(); ++i) {
        float v = QVector3D::dotProduct(m_vertices[i], dir);
        if (v > sV)
            sV = v;
    }
    return sV;
}

float ConvexHull::supportValue(QVector3D& supportVertex, const QVector3D& dir) const
{
    float sV = QVector3D::dotProduct(m_vertices[0], dir);
    supportVertex = m_vertices[0];
    for (std::size_t i=1; i<m_vertices.size(); ++i) {
        float v = QVector3D::dotProduct(m_vertices[i], dir);
        if (v > sV) {
            sV = v;
            supportVertex = m_vertices[i];
        }
    }
    return sV;
}

void ConvexHull::transformVertices(const QMatrix4x4& transform)
{
    m_transformMatrix = transform;
    for (std::size_t i=0; i<m_vertices.size(); ++i)
        m_transformedVertices[i] = m_transformMatrix * m_vertices[i];
}

void ConvexHull::setTransformedVertex(std::size_t index, const QVector3D& vertex)
{
    m_vertices[index] = vertex;
}

QVector3D& ConvexHull::transformedVertex(std::size_t index)
{
    return m_transformedVertices[index];
}

const QVector3D& ConvexHull::transformedVertex(std::size_t index) const
{
    return m_transformedVertices[index];
}

QMatrix4x4 ConvexHull::transformMatrix() const
{
    return m_transformMatrix;
}

float ConvexHull::transformedSupprtValue(const QVector3D& dir) const
{
    float sV = QVector3D::dotProduct(m_transformedVertices[0], dir);
    for (std::size_t i=1; i<m_transformedVertices.size(); ++i) {
        float v = QVector3D::dotProduct(m_transformedVertices[i], dir);
        if (v > sV)
            sV = v;
    }
    return sV;
}

float ConvexHull::transformedSupprtValue(QVector3D& transformedSupportVertex, const QVector3D& dir) const
{
    float sV = QVector3D::dotProduct(m_transformedVertices[0], dir);
    for (std::size_t i=1; i<m_transformedVertices.size(); ++i) {
        float v = QVector3D::dotProduct(m_transformedVertices[i], dir);
        if (v > sV) {
            sV = v;
            transformedSupportVertex = m_transformedVertices[i];
        }
    }
    return sV;
}

const QScrollEngine::QBoundingBox& ConvexHull::boundingBox() const
{
    return m_boundingBox;
}

void ConvexHull::computeBoundingBox()
{
    m_boundingBox.toPoint(m_vertices[0]);
    for (std::size_t i=1; i<m_vertices.size(); ++i)
        m_boundingBox.addPoint(m_vertices[i]);
}

}
