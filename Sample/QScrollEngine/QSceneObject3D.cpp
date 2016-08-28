#include "QScrollEngine/QSceneObject3D.h"
#include "QScrollEngine/QScrollEngineContext.h"
#include "QScrollEngine/QScene.h"
#include "QScrollEngine/QEntity.h"
#include <qmath.h>

namespace QScrollEngine {

bool QSceneObject3D::getGlobalParameters(QVector3D& globalScale, QMatrix3x3& globalOrieantion) const
{
    globalScale.setX(QVector3D(m_matrix.world(0, 0), m_matrix.world(1, 0), m_matrix.world(2, 0)).length());
    globalScale.setY(QVector3D(m_matrix.world(0, 1), m_matrix.world(1, 1), m_matrix.world(2, 1)).length());
    globalScale.setZ(QVector3D(m_matrix.world(0, 2), m_matrix.world(1, 2), m_matrix.world(2, 2)).length());
    if (fabsf(globalScale.x()) > QOtherMathFunctions::EPS) {
        globalOrieantion(0, 0) = m_matrix.world(0, 0) / globalScale.x();
        globalOrieantion(1, 0) = m_matrix.world(1, 0) / globalScale.x();
        globalOrieantion(2, 0) = m_matrix.world(2, 0) / globalScale.x();
    } else
        return false;
    if (fabsf(globalScale.y()) > QOtherMathFunctions::EPS) {
        globalOrieantion(0, 1) = m_matrix.world(0, 1) / globalScale.y();
        globalOrieantion(1, 1) = m_matrix.world(1, 1) / globalScale.y();
        globalOrieantion(2, 1) = m_matrix.world(2, 1) / globalScale.y();
    } else
        return false;
    if (fabsf(globalScale.z()) > QOtherMathFunctions::EPS) {
        globalOrieantion(0, 2) = m_matrix.world(0, 2) / globalScale.z();
        globalOrieantion(1, 2) = m_matrix.world(1, 2) / globalScale.z();
        globalOrieantion(2, 2) = m_matrix.world(2, 2) / globalScale.z();
    } else
        return false;
    return true;
}

bool QSceneObject3D::getGlobalParameters(QVector3D& globalScale, QQuaternion& globalOrieantion) const
{
    QMatrix3x3 matrixRotation;
    if (!getGlobalParameters(globalScale, matrixRotation))
        return false;
    QOtherMathFunctions::matrixToQuaternion(matrixRotation, globalOrieantion);
    return true;
}

QVector3D QSceneObject3D::fromLocalToGlobal(const QVector3D& localPoint) const
{
    return QOtherMathFunctions::transform(m_matrix.world, localPoint);
}

QScrollEngineContext* QSceneObject3D::parentContext()
{
    return (m_scene == nullptr) ? nullptr : m_scene->parentContext();
}

}
