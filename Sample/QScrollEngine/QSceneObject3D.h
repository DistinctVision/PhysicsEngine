#ifndef QSCENEOBJECT3D_H
#define QSCENEOBJECT3D_H

#include <vector>

#include <QObject>
#include <QVector3D>
#include <QMatrix4x4>

namespace QScrollEngine {

class QScene;
class QScrollEngineContext;
class QEntity;

class QSceneObject3D:
        public QObject
{
    Q_OBJECT

public:
    typedef struct MatrixOfObject
    {
        QMatrix4x4 world;
        QMatrix4x4 worldViewProj;
    } MatrixOfObject;

public:
    QSceneObject3D() { m_transformHasChanged = true; }
    ~QSceneObject3D() { emit onDelete(this); }
    QEntity* parentEntity() const { return m_parentEntity; }
    QMatrix4x4 matrixWorld() const { return m_matrix.world; }
    QMatrix4x4 matrixWorldViewProj() const { return m_matrix.worldViewProj; }
    void updateMatrxWorldViewProj(const QMatrix4x4& matrixViewProj) { m_matrix.worldViewProj =
                matrixViewProj * m_matrix.world; }

    QScene* scene() const { return m_scene; }
    QVector3D position() const { return m_position; }
    QVector3D globalPosition() const { return m_globalPosition; }
    bool transformHasChanged() const { return m_transformHasChanged; }
    bool getGlobalParameters(QVector3D& globalScale, QMatrix3x3& globalOrieantion) const;
    bool getGlobalParameters(QVector3D& globalScale, QQuaternion& globalOrieantion) const;
    QVector3D fromLocalToGlobal(const QVector3D& localPoint) const;
    QScrollEngineContext* parentContext();

signals:
    void onDelete(QSceneObject3D* object);
    void onSceneUpdate();

public slots:
    void setChangedTransform() { m_transformHasChanged = true; }

protected:
    friend class QScrollEngineContext;
    friend class QScene;

    QEntity* m_parentEntity;
    MatrixOfObject m_matrix;
    QScene* m_scene;
    QVector3D m_position;
    QVector3D m_globalPosition;
    bool m_transformHasChanged;
};

}
#endif
