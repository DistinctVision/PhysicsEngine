#ifndef SCENE_H
#define SCENE_H

#include <functional>

#include "QScrollEngine/QScrollEngine.h"
#include "QScrollEngine/Tools/QPlanarShadows.h"
#include <QSet>
#include <QTouchEvent>
#include <QPair>
#include <QVector>
#include <QColor>

#include "Physics/Physics.h"

class Scene :
        public QScrollEngine::QScene//наследуем от QScene, это один вариантов использования сцены
{
    Q_OBJECT//раз нужны сигналы и слоты, то нужно исполтьзовать эту макро функцию
public:
    Scene(QScrollEngine::QScrollEngineWidget* widget);
    ~Scene();

private slots:
    void slotKeyPress(int keyCode);//слот нажатия на клавишу
    void slotKeyRelease(int keyCode);//отпускания клавиши
    void slotTouchPress(const QTouchEvent::TouchPoint& touchPoint);//нажатие мышки
    void slotTouchMove(const QTouchEvent::TouchPoint& touchPoint);//движения нажатой мышки
    void slotTouchRelease(const QTouchEvent::TouchPoint& touchPoint);//нажатие мышки
    void slotBeginDrawing();//этот слот будет соответствовать началу рисованию сцены
    void slotEndDrawing();//а этот концу рисования

protected:
    bool m_showDebug = false;

    QScrollEngine::QPlanarShadows m_planarShadows;//При помощи этого объекта создадим плоские тени.
    QSet<int> m_keys;//какие клвиши в данный момент нажаты.
    bool m_mousePressed;//нажата ли левая кнопка мышки
    //Предыдущее и текущее положение мышки
    QPointF m_prevMousePos;
    QPointF m_currentMousePos;

    PE::PhysicsWorld m_physicsWorld;
    QVector<QPair<PE::Body*, QScrollEngine::QEntity*>> m_bodies;

    QScrollEngine::QEntity* m_originalDebugContact;
    std::vector<QScrollEngine::QEntity*> m_debugContacts;

    std::map<std::shared_ptr<const PE::CollisionGroup>, QColor> m_colors;

    QScrollEngine::QEntity* m_bulletEntity;

    void _updatePhysic();
    QQuaternion _toQuaternion(const PE::RotationMatrix& R) const;
    void _initOriginalDebugContact();
    void _updateDebugContacts();
    QScrollEngine::QEntity* _createHumanEntity();
    PE::Body* _createHumanBody();
    void _shot();
    void _setColor(QScrollEngine::QEntity* entity, PE::Body* body);
    void _setColor(QScrollEngine::QEntity* entity, const QColor& c);
};

#endif // SCENE_H
