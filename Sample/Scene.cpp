#include "Scene.h"
#include "QScrollEngine/QScrollEngineWidget.h"
#include "QScrollEngine/QMesh.h"
#include <cmath>
#include "Physics/Physics.h"

Scene::Scene(QScrollEngine::QScrollEngineWidget* widget):
    m_physicsWorld(PE::Vector3(0.0f, 0.0f, -9.81f)),
    QScene(widget, 0) // обязательно нужно указать QScene какому объекту QScrollEngineContext она принадлежит.
    //А QScrollEngineWidget наследуется от QScrollEngineContext.
    //Параметр order равен 0, а значит ее порядок рисования будет стандартной.
{
    using namespace QScrollEngine;
    using namespace PE;

    //Соединяем нужные сигналы и слоты.
    connect(this, SIGNAL(beginDrawing()), this, SLOT(slotBeginDrawing()));
    connect(this, SIGNAL(endDrawing()), this, SLOT(slotEndDrawing()));
    connect(widget, SIGNAL(keyPress(int)), this, SLOT(slotKeyPress(int)));
    connect(widget, SIGNAL(keyRelease(int)), this, SLOT(slotKeyRelease(int)));
    //В виджете оди и тотже сигнал для нажатия мыши и нажатия тачскина.
    //Тачскрин в коде ниже мы не учитываем, и в таких случаях поддержку тачскрина можно отключить
    //при помощи макро - "#define SUPPORT_TOUCHSCREEN 0". Но в данном случае этого делать не будем.
    connect(widget, SIGNAL(touchPressed(QTouchEvent::TouchPoint)), this, SLOT(slotTouchPress(QTouchEvent::TouchPoint)));
    connect(widget, SIGNAL(touchMoved(QTouchEvent::TouchPoint)), this, SLOT(slotTouchMove(QTouchEvent::TouchPoint)));
    connect(widget, SIGNAL(touchReleased(QTouchEvent::TouchPoint)), this, SLOT(slotTouchRelease(QTouchEvent::TouchPoint)));

    widget->setPostEffectUsed(true);//Используем пост-эффект Bloom.
    widget->backgroundColor = QColor(0, 150, 255);//Пусть фон будет голубым.
    setAmbientColor(100, 100, 100);
    widget->camera->setPosition(QVector3D(0.0f, 0.0f, 7.0f));
    widget->camera->setOrientation(QQuaternion::fromAxisAndAngle(1.0f, 0.0f, 0.0f, 90.0f));

    _initOriginalDebugContact();

    //Создадим источник освещения
    QLight* light = new QLight(this);
    light->setRadius(5000.0f);
    light->setPosition(-300.0f, 100.0f, 500.0f);

    m_planarShadows.setScene(this);
    m_planarShadows.setLight(light);
    m_planarShadows.setAlpha(true);
    m_planarShadows.setColorShadows(QColor(0, 0, 0, 100));

    QMesh* cubeMesh = new QMesh(this);
    QGLPrimitiv::createCube(cubeMesh);

    QEntity* groundEntity = new QEntity(this);
    groundEntity->setName("Ground");
    groundEntity->addPart(cubeMesh, QShPtr(new QSh_Light(nullptr, QColor(155, 55, 0))));
    groundEntity->setScale(500.0f, 500.0f, 1.0f);
    Body* groundBody = new Body(&m_physicsWorld);
    groundBody->setMass(0.0f);
    Hull* groundHull = new Hull();
    groundHull->setCountVertices(8);
    groundHull->setLocalVertex(0, Vector3(-500.0f, -500.0f, -1.0f));
    groundHull->setLocalVertex(1, Vector3(500.0f, -500.0f, -1.0f));
    groundHull->setLocalVertex(2, Vector3(500.0f, 500.0f, -1.0f));
    groundHull->setLocalVertex(3, Vector3(-500.0f, 500.0f, -1.0f));
    groundHull->setLocalVertex(4, Vector3(-500.0f, -500.0f, 1.0f));
    groundHull->setLocalVertex(5, Vector3(500.0f, -500.0f, 1.0f));
    groundHull->setLocalVertex(6, Vector3(500.0f, 500.0f, 1.0f));
    groundHull->setLocalVertex(7, Vector3(-500.0f, 500.0f, 1.0f));
    groundHull->formedHull();
    groundHull->setBody(groundBody);
    groundBody->updateShapes();

    m_bodies.push_back(qMakePair(groundBody, groundEntity));
    m_planarShadows.setPlanePos(QVector3D(0.0f, 0.0f, 1.0f));
    m_planarShadows.setPlaneDir(QVector3D(0.0f, 0.0f, 1.0f));

    m_bulletEntity = new QEntity(this);
    m_bulletEntity->setName("Bullet");
    m_bulletEntity->addPart(parentContext(), QGLPrimitiv::Primitives::Sphere, QShPtr(new QSh_Light));
    m_bulletEntity->setVisibled(false);

    int i, j;
    QEntity* sphereEntity = new QEntity(this);
    sphereEntity->setName("Sphere");
    sphereEntity->addPart(parentContext(), QGLPrimitiv::Primitives::Sphere, QShPtr(new QSh_Light));
    sphereEntity->setPosition(5.0f, 10.0f, 1.0f);
    QVector3D p = sphereEntity->position();
    Body* sphereBody = new Body(&m_physicsWorld);
    sphereBody->setPosition(Vector3(p.x(), p.y(), p.z()));
    (new Sphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f))->setBody(sphereBody);
    m_bodies.push_back(qMakePair(sphereBody, sphereEntity));
    m_planarShadows.addEntity(sphereEntity);
    for (i = 1; i < 10; ++i) {
        sphereEntity = sphereEntity->clone();
        sphereEntity->setPosition(sphereEntity->position() + QVector3D(0.0f, 0.0f, 1.0f));
        p = sphereEntity->position();
        Body* sphereBody = new Body(&m_physicsWorld);
        sphereBody->setPosition(Vector3(p.x(), p.y(), p.z()));
        (new Sphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f))->setBody(sphereBody);
        m_bodies.push_back(qMakePair(sphereBody, sphereEntity));
        m_planarShadows.addEntity(sphereEntity);
    }

    Hull* cubeHull = new Hull();
    cubeHull->setCountVertices(8);
    cubeHull->setLocalVertex(0, Vector3(-0.5f, -0.5f, -0.5f));
    cubeHull->setLocalVertex(1, Vector3(0.5f, -0.5f, -0.5f));
    cubeHull->setLocalVertex(2, Vector3(-0.5f, 0.5f, -0.5f));
    cubeHull->setLocalVertex(3, Vector3(0.5f, 0.5f, -0.5f));
    cubeHull->setLocalVertex(4, Vector3(-0.5f, -0.5f, 0.5f));
    cubeHull->setLocalVertex(5, Vector3(0.5f, -0.5f, 0.5f));
    cubeHull->setLocalVertex(6, Vector3(-0.5f, 0.5f, 0.5f));
    cubeHull->setLocalVertex(7, Vector3(0.5f, 0.5f, 0.5f));
    cubeHull->formedHull();
    for (i = 8; i >= 0; --i) {
        for (j = 0; j < i; ++j) {
            Body* cubeBody = new Body(&m_physicsWorld);
            cubeBody->setPosition(Vector3(i * (1.0f + 0.05f) - j * 0.5f - 8.0f,
                                          10.0f,
                                          j * (1.0f + 0.05f) + 6.0f));
            Hull* s = (Hull*)cubeHull->copy();
            s->setBody(cubeBody);
            cubeBody->updateShapes();
            cubeBody->updateBoundsTree();
            cubeBody->calculateLocalInertia();
            QEntity* entity = new QEntity(this);
            entity->addPart(cubeMesh, QShPtr(new QSh_Light));
            m_bodies.push_back(qMakePair(cubeBody, entity));
            m_planarShadows.addEntity(entity);
        }
    }
    delete cubeHull;

    QEntity* capsuleEntity = new QEntity(this);
    capsuleEntity->setName("Capsule");
    QEntity::Part* part = capsuleEntity->addPart(parentContext(), QGLPrimitiv::Primitives::Sphere, QShPtr(new QSh_Light));
    part->mesh()->moveVertices(0.0f, 0.0f, 0.5f);
    part->mesh()->applyChangesOfVertexPositions();
    part->mesh()->updateLocalBoundingBox();
    part = capsuleEntity->addPart(parentContext(), QGLPrimitiv::Primitives::Sphere, QShPtr(new QSh_Light));
    part->mesh()->moveVertices(0.0f, 0.0f, -0.5f);
    part->mesh()->applyChangesOfVertexPositions();
    part->mesh()->updateLocalBoundingBox();
    capsuleEntity->addPart(parentContext(), QGLPrimitiv::Primitives::Cylinder, QShPtr(new QSh_Light));
    capsuleEntity->setPosition(-7.0f, 11.0f, 1.0f);

    Body* capsuleBody = new Body(&m_physicsWorld);
    p = capsuleEntity->position();
    capsuleBody->setPosition(Vector3(p.x(), p.y(), p.z()));
    (new Capsule(1.0f, 0.5f))->setBody(capsuleBody);
    m_bodies.push_back(qMakePair(capsuleBody, capsuleEntity));
    m_planarShadows.addEntity(capsuleEntity);

    for (i = 1; i < 10; ++i) {
        capsuleEntity = capsuleEntity->clone();
        capsuleEntity->setPosition(capsuleEntity->position() + QVector3D(0.0f, 0.0f, 2.0f));

        Body* capsuleBody = new Body(&m_physicsWorld);
        p = capsuleEntity->position();
        capsuleBody->setPosition(Vector3(p.x(), p.y(), p.z()));
        (new Capsule(1.0f, 0.5f))->setBody(capsuleBody);

        m_bodies.push_back(qMakePair(capsuleBody, capsuleEntity));
        m_planarShadows.addEntity(capsuleEntity);
    }

    QEntity* humanEntity = _createHumanEntity();
    humanEntity->setName("Human");
    humanEntity->setPosition(5.0f, 8.0f, 5.0f);
    p = humanEntity->position();
    Body* humanBody = _createHumanBody();
    humanBody->setPosition(Vector3(p.x(), p.y(), p.z()));
    m_bodies.push_back(qMakePair(humanBody, humanEntity));
    m_planarShadows.addEntity(humanEntity);
    for (i = 1; i < 5; ++i) {
        humanEntity = humanEntity->clone();
        humanEntity->setPosition(humanEntity->position() + QVector3D(0.0f, 0.0f, 5.0f));
        p = humanEntity->position();
        Body* hBody = humanBody->copy();
        hBody->setPosition(Vector3(p.x(), p.y(), p.z()));
        m_bodies.push_back(qMakePair(hBody, humanEntity));
        m_planarShadows.addEntity(humanEntity);
    }
}

Scene::~Scene()
{
    //По идее нужно удалить все остальные созданные объекты, на так как они все привязаны к сценам,
    //то при удалении сцен удалятся и привязанные объекты
}

void Scene::slotKeyPress(int keyCode)
{
    m_keys.insert(keyCode);
}

void Scene::slotKeyRelease(int keyCode)
{
    m_keys.remove(keyCode);
}

void Scene::slotTouchPress(const QTouchEvent::TouchPoint& touchPoint)
{
    m_mousePressed = true;
    m_prevMousePos = m_currentMousePos = touchPoint.pos();
}

void Scene::slotTouchMove(const QTouchEvent::TouchPoint& touchPoint)
{
    m_mousePressed = true;
    m_currentMousePos = touchPoint.pos();
}

void Scene::slotTouchRelease(const QTouchEvent::TouchPoint& touchPoint)
{
    Q_UNUSED(touchPoint);
    m_mousePressed = false;
}

void Scene::slotBeginDrawing()
{
    using namespace QScrollEngine;
    //Слот вызывается перед отрисовкой всех объектов сцены.
    //Нужно понимать, что сцены не обязательно будут рисоваться последовательно.
    QScene::CameraInfo info = cameraInfo();//Информация о положении камеры в сцене
    QCamera3D* camera = parentContext()->camera;
    if (m_keys.contains(Qt::Key_Up) || m_keys.contains(Qt::Key_W))
        camera->setPosition(camera->position() - info.localZ * 0.2f);
    if (m_keys.contains(Qt::Key_Down) || m_keys.contains(Qt::Key_S))
        camera->setPosition(camera->position() + info.localZ * 0.2f);
    if (m_keys.contains(Qt::Key_Left) || m_keys.contains(Qt::Key_A))
        camera->setPosition(camera->position() - info.localX * 0.1f);
    if (m_keys.contains(Qt::Key_Right) || m_keys.contains(Qt::Key_D))
        camera->setPosition(camera->position() + info.localX * 0.1f);
    if (m_keys.contains(Qt::Key_Space)) {
        m_keys.remove(Qt::Key_Space);
        _shot();
    }
    if (m_mousePressed) {
        QPointF deltaMouse = m_currentMousePos - m_prevMousePos;
        camera->setOrientation(camera->orientation() *
                               QQuaternion::fromAxisAndAngle(0.0f, -1.0f, 0.0f, deltaMouse.x() * 0.01f) *
                               QQuaternion::fromAxisAndAngle(-1.0f, 0.0f, 0.0f, deltaMouse.y() * 0.01f));
    }
    _updatePhysic();
}

void Scene::slotEndDrawing()
{
    //Слот просто создан для примера и вызывается после отрисовки всех объектов сцены.
}

void Scene::_updatePhysic()
{
    using namespace PE;

    m_physicsWorld.update(1.0f / 60.0f);
    for (auto it = m_bodies.begin(); it != m_bodies.end(); ++it) {
        Vector3 pos = it->first->position();
        it->second->setPosition(pos.x, pos.y, pos.z);
        it->second->setOrientation(_toQuaternion(it->first->rotation()));
        if (it->first->isDynamic())
            _setColor(it->second, it->first);
    }
    if (m_showDebug)
        _updateDebugContacts();
}

QQuaternion Scene::_toQuaternion(const PE::RotationMatrix& R) const
{
    QMatrix3x3 m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m(j, i) = R[i][j];
    QQuaternion q;
    QScrollEngine::QOtherMathFunctions::matrixToQuaternion(m, q);
    q.normalize();
    return q;
}

void Scene::_initOriginalDebugContact()
{
    using namespace QScrollEngine;
    m_originalDebugContact = new QEntity(this);
    m_originalDebugContact->setVisibled(false);
    QMesh* mesh = new QMesh(parentContext());
    mesh->setCountVertices(4);
    mesh->setVertexPosition(0, QVector3D(0.0f, 0.0f, 0.0f));
    mesh->setVertexPosition(1, QVector3D(0.2f, 0.0f, 0.0f));
    mesh->setVertexPosition(2, QVector3D(0.0f, 0.05f, 0.0f));
    mesh->setVertexPosition(3, QVector3D(0.0f, 0.0f, 0.05f));
    mesh->setCountTriangles(4);
    mesh->setTriangle(0, 0, 1, 2);
    mesh->setTriangle(1, 0, 2, 1);
    mesh->setTriangle(2, 0, 1, 3);
    mesh->setTriangle(3, 0, 3, 1);
    mesh->applyChanges();
    mesh->updateLocalBoundingBox();
    m_originalDebugContact->addPart(mesh, QShPtr(new QSh_Color(QColor(255, 0, 0, 255))));
}

void Scene::_updateDebugContacts()
{
    using namespace PE;
    using namespace QScrollEngine;
    for (auto it = m_debugContacts.begin(); it != m_debugContacts.end(); ++it)
        delete *it;
    m_debugContacts.clear();
    QMatrix3x3 m;
    QQuaternion q;
    const ContactsContainer& contacts = m_physicsWorld.contactsContainer();
    for (std::size_t i = 0; i < contacts.countContactManifolds(); ++i) {
        const ContactTypes::ContactManifold& cm = contacts.contactManifold(i);
        Vector3 n, n1, n2;
        n = cm.normal;
        createNormal12(n, n1, n2);
        m(0, 0) = n.x;
        m(0, 1) = n.y;
        m(0, 2) = n.z;
        m(1, 0) = n1.x;
        m(1, 1) = n1.y;
        m(1, 2) = n1.z;
        m(2, 0) = n2.x;
        m(2, 1) = n2.y;
        m(2, 2) = n2.z;
        for (int j = 0; j < cm.countPoints; ++j) {
            QEntity* e = m_originalDebugContact->clone();
            e->setVisibled(true);
            QOtherMathFunctions::matrixToQuaternion(m, q);
            e->setOrientation(q);
            Vector3 p = cm.getPointA(j);
            e->setPosition(p.x, p.y, p.z);
            m_debugContacts.push_back(e);
        }
    }
}

QScrollEngine::QEntity* Scene::_createHumanEntity()
{
    using namespace PE;
    using namespace QScrollEngine;

    QEntity* entity = new QEntity(this);

    QMesh* mesh = new QMesh(parentContext());
    QGLPrimitiv::createCube(mesh);
    mesh->scaleVertices(1.0f, 0.5, 1.5f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light()));

    mesh = new QMesh(parentContext());
    QGLPrimitiv::createSphere(mesh);
    mesh->moveVertices(0.0f, 0.0f, 1.5f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light()));

    mesh = new QMesh(parentContext());
    QGLPrimitiv::createCylinder(mesh, 0.2f, 3.0f);
    QMatrix4x4 transform;
    transform.rotate(90.0f, 0.0f, 1.0f, 0.0f);
    mesh->transformVertices(transform);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light));
    mesh = new QMesh(parentContext());
    QGLPrimitiv::createSphere(mesh, 0.2f);
    mesh->moveVertices(-1.5f, 0.0f, 0.0f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light));
    mesh = new QMesh(parentContext());
    QGLPrimitiv::createSphere(mesh, 0.2f);
    mesh->moveVertices(1.5f, 0.0f, 0.0f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light));

    mesh = new QMesh(parentContext());
    QGLPrimitiv::createCube(mesh);
    mesh->scaleVertices(0.3f, 0.3f, 1.0f);
    mesh->moveVertices(-0.2f, 0.0f, -1.0f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light()));

    mesh = new QMesh(parentContext());
    QGLPrimitiv::createCube(mesh);
    mesh->scaleVertices(0.3f, 0.3f, 1.0f);
    mesh->moveVertices(0.2f, 0.0f, -1.0f);
    mesh->applyChangesOfVertexPositions();
    mesh->updateLocalBoundingBox();
    entity->addPart(mesh, QShPtr(new QSh_Light()));

    return entity;
}

PE::Body* Scene::_createHumanBody()
{
    using namespace PE;

    Body* body = new Body(&m_physicsWorld);

    Hull* corpus = Hull::creeateCube(Vector3(1.0f, 0.5, 1.5f) * 0.5f);
    corpus->setBody(body);

    Sphere* head = new Sphere(Vector3(0.0f, 0.0f, 1.5f), 0.5f);
    head->setBody(body);

    Capsule* hands = new Capsule(Vector3(-3.0f, 0.0f, 0.0f), (3.0f, 0.0f, 0.0f), 0.2f);
    hands->setBody(body);

    Hull* leftLeg = Hull::creeateCube();
    leftLeg->scaleLocalVertices(Vector3(0.3f, 0.3f, 1.0f) * 0.5f);
    leftLeg->moveLocalVertices(Vector3(-0.2f, 0.0f, -1.0f));
    leftLeg->setBody(body);

    Hull* rightLeg = Hull::creeateCube();
    rightLeg->scaleLocalVertices(Vector3(0.3f, 0.3f, 1.0f) * 0.5f);
    rightLeg->moveLocalVertices(Vector3(0.2f, 0.0f, -1.0f));
    rightLeg->setBody(body);

    return body;
}

void Scene::_shot()
{
    using namespace PE;
    using namespace QScrollEngine;

    QEntity* bullet = m_bulletEntity->clone();
    bullet->setVisibled(true);

    QScene::CameraInfo cameraInfo = this->cameraInfo();
    float vel = 50.0f;

    Body* body = new Body(&m_physicsWorld);
    body->setPosition(Vector3(cameraInfo.position.x(), cameraInfo.position.y(), cameraInfo.position.z()));
    body->setVelocity(- Vector3(cameraInfo.localZ.x() * vel, cameraInfo.localZ.y() * vel, cameraInfo.localZ.z() * vel));

    Sphere* s = new Sphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f);
    s->setBody(body);

    m_bodies.push_back(qMakePair(body, bullet));
    m_planarShadows.addEntity(bullet);
}

void Scene::_setColor(QScrollEngine::QEntity* entity, PE::Body* body)
{
    if (!body->nonSleeping()) {
        _setColor(entity, QColor(255, 255, 255));
        return;
    }
    QColor c;
    auto it = m_colors.find(body->currentCollionGroup());
    if (it == m_colors.end()) {
        c.setRgbF((std::rand() % 255) / 255.0f,
                  (std::rand() % 255) / 255.0f,
                  (std::rand() % 255) / 255.0f, 1.0f);
        m_colors.insert({ body->currentCollionGroup(), c });
    } else {
        c = it->second;
    }
    _setColor(entity, c);
}

void Scene::_setColor(QScrollEngine::QEntity* entity, const QColor& c)
{
    using namespace QScrollEngine;

    std::size_t i;
    for (i = 0; i < entity->countParts(); ++i) {
        QSh_Color* s = dynamic_cast<QSh_Color*>(entity->part(i)->shader().data());
        if (s != nullptr) {
            s->setColor(c);
        }
    }
    for (i = 0; i < entity->countEntityChilds(); ++i) {
        _setColor(entity->childEntity(i), c);
    }
}
