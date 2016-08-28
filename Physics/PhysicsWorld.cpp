#include "PhysicsWorld.h"
#include <cmath>
#include <cassert>
#include <memory>
#include "Bodies/Sphere.h"
#include "Bodies/Capsule.h"
#include "Bodies/Hull.h"

namespace PE {

PhysicsWorld::PhysicsWorld(const Vector3& gravity)
{
    m_gravity = gravity;
    m_damp = 0.99f;
    m_sleepTime = 60;
    m_sleepVelocity = 0.1f;
    m_sleepAngularVelocity = 0.1f;
    m_countIterationsForCollisionGroups = 150;
}

PhysicsWorld::~PhysicsWorld()
{
    while (!m_bodies.empty())
        delete m_bodies[m_bodies.size() - 1];
}

float PhysicsWorld::damp() const
{
    return m_damp;
}

void PhysicsWorld::setDamp(float damp)
{
    m_damp = damp;
}

int PhysicsWorld::countIterationsForCollisionGroups() const
{
    return m_countIterationsForCollisionGroups;
}

void PhysicsWorld::setCountIterationsForCollisionGroups(int countIterationsForCollisionGroup)
{
    m_countIterationsForCollisionGroups = countIterationsForCollisionGroup;
}

bool PhysicsWorld::enableShockPropagation() const
{
    return m_solver.enableShockPropagation();
}

void PhysicsWorld::setEnableShockPropagation(bool enable)
{
    m_solver.setEnableShockPropagation(enable);
}

void PhysicsWorld::setCountIterations(int solverCountIterations, int splitImpulsesCountIterations)
{
    m_solver.setCountIterations(solverCountIterations, splitImpulsesCountIterations);
}

int PhysicsWorld::solverCountIterations() const
{
    return m_solver.solverCountIterations();
}

int PhysicsWorld::splitImpulsesIterations() const
{
    return m_solver.splitImpulsesIterations();
}

float PhysicsWorld::sleepVelocity() const
{
    return m_sleepVelocity;
}

void PhysicsWorld::setSleepVelocity(float sleepVelocity)
{
    m_sleepVelocity = sleepVelocity;
}

float PhysicsWorld::sleepAngularVelocity() const
{
    return m_sleepAngularVelocity;
}

void PhysicsWorld::setSleepAngularVelocity(float sleepAngularVelocity)
{
    m_sleepAngularVelocity = sleepAngularVelocity;
}

const ContactsContainer& PhysicsWorld::contactsContainer() const
{
    return m_solver;
}

void PhysicsWorld::update(float dt)
{
    assert(dt > PE_EPSf);
    _updateBodies(dt);
    _updateCollisions(1.0f / dt);
    m_solver.solve();
    for (int i = 0; i < m_countIterationsForCollisionGroups; ++i)
        m_solver.updateCollisionGroups();
}

std::size_t PhysicsWorld::_addBody(Body* body)
{
    m_bodies.push_back(body);
    return m_bodies.size() - 1;
}

void PhysicsWorld::_removeBody(std::size_t index)
{
    m_bodies.erase(m_bodies.begin() + index);
}

void PhysicsWorld::_updateBodies(float dt)
{

    for (auto it = m_bodies.begin(); it != m_bodies.end(); ++it) {
        Body* body = *it;
        if (body->isEnabled()) {
            if (body->isDynamic()) {
                if (body->isEnabled() && body->m_defaultCollisionGroup->nonSleep) {
                    if ((body->velocity().inBound(m_sleepVelocity)) &&
                        (body->angularVelocity().inBound(m_sleepAngularVelocity))) {
                        ++body->m_defaultCollisionGroup->time_without_movement;
                        if (body->m_currentCollisionGroup->time_without_movement >
                                body->m_defaultCollisionGroup->time_without_movement) {
                            body->m_currentCollisionGroup->time_without_movement =
                                    body->m_defaultCollisionGroup->time_without_movement;
                        } else {
                            body->m_defaultCollisionGroup->time_without_movement =
                                    body->m_currentCollisionGroup->time_without_movement;
                        }
                    } else {
                        body->m_defaultCollisionGroup->time_without_movement = 0;
                        body->m_currentCollisionGroup->time_without_movement = 0;
                    }
                }
            } else {
                body->_updateContactsOnBody();
            }
        }
    }
    for (auto it = m_bodies.begin(); it != m_bodies.end(); ++it) {
        Body* body = *it;
        if (body->isEnabled() && body->isDynamic()) {
            body->m_defaultCollisionGroup->time_without_movement =
                    body->m_currentCollisionGroup->time_without_movement;
            if (body->m_defaultCollisionGroup->time_without_movement < m_sleepTime) {
                body->m_defaultCollisionGroup->nonSleep = true;
                body->update(dt, m_gravity, m_damp);
                body->updateShapes();
                body->updateBoundsTree();
            } else {
                body->m_currentCollisionGroup = body->m_defaultCollisionGroup;
                body->m_defaultCollisionGroup->nonSleep = false;
            }
        }
    }
}

void PhysicsWorld::_updateCollisions(float xdt)
{
    m_solver.deleteAllContacts();
    for (auto itA = m_bodies.begin(); itA != m_bodies.end(); ++itA) {
        Body* bodyA = *itA;
        if (!bodyA->isEnabled())
            continue;
        for (auto itB = itA + 1; itB != m_bodies.end(); ++itB) {
            Body* bodyB = *itB;
            if (!bodyB->isEnabled())
                continue;
            if (bodyA->isDynamic() || bodyB->isDynamic()) {
                _updateCollision(bodyA->boundsTree(), bodyB->boundsTree(), xdt);
            }
        }
    }
}

void PhysicsWorld::_updateCollision(BoundsTree& boundsTreeA, BoundsTree& boundsTreeB, float xdt)
{
    if (boundsTreeA.isEmpty())
        return;
    if (boundsTreeB.isEmpty())
        return;
    _updateCollision(boundsTreeA, boundsTreeA.rootNode(), boundsTreeB, boundsTreeB.rootNode(), xdt);
}

void PhysicsWorld::_updateCollision(BoundsTree& boundsTreeA, BoundsTree::Node& nodeA,
                                    BoundsTree& boundsTreeB, BoundsTree::Node& nodeB,
                                    float xdt)
{
    if (!nodeA.bounds.collision(nodeB.bounds))
        return;
    if (nodeA.shape != nullptr) {
        if (nodeB.shape != nullptr) {
            _updateCollision(nodeA.shape, nodeB.shape, xdt);
        } else {
            _updateCollision(nodeA.shape, boundsTreeB, boundsTreeB.node(nodeB.indexA), xdt);
            _updateCollision(nodeA.shape, boundsTreeB, boundsTreeB.node(nodeB.indexB), xdt);
        }
    } else {
        if (nodeB.shape != nullptr) {
            _updateCollision(nodeB.shape, boundsTreeA, boundsTreeA.node(nodeA.indexA), xdt);
            _updateCollision(nodeB.shape, boundsTreeA, boundsTreeA.node(nodeA.indexB), xdt);
        } else {
            _updateCollision(boundsTreeA, boundsTreeA.node(nodeA.indexA),
                             boundsTreeB, boundsTreeB.node(nodeB.indexA),
                             xdt);
            _updateCollision(boundsTreeA, boundsTreeA.node(nodeA.indexA),
                             boundsTreeB, boundsTreeB.node(nodeB.indexB),
                             xdt);
            _updateCollision(boundsTreeA, boundsTreeA.node(nodeA.indexB),
                             boundsTreeB, boundsTreeB.node(nodeB.indexA),
                             xdt);
            _updateCollision(boundsTreeA, boundsTreeA.node(nodeA.indexB),
                             boundsTreeB, boundsTreeB.node(nodeB.indexB),
                             xdt);
        }
    }
}

void PhysicsWorld::_updateCollision(Shape* shapeA, Shape* shapeB, float xdt)
{
    switch (shapeA->type()) {
    case TypeShape::Sphere: {
        _updateCollision(static_cast<Sphere*>(shapeA), shapeB, xdt);
    } break;
    case TypeShape::Capsule: {
        _updateCollision(static_cast<Capsule*>(shapeA), shapeB, xdt);
    } break;
    case TypeShape::Hull: {
        _updateCollision(static_cast<Hull*>(shapeA), shapeB, xdt);
    } break;
    default:
        break;
    }
}

void PhysicsWorld::_updateCollision(Sphere* sphereA, Shape* shapeB, float xdt)
{
    switch (shapeB->type()) {
    case TypeShape::Sphere: {
        m_solver.collision(sphereA, static_cast<Sphere*>(shapeB), xdt);
    } break;
    case TypeShape::Capsule: {
        m_solver.collision(sphereA, static_cast<Capsule*>(shapeB), xdt);
    } break;
    case TypeShape::Hull: {
        m_solver.collision(sphereA, static_cast<Hull*>(shapeB), xdt);
    } break;
    default:
        break;
    }
}

void PhysicsWorld::_updateCollision(Capsule* capsuleA, Shape* shapeB, float xdt)
{
    switch (shapeB->type()) {
    case TypeShape::Sphere: {
        m_solver.collision(capsuleA, static_cast<Sphere*>(shapeB), xdt);
    } break;
    case TypeShape::Capsule: {
        m_solver.collision(capsuleA, static_cast<Capsule*>(shapeB), xdt);
    } break;
    case TypeShape::Hull: {
        m_solver.collision(capsuleA, static_cast<Hull*>(shapeB), xdt);
    } break;
    default:
        break;
    }
}

void PhysicsWorld::_updateCollision(Hull* hullA, Shape* shapeB, float xdt)
{
    switch (shapeB->type()) {
    case TypeShape::Sphere: {
        m_solver.collision(hullA, static_cast<Sphere*>(shapeB), xdt);
    } break;
    case TypeShape::Capsule: {
        m_solver.collision(hullA, static_cast<Capsule*>(shapeB), xdt);
    } break;
    case TypeShape::Hull: {
        m_solver.collision(hullA, static_cast<Hull*>(shapeB), xdt);
    } break;
    default:
        break;
    }
}

void PhysicsWorld::_updateCollision(Shape* shapeA,
                                    BoundsTree& boundsTreeB,
                                    BoundsTree::Node& nodeB,
                                    float xdt)
{
    if (!shapeA->bounds().collision(nodeB.bounds))
        return;
    if (nodeB.shape != nullptr) {
        _updateCollision(shapeA, nodeB.shape, xdt);
    } else {
        _updateCollision(shapeA, boundsTreeB, boundsTreeB.node(nodeB.indexA), xdt);
        _updateCollision(shapeA, boundsTreeB, boundsTreeB.node(nodeB.indexB), xdt);
    }
}

} // namespace PE
