#include "Body.h"
#include "../PhysicsWorld.h"
#include "Hull.h"
#include "Sphere.h"
#include "Capsule.h"

namespace PE {

Body::Body(PhysicsWorld* physicsWorld)
{
    m_physicsWorld = physicsWorld;
    m_index = m_physicsWorld->_addBody(this);
    m_defaultCollisionGroup = std::shared_ptr<CollisionGroup>(new CollisionGroup);
    m_currentCollisionGroup = m_defaultCollisionGroup;
    m_level = 0;
    m_isEnabled = true;
    m_position.set(0.0f, 0.0f, 0.0f);
    m_velocity.set(0.0f, 0.0f, 0.0f);
    m_angularVelocity.set(0.0f, 0.0f, 0.0f);
    m_pseudoVelocity.set(0.0f, 0.0f, 0.0f);
    m_pseudoAngularVelocity.set(0.0f, 0.0f, 0.0f);
    setMass(PE_default_mass);
    setInertia(PE_default_inertia);
}

Body::~Body()
{
    while (!m_shapes.empty()) {
        delete m_shapes[m_shapes.size() - 1];
    }
    m_physicsWorld->_removeBody(m_index);
}

std::size_t Body::index() const
{
    return m_index;
}

Vector3 Body::position() const
{
    return m_position;
}

void Body::setPosition(const Vector3& position)
{
    m_position = position;
}

const RotationMatrix& Body::rotation() const
{
    return m_rotation;
}

void Body::setRotation(const Vector3& eulerAngle)
{
    m_rotation.fromEulerAngle(eulerAngle);
}

void Body::setRotation(const RotationMatrix& rotation)
{
    m_rotation = rotation;
}

Vector3 Body::velocity() const
{
    return m_velocity;
}

void Body::setVelocity(const Vector3& velocity)
{
    m_velocity = velocity;
}

Vector3 Body::angularVelocity() const
{
    return m_angularVelocity;
}

void Body::setAngularVelocity(const Vector3& angularVelocity)
{
    m_angularVelocity = angularVelocity;
}

Vector3 Body::pseudoVelocity() const
{
    return m_pseudoVelocity;
}

Vector3 Body::pseudoAngularVelocity() const
{
    return m_pseudoAngularVelocity;
}

std::size_t Body::countShapes() const
{
    return m_shapes.size();
}

Shape* Body::shape(std::size_t index)
{
    return m_shapes[index];
}

void Body::addContact(int nCM)
{
    m_contacts.push_back(nCM);
}

float Body::mass() const
{
    return (m_invMass > 0.0f) ? (1.0f / m_invMass) : 0.0f;
}

void Body::setMass(float mass)
{
    m_invMass = (mass > PE_EPSf) ? (1.0f / mass) : 0.0f;
}


#if (PE_BodyInertia == 3)
void Body::setInertia(const Vector3& inertia)
{
    for (int i = 0; i < 3; ++i)
        m_invInertia[i] = (inertia[i] > PE_EPSf) ? (1.0f / inertia[i]) : 0.0f;
}
#else
void Body::setInertia(float inertia)
{
    m_invInertia = (inertia > PE_EPSf) ? (1.0f / inertia) : 0.0f;
}
#endif


void Body::applyLinearImpulse(const Vector3& normal, float impulse)
{
    m_velocity += normal * (m_invMass * impulse);
}

void Body::applyLinearImpulse(const Vector3& lImpulse)
{
    m_velocity += lImpulse * m_invMass;
}

void Body::applyAngularImpulse(const Vector3& rn, float impulse)
{
#if (PE_BodyInertia == 3)
    m_angularVelocity += Vector3(rn.x * m_invInertia.x * impulse,
                                 rn.y * m_invInertia.y * impulse,
                                 rn.z * m_invInertia.z * impulse);
#else
    m_angularVelocity += rn * (m_invInertia * impulse);
#endif
}

void Body::applyAngularImpulse(const Vector3& aImpulse)
{
#if (PE_BodyInertia == 3)
    m_angularVelocity += Vector3(aImpulse.x * m_invInertia.x,
                                 aImpulse.y * m_invInertia.y,
                                 aImpulse.z * m_invInertia.z);
#else
    m_angularVelocity += aImpulse * m_invInertia;
#endif
}

void Body::applyImpulse(float impulse, const Vector3& normal, const Vector3& point)
{
    applyLinearImpulse(normal, impulse);
    applyAngularImpulse(cross((point - m_position), normal), impulse);
}

void Body::applyLinearPseudoImpulse(const Vector3& normal, float pseudoImpulse)
{
    m_pseudoVelocity += normal * pseudoImpulse;
}

void Body::applyLinearPseudoImpulse(const Vector3& pseudoLinearImpulse)
{
    m_pseudoVelocity += pseudoLinearImpulse;
}

void Body::applyAngularPseudoImpulse(const Vector3& rn, float pseudoImpulse)
{
#if (PE_BodyInertia == 3)
    m_pseudoAngularVelocity += Vector3(rn.x * m_invInertia.x * pseudoImpulse,
                                       rn.y * m_invInertia.y * pseudoImpulse,
                                       rn.z * m_invInertia.z * pseudoImpulse);
#else
    m_pseudoAngularVelocity += rn * (m_invInertia * pseudoImpulse);
#endif
}

void Body::applyAngularPseudoImpulse(const Vector3& pseudoAngularImpulse)
{
#if (PE_BodyInertia == 3)
    m_pseudoAngularVelocity += Vector3(pseudoAngularImpulse.x * m_invInertia.x,
                                       pseudoAngularImpulse.y * m_invInertia.y,
                                       pseudoAngularImpulse.z * m_invInertia.z);
#else
    m_pseudoAngularVelocity += pseudoAngularImpulse * m_invInertia;
#endif
}

bool Body::isStatic() const
{
    return (m_invMass <= 0.0f);
}

bool Body::isDynamic() const
{
    return (m_invMass > 0.0f);
}

const BoundsTree& Body::boundsTree() const
{
    return m_boundsTrees;
}

BoundsTree& Body::boundsTree()
{
    return m_boundsTrees;
}

void Body::calculateLocalInertia()
{
    Bounds bounds;
    bounds.init();
    for (auto it = m_shapes.begin(); it != m_shapes.end(); ++it) {
        Shape* shape = *it;
        switch (shape->type()) {
        case TypeShape::Hull: {
            Hull* hull = static_cast<Hull*>(shape);
            for (int j = 0; j < hull->countVertices(); ++j) {
                bounds.min.minAxis(hull->localVertex(j));
                bounds.max.maxAxis(hull->localVertex(j));
            }
        } break;
        case TypeShape::Sphere: {
            Sphere* sphere = static_cast<Sphere*>(shape);
            bounds.min = bounds.max = sphere->localPosition();
            float r = sphere->radius();
            bounds.min -= Vector3(r, r, r);
            bounds.max += Vector3(r, r, r);
        } break;
        case TypeShape::Capsule: {
            Capsule* capsule = static_cast<Capsule*>(shape);
            bounds.min = bounds.max = capsule->localVertexA();
            float r = capsule->radius();
            bounds.min -= Vector3(r, r, r);
            bounds.max += Vector3(r, r, r);
        } break;
        default:
            break;
        }
    }
    Vector3 d = bounds.max - bounds.min;
#if (PE_BodyInertia == 3)
    m_invInertia.x = (12.0f) / (d.y * d.y + d.z * d.z);
    m_invInertia.y = (12.0f) / (d.x * d.x + d.z * d.z);
    m_invInertia.z = (12.0f) / (d.x * d.x + d.y * d.y);
#else
    m_invInertia = (12.0f) / (d.lengthSquared());
#endif
    m_invInertia *= m_invMass;
}

void Body::update(float dt, const Vector3& gravity, float damping)
{
    m_position += ((m_velocity + m_pseudoVelocity) * dt);
    m_pseudoAngularVelocity += m_angularVelocity;
    if (!m_rotation.rotate(m_pseudoAngularVelocity, dt))
        m_angularVelocity.set(0.0f, 0.0f, 0.0f);
    m_pseudoVelocity.set(0.0f, 0.0f, 0.0f);
    m_pseudoAngularVelocity.set(0.0f, 0.0f, 0.0f);
    m_velocity = (m_velocity + (gravity * dt)) * damping;
    m_angularVelocity *= damping;
    for (auto it = m_shapes.begin(); it != m_shapes.end(); ++it) {
        (*it)->update();
    }
}

void Body::updateShapes()
{
    for (auto it = m_shapes.begin(); it != m_shapes.end(); ++it)
        (*it)->update();
}

void Body::updateBoundsTree()
{
    m_boundsTrees.update();
}

bool Body::isEnabled() const
{
    return m_isEnabled;
}

void Body::setEnabled(bool enabled)
{
    m_isEnabled = enabled;
}

std::shared_ptr<const CollisionGroup> Body::defaultCollionGroup() const
{
    return m_defaultCollisionGroup;
}

std::shared_ptr<const CollisionGroup> Body::currentCollionGroup() const
{
    return m_currentCollisionGroup;
}

bool Body::nonSleeping() const
{
    return m_currentCollisionGroup->nonSleep;
}

Body* Body::copy() const
{
    Body* body = new Body(m_physicsWorld);
    for (auto it = m_shapes.begin(); it != m_shapes.end(); ++it) {
        (*it)->copy()->setBody(body);
    }
    return body;
}

std::size_t Body::_addShape(Shape* shape)
{
    m_shapes.push_back(shape);
    m_boundsTrees.compute(m_shapes);
    return m_shapes.size() - 1;
}

void Body::_removeShape(std::size_t index)
{
    m_shapes.erase(m_shapes.begin() + index);
    m_boundsTrees.compute(m_shapes);
}

void Body::_updateContactsOnBody()
{
    m_contacts.swap(m_prev_contacts);
    m_contacts.resize(0);
}

void Body::_mergeCollisionGroup(Body* body)
{
    if (m_index > body->m_index) {
        body->m_currentCollisionGroup = m_currentCollisionGroup;
    } else {
        m_currentCollisionGroup = body->m_currentCollisionGroup;
    }
}

void Body::_mergeCollisionGroupAndAwake(Body* body)
{
    if (m_index > body->m_index) {
        body->m_currentCollisionGroup = m_currentCollisionGroup;
    } else {
        m_currentCollisionGroup = body->m_currentCollisionGroup;
    }
    m_defaultCollisionGroup->nonSleep = true;
}

} // namespace PE
