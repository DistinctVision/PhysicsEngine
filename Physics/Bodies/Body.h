#ifndef PE_BODY_H
#define PE_BODY_H

#include <cstdlib>
#include <memory>
#include "../Settings.h"
#include "../VectorMath/Vector3.h"
#include "../VectorMath/RotationMatrix.h"
#include "Shape.h"
#include "Material.h"
#include "BoundsTrees.h"

namespace PE {

class PhysicsWorld;
class CollisionDetected;
class ContactTypes;
class ContactsContainer;
class Solver;
class ShockPropagationSolver;

class Sphere;
class Hull;
class Capsule;

struct CollisionGroup
{
    int time_without_movement = 0;
    bool nonSleep = true;
};

class Body
{
public:
    Body(PhysicsWorld* physicsWorld);
    ~Body();

    std::size_t index() const;

    Vector3 position() const;
    void setPosition(const Vector3& position);

    const RotationMatrix& rotation() const;
    void setRotation(const Vector3& eulerAngle);
    void setRotation(const RotationMatrix& rotation);

    Vector3 velocity() const;
    void setVelocity(const Vector3& velocity);

    Vector3 angularVelocity() const;
    void setAngularVelocity(const Vector3& angularVelocity);

    Vector3 pseudoVelocity() const;
    Vector3 pseudoAngularVelocity() const;

    std::size_t countShapes() const;
    Shape* shape(std::size_t index);

    void addContact(int nCM);

    float mass() const;
    void setMass(float mass);

#if (PE_BodyInertia == 3)
    Vector3 inertia() const;
    void setInertia(const Vector3& inertia);
#else
    float inertia() const;
    void setInertia(float inertia);
#endif

    void applyLinearImpulse(const Vector3& normal, float impulse);
    void applyLinearImpulse(const Vector3& lImpulse);
    void applyAngularImpulse(const Vector3& rn, float impulse);
    void applyAngularImpulse(const Vector3& aImpulse);
    void applyImpulse(float impulse, const Vector3& normal, const Vector3& point);
    void applyLinearPseudoImpulse(const Vector3& normal, float pseudoImpulse);
    void applyLinearPseudoImpulse(const Vector3& pseudoLinearImpulse);
    void applyAngularPseudoImpulse(const Vector3& rn, float pseudoImpulse);
    void applyAngularPseudoImpulse(const Vector3& pseudoAngularImpulse);

    void calculateLocalInertia();

    bool isStatic() const;
    bool isDynamic() const;

    const BoundsTree& boundsTree() const;
    BoundsTree& boundsTree();

    void update(float dt, const Vector3& gravity, float damping);

    void updateShapes();
    void updateBoundsTree();

    bool isEnabled() const;
    void setEnabled(bool enabled);

    std::shared_ptr<const CollisionGroup> defaultCollionGroup() const;
    std::shared_ptr<const CollisionGroup> currentCollionGroup() const;

    bool nonSleeping() const;

    Body* copy() const;

protected:
    friend class PhysicsWorld;
    friend class Shape;
    friend class Sphere;
    friend class Hull;
    friend class Capsule;
    friend class CollisionDetected;
    friend class ContactTypes;
    friend class ContactsContainer;
    friend class Solver;
    friend class ShockPropagationSolver;

    std::shared_ptr<CollisionGroup> m_defaultCollisionGroup;
    std::shared_ptr<CollisionGroup> m_currentCollisionGroup;

    PhysicsWorld* m_physicsWorld;
    std::size_t m_index;
    Vector3 m_position;
    RotationMatrix m_rotation;
    Vector3 m_velocity, m_angularVelocity;
    Vector3 m_pseudoVelocity, m_pseudoAngularVelocity;

    float m_invMass;
#if (PE_BodyInertia == 3)
    Vector3 m_invInertia;
#else
    float m_invInertia;
#endif

    bool m_isEnabled;
    std::vector<int> m_contacts;
    std::vector<int> m_prev_contacts;
    int m_level;

    std::vector<Shape*> m_shapes;

    BoundsTree m_boundsTrees;

    std::size_t _addShape(Shape* shape);
    void _removeShape(std::size_t index);
    void _updateContactsOnBody();

    void _mergeCollisionGroup(Body* body);
    void _mergeCollisionGroupAndAwake(Body* body);


};

} // namespace PE

#endif // PE_BODY_H
