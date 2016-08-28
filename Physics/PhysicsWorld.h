#ifndef PE_PHYSICSWORLD_H
#define PE_PHYSICSWORLD_H

#include <vector>
#include "VectorMath/Vector3.h"
#include "Bodies/Shape.h"
#include "Bodies/BoundsTrees.h"
#include "Dynamic/ShockPropagationSolver.h"

namespace PE {

class Body;

class PhysicsWorld
{
public:
    PhysicsWorld(const Vector3& gravity = Vector3(0.0f, -9.81f, 0.0f));
    ~PhysicsWorld();

    float damp() const;
    void setDamp(float damp);

    int countIterationsForCollisionGroups() const;
    void setCountIterationsForCollisionGroups(int countIterationsForCollisionGroup);

    bool enableShockPropagation() const;
    void setEnableShockPropagation(bool enable);

    void setCountIterations(int solverCountIterations, int splitImpulsesCountIterations);
    int solverCountIterations() const;
    int splitImpulsesIterations() const;

    float sleepVelocity() const;
    void setSleepVelocity(float sleepVelocity);

    float sleepAngularVelocity() const;
    void setSleepAngularVelocity(float sleepAngularVelocity);

    const ContactsContainer& contactsContainer() const;

    void update(float dt);

private:
    friend class Body;

    Vector3 m_gravity;
    float m_damp;
    int m_sleepTime;
    int m_countIterationsForCollisionGroups;
    float m_sleepVelocity;
    float m_sleepAngularVelocity;
    std::vector<Body*> m_bodies;
    ShockPropagationSolver m_solver;

    std::size_t _addBody(Body* body);
    void _removeBody(std::size_t index);\

    void _updateBodies(float dt);
    void _updateCollisions(float xdt);
    void _updateCollision(BoundsTree& boundsTreeA, BoundsTree& boundsTreeB, float xdt);
    void _updateCollision(BoundsTree& boundsTreeA, BoundsTree::Node& nodeA,
                          BoundsTree& boundsTreeB, BoundsTree::Node& nodeB,
                          float xdt);
    void _updateCollision(Shape* shapeA, Shape* shapeB, float xdt);
    void _updateCollision(Sphere* sphereA, Shape* shapeB, float xdt);
    void _updateCollision(Capsule* capsuleA, Shape* shapeB, float xdt);
    void _updateCollision(Hull* hullA, Shape* shapeB, float xdt);
    void _updateCollision(Shape* shapeA,
                          BoundsTree& boundsTreeB, BoundsTree::Node& nodeB,
                          float xdt);
};

} // namespace PE

#endif // PE_PHYSICSWORLD_H
