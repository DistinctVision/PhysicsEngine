#ifndef PE_SOLVER_H
#define PE_SOLVER_H

#include "../VectorMath/Vector3.h"
#include "../Settings.h"
#include "../Bodies/Body.h"
#include "ContactTypes.h"
#include "ContactsContainer.h"
#include "../CollisionDetected/CollisionDetected.h"

namespace PE {

class Solver:
        public CollisionDetected
{
public:
    Solver();

    void setCountIterations(int solverCountIterations, int splitImpulsesCountIterations);
    int solverCountIterations() const;
    int splitImpulsesIterations() const;

    void preSolve(ContactManifold& cM, Body* bodyA, Body* bodyB);
    void preSolve_static(ContactManifold& cM, Body* bodyA);
    void solveImpulse(float e, Body* bodyA, Body* bodyB, const Vector3& normal, const R_ContactPoint& cPA, const R_ContactPoint& cPB, InfoPointOnCM& cInfo);
    void solveImpulse_static(float e, Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, InfoPointOnCM& cInfo);
    void solveImpulseFriction(float mu, Body* bodyA, Body* bodyB, const R_ContactPoint& cPA, const R_ContactPoint& cPB, InfoPointOnCM& cInfo);
    void solveImpulseFriction_static(float mu, Body* bodyA, const R_ContactPoint& cPA, InfoPointOnCM& cInfo);
    void solvePseudoImpulse(Body* bodyA, Body* bodyB, const Vector3& normal, const R_ContactPoint& cPA, const R_ContactPoint& cPB, InfoPointOnCM& cInfo);
    void solvePseudoImpulse_static(Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, InfoPointOnCM& cInfo);

    void preSolve();
    void solveContacts();
    void solvePseudoContacts();
    virtual void solve();

private:
    int m_solverCountIterations;
    int m_splitImpulsesIterations;
};

} // namespace PE

#endif // PE_SOLVER_H
