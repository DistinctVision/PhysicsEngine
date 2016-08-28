#ifndef PE_SHOCKPROPAGATIONSOLVER_H
#define PE_SHOCKPROPAGATIONSOLVER_H

#include <vector>

#include "../VectorMath/Vector3.h"
#include "ContactTypes.h"
#include "ContactsContainer.h"
#include "Solver.h"

namespace PE {

class ShockPropagationSolver:
        public Solver
{
public:
    ShockPropagationSolver();

    bool enableShockPropagation() const;
    void setEnableShockPropagation(bool enableShockPropagation);

    void solveImpulseSP(float e, Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo);
    void solveImpulseFrictionSP(float mu, Body* bodyA, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo);
    void solvePseudoImpulseSP(Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo);
    void solveShockPropagation();

    void solve() override;

private:
    std::vector<std::size_t> m_graph_contactManifolds;
    std::size_t m_countStaticContacts;
    bool m_enableShockPropagation;

    void _swapBodyOnContactManifold(ContactManifold& cM);
    void _computeGraph();
};

} // namespace PE

#endif // PE_SHOCKPROPAGATIONSOLVER_H
