#include "ShockPropagationSolver.h"

namespace PE {

ShockPropagationSolver::ShockPropagationSolver():
    Solver()
{
    m_enableShockPropagation = true;
}

bool ShockPropagationSolver::enableShockPropagation() const
{
    return m_enableShockPropagation;
}

void ShockPropagationSolver::setEnableShockPropagation(bool enableShockPropagation)
{
    m_enableShockPropagation = enableShockPropagation;
}

void ShockPropagationSolver::solveImpulseSP(float e, Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity(), normal) + dot(bodyA->angularVelocity(), cPA.rn), impulse;
    impulse = (cInfo.depthA - e * nVelProj) / cInfo.kNormal;
    if (impulse > 0.0f) {
		bodyA->applyLinearImpulse(normal, impulse);
		bodyA->applyAngularImpulse(cPA.rn, impulse);
	}
}

void ShockPropagationSolver::solveImpulseFrictionSP(float mu, Body* bodyA, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity(), cInfo.binormal) + dot(bodyA->angularVelocity(), cPA.rb),
		impulseMax, impulseFriction;
    impulseMax = cInfo.impulse * mu;
    impulseFriction = - nVelProj / cInfo.kBinormal;
    impulseFriction = std::max(-impulseMax, std::min((impulseFriction), impulseMax));
    bodyA->applyLinearImpulse(cInfo.binormal, impulseFriction);
	bodyA->applyAngularImpulse(cPA.rb, impulseFriction);
}

void ShockPropagationSolver::solvePseudoImpulseSP(Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, const InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->pseudoVelocity(), normal) + dot(bodyA->pseudoAngularVelocity(), cPA.rn), pseudoImpulse;
    pseudoImpulse = (cInfo.depthB - nVelProj) / cInfo.kPseudo;
	if (pseudoImpulse > 0.0f) {
		bodyA->applyLinearPseudoImpulse(normal, pseudoImpulse);
		bodyA->applyAngularPseudoImpulse(cPA.rn, pseudoImpulse);
	}
}

void ShockPropagationSolver::_swapBodyOnContactManifold(ContactManifold& cM)
{
    std::swap(cM.bodyA, cM.bodyB);
    R_ContactPoint t;
    for (int i = 0; i < cM.countPoints; ++i) {
		t = cM.pointA[i];
		cM.pointA[i] = cM.pointB[i];
		cM.pointB[i] = t;
		cM.pointA[i].rn = - cM.pointA[i].rn;
		cM.pointB[i].rn = - cM.pointB[i].rn;
		cM.pointA[i].rb = - cM.pointA[i].rb;
		cM.pointB[i].rb = - cM.pointB[i].rb;
	}
    cM.normal = - cM.normal;
}

inline void ShockPropagationSolver::_computeGraph()
{
    m_graph_contactManifolds.resize(0);
    std::size_t i, j, k, levelA, levelB;
    for (i = 0; i < m_contactManifolds.size(); ++i) {
        if (!m_contactManifolds[i].notStatB) {
            m_contactManifolds[i].solved = true;
            m_graph_contactManifolds.push_back(i);
		}
	}
    m_countStaticContacts = m_graph_contactManifolds.size();

	bool flag_local;
	levelB = 0;
    levelA = m_graph_contactManifolds.size();
    while (levelB != m_graph_contactManifolds.size()) {
        for (i = 0; i < m_contactManifolds.size(); ++i) {
            ContactManifold& cm = m_contactManifolds[i];
            if (cm.solved == false) {
                cm.bodyA->_mergeCollisionGroup(cm.bodyB);
                for (j = levelB; j < levelA; ++j) {
                    if (m_contactManifolds[m_graph_contactManifolds[j]].bodyA == cm.bodyA) {
                        cm.solved = true;
						flag_local = true;
                        for (k = levelB; k < levelA; ++k) {
                            if (m_contactManifolds[m_graph_contactManifolds[k]].bodyA == cm.bodyB) {
                                cm.solved = true;
								flag_local = false;
								break;
							}
						}
                        if (flag_local) {
                            cm.bodyB->m_level = levelA;
                            _swapBodyOnContactManifold(cm);
                            m_graph_contactManifolds.push_back(i);
							break;
						}
                    } else if (m_contactManifolds[m_graph_contactManifolds[j]].bodyA == cm.bodyB) {
                        cm.solved = true;
						flag_local = true;
                        for (k = levelB; k < levelA; ++k) {
                            if (m_contactManifolds[m_graph_contactManifolds[k]].bodyA == cm.bodyA) {
                                cm.solved = true;
								flag_local = false;
								break;
							}
						}
                        if (flag_local) {
                            cm.bodyA->m_level = levelA;
                            //_swapBodyOnContactManifold(cm);
                            m_graph_contactManifolds.push_back(i);
							break;
						}
					}
				}
			}
		}
		levelB = levelA;
        levelA = m_graph_contactManifolds.size();
	}
}

void ShockPropagationSolver::solveShockPropagation()
{
    _computeGraph();
    std::size_t i;
    int j;
    for (i = 0; i < m_countStaticContacts; ++i) {
        ContactManifold& cm = m_contactManifolds[m_graph_contactManifolds[i]];
        for (j = 0; j < cm.countPoints; ++j) {
            solveImpulse_static(cm.e, cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
            solveImpulseFriction_static(cm.mu, cm.bodyA, cm.pointA[j], cm.infoPoint[j]);
            solvePseudoImpulse_static(cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
		}
	}
    for (i = m_countStaticContacts; i < m_graph_contactManifolds.size(); ++i) {
        ContactManifold& cm = m_contactManifolds[m_graph_contactManifolds[i]];
        for (j = 0; j < cm.countPoints; ++j) {
            preSolve_static(cm, cm.bodyA);
            solveImpulseSP(cm.e, cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
            solveImpulseFrictionSP(cm.mu, cm.bodyA, cm.pointA[j], cm.infoPoint[j]);
            solvePseudoImpulseSP(cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
		}
	}
}

void ShockPropagationSolver::solve()
{
    Solver::solve();
    if (m_enableShockPropagation)
        solveShockPropagation();
}

} // namespace PE
