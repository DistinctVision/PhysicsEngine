#include "Solver.h"
#include <cmath>

namespace PE {

Solver::Solver():
    CollisionDetected()
{
    m_solverCountIterations = 8;
    m_splitImpulsesIterations = 3;
}

void Solver::setCountIterations(int solverCountIterations, int splitImpulsesCountIterations)
{
    m_solverCountIterations = solverCountIterations;
    m_splitImpulsesIterations = splitImpulsesCountIterations;
}

int Solver::solverCountIterations() const
{
    return m_solverCountIterations;
}

int Solver::splitImpulsesIterations() const
{
    return m_splitImpulsesIterations;
}

void Solver::preSolve(ContactManifold& cM, Body* bodyA, Body* bodyB)
{
	cM.solved = false;
	int i;
    float invMassSum = bodyA->m_invMass + bodyB->m_invMass;
#if (PE_BodyInertia == 3)
    Vector3 invInertiaA = bodyA->m_invInertia, invInertiaB = bodyB->m_invInertia;
    for (i = 0; i < cM.countPoints; ++i) {
        cM.infoPoint[i].kNormal = invMassSum +
                (dot(cM.pointA[i].rn, Vector3(cM.pointA[i].rn.x * invInertiaA.x,
                                              cM.pointA[i].rn.y * invInertiaA.y,
                                              cM.pointA[i].rn.z * invInertiaA.z))) +
                (dot(cM.pointB[i].rn, Vector3(cM.pointB[i].rn.x * invInertiaB.x,
                                              cM.pointB[i].rn.y * invInertiaB.y,
                                              cM.pointB[i].rn.z * invInertiaB.z)));
        cM.infoPoint[i].kBinormal = invMassSum +
                (dot(cM.pointA[i].rb, Vector3(cM.pointA[i].rb.x * invInertiaA.x,
                                              cM.pointA[i].rb.y * invInertiaA.y,
                                              cM.pointA[i].rb.z * invInertiaA.z))) +
                (dot(cM.pointB[i].rb, Vector3(cM.pointB[i].rb.x * invInertiaB.x,
                                              cM.pointB[i].rb.y * invInertiaB.y,
                                              cM.pointB[i].rb.z * invInertiaB.z)));
        cM.infoPoint[i].kPseudo = 2.0f +
                (dot(cM.pointA[i].rb, Vector3(cM.pointA[i].rb.x * invInertiaA.x,
                                              cM.pointA[i].rb.y * invInertiaA.y,
                                              cM.pointA[i].rb.z * invInertiaA.z))) +
                (dot(cM.pointB[i].rb, Vector3(cM.pointB[i].rb.x * invInertiaB.x,
                                              cM.pointB[i].rb.y * invInertiaB.y,
                                              cM.pointB[i].rb.z * invInertiaB.z)));
	}
#else
    float invInertiaA = bodyA->m_invInertia, invInertiaB = bodyB->m_invInertia;
    for (i = 0; i < cM.countPoints; ++i) {
        cM.infoPoint[i].kNormal = invMassSum +
                (invInertiaA * dot(cM.pointA[i].rn, cM.pointA[i].rn)) +
                (invInertiaB * dot(cM.pointB[i].rn, cM.pointB[i].rn));
        cM.infoPoint[i].kBinormal = invMassSum +
                (invInertiaA * dot(cM.pointA[i].rb, cM.pointA[i].rb)) +
                (invInertiaB * dot(cM.pointB[i].rb, cM.pointB[i].rb));
        cM.infoPoint[i].kPseudo = 2.0f +
                (invInertiaA * dot(cM.pointA[i].rn, cM.pointA[i].rn)) +
                (invInertiaB * dot(cM.pointB[i].rn, cM.pointB[i].rn));
	}
#endif
}

void Solver::preSolve_static(ContactManifold& cM, Body* bodyA)
{
	cM.solved = false;
	int i;
    float invMassSum = bodyA->m_invMass;
#if (BodyInertia)
    Vector3 invInertiaA = bodyA->m_invInertia;
    for (i = 0; i < cM.countPoints; ++i) {
        cM.infoPoint[i].kNormal = invMassSum +
                (dot(cM.pointA[i].rn, Vector3(cM.pointA[i].rn.x * invInertiaA.x,
                                              cM.pointA[i].rn.y * invInertiaA.y,
                                              cM.pointA[i].rn.z * invInertiaA.z)));
        cM.infoPoint[i].kBinormal = invMassSum +
                (dot(cM.pointA[i].rb, Vector3(cM.pointA[i].rb.x * invInertiaA.x,
                                              cM.pointA[i].rb.y * invInertiaA.y,
                                              cM.pointA[i].rb.z * invInertiaA.z)));
        cM.infoPoint[i].kPseudo = 2.0f + (dot(cM.pointA[i].rb, Vector3(cM.pointA[i].rb.x * invInertiaA.x,
                                                                       cM.pointA[i].rb.y * invInertiaA.y,
                                                                       cM.pointA[i].rb.z * invInertiaA.z)));
	}
#else
    float invInertiaA = bodyA->m_invInertia;
    for (i = 0; i < cM.countPoints; ++i) {
        cM.infoPoint[i].kNormal = invMassSum + (invInertiaA * dot(cM.pointA[i].rn, cM.pointA[i].rn));
        cM.infoPoint[i].kBinormal = invMassSum + (invInertiaA * dot(cM.pointA[i].rb, cM.pointA[i].rb));
        cM.infoPoint[i].kPseudo = 1.0f + (invInertiaA * dot(cM.pointA[i].rn, cM.pointA[i].rn));
	}
#endif
}

void Solver::solveImpulse(float e, Body* bodyA, Body* bodyB, const Vector3& normal, const R_ContactPoint& cPA,
                          const R_ContactPoint& cPB, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity() - bodyB->velocity(), normal) +
                        dot(bodyA->angularVelocity(), cPA.rn) - dot(bodyB->angularVelocity(), cPB.rn),
                     impulse;
    impulse = (cInfo.depthA - e * nVelProj) / cInfo.kNormal;
    cInfo.impulse += impulse;
    if (cInfo.impulse < 0.0f) {
        impulse -= cInfo.impulse;
        cInfo.impulse = 0.0f;
	}
	bodyA->applyLinearImpulse(normal, impulse);
	bodyA->applyAngularImpulse(cPA.rn, impulse);
	impulse = - impulse;
	bodyB->applyLinearImpulse(normal, impulse);
	bodyB->applyAngularImpulse(cPB.rn, impulse);
}

void Solver::solveImpulse_static(float e, Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity(), normal) + dot(bodyA->angularVelocity(), cPA.rn), impulse;
    impulse = (cInfo.depthA - e * nVelProj) / cInfo.kNormal;
    cInfo.impulse += impulse;
    if (cInfo.impulse < 0.0f) {
        impulse -= cInfo.impulse;
        cInfo.impulse = 0.0f;
	}
	bodyA->applyLinearImpulse(normal, impulse);
	bodyA->applyAngularImpulse(cPA.rn, impulse);
}

void Solver::solveImpulseFriction(float mu, Body* bodyA, Body* bodyB, const R_ContactPoint& cPA,
                                  const R_ContactPoint& cPB, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity() - bodyB->velocity(), cInfo.binormal) +
            dot(bodyA->angularVelocity(), cPA.rb) - dot(bodyB->angularVelocity(), cPB.rb),
          impulseMax, impulseFriction, old;
    impulseMax = cInfo.impulse * mu;
    impulseFriction = - nVelProj / cInfo.kBinormal;
    old = cInfo.impulseFriction;
    cInfo.impulseFriction = std::max(-impulseMax, std::min((old + impulseFriction), impulseMax));
    impulseFriction = cInfo.impulseFriction - old;
    bodyA->applyLinearImpulse(cInfo.binormal, impulseFriction);
	bodyA->applyAngularImpulse(cPA.rb, impulseFriction);
	impulseFriction = - impulseFriction;
    bodyB->applyLinearImpulse(cInfo.binormal, impulseFriction);
	bodyB->applyAngularImpulse(cPB.rb, impulseFriction);
}

void Solver::solveImpulseFriction_static(float mu, Body* bodyA, const R_ContactPoint& cPA, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->velocity(), cInfo.binormal) + dot(bodyA->angularVelocity(), cPA.rb),
		impulseMax, impulseFriction, old;
    impulseMax = cInfo.impulse * mu;
    impulseFriction = - nVelProj / cInfo.kBinormal;
    old = cInfo.impulseFriction;
    cInfo.impulseFriction = fmax(-impulseMax, fmin((old + impulseFriction), impulseMax));
    impulseFriction = cInfo.impulseFriction - old;
    bodyA->applyLinearImpulse(cInfo.binormal, impulseFriction);
	bodyA->applyAngularImpulse(cPA.rb, impulseFriction);
}

void Solver::solvePseudoImpulse(Body* bodyA, Body* bodyB, const Vector3& normal,
                                const R_ContactPoint& cPA, const R_ContactPoint& cPB, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->pseudoVelocity() - bodyB->pseudoVelocity(), normal) +
                 dot(bodyA->pseudoAngularVelocity(), cPA.rn) - dot(bodyB->pseudoAngularVelocity(), cPB.rn),
          pseudoImpulse;
    pseudoImpulse = (cInfo.depthB - nVelProj) / cInfo.kPseudo;
    cInfo.pseudoImpulse += pseudoImpulse;
    if (cInfo.pseudoImpulse < 0.0f) {
        pseudoImpulse -= cInfo.pseudoImpulse;
        cInfo.pseudoImpulse = 0.0f;
	}
	bodyA->applyLinearPseudoImpulse(normal, pseudoImpulse);
	bodyA->applyAngularPseudoImpulse(cPA.rn, pseudoImpulse);
	pseudoImpulse = - pseudoImpulse;
	bodyB->applyLinearPseudoImpulse(normal, pseudoImpulse);
	bodyB->applyAngularPseudoImpulse(cPB.rn, pseudoImpulse);
}

void Solver::solvePseudoImpulse_static(Body* bodyA, const Vector3& normal, const R_ContactPoint& cPA, InfoPointOnCM& cInfo)
{
    float nVelProj = dot(bodyA->pseudoVelocity(), normal) + dot(bodyA->pseudoAngularVelocity(), cPA.rn), pseudoImpulse;
    pseudoImpulse = (cInfo.depthB - nVelProj) / cInfo.kPseudo;
    cInfo.pseudoImpulse += pseudoImpulse;
    if (cInfo.pseudoImpulse < 0.0f) {
        pseudoImpulse -= cInfo.pseudoImpulse;
        cInfo.pseudoImpulse = 0.0f;
	}
	bodyA->applyLinearPseudoImpulse(normal, pseudoImpulse);
	bodyA->applyAngularPseudoImpulse(cPA.rn, pseudoImpulse);
}

void Solver::preSolve()
{
    int j;
    for (std::size_t i = 0; i < m_contactManifolds.size(); ++i) {
        ContactManifold& cm = m_contactManifolds[i];
        if (cm.notStatB) {
            cm.bodyA->_mergeCollisionGroup(cm.bodyB);
            preSolve(cm, cm.bodyA, cm.bodyB);
            for (j = 0; j < cm.countPoints; ++j) {
                solveImpulse(cm.e, cm.bodyA, cm.bodyB, cm.normal, cm.pointA[j], cm.pointB[j], cm.infoPoint[j]);
                solveImpulseFriction(cm.mu, cm.bodyA, cm.bodyB, cm.pointA[j], cm.pointB[j], cm.infoPoint[j]);
            }
        } else {
            preSolve_static(cm, cm.bodyA);
            for (j = 0; j < cm.countPoints; ++j) {
                solveImpulse_static(cm.e, cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
                solveImpulseFriction_static(cm.mu, cm.bodyA, cm.pointA[j], cm.infoPoint[j]);
            }
        }
    }
}

void Solver::solveContacts()
{
    int j;
    for (std::size_t i = 0; i < m_contactManifolds.size(); ++i) {
        ContactManifold& cm = m_contactManifolds[i];
        if (cm.notStatB) {
            cm.bodyA->_mergeCollisionGroup(cm.bodyB);
            for (j = 0; j < cm.countPoints; ++j) {
                solveImpulse(cm.e, cm.bodyA, cm.bodyB, cm.normal, cm.pointA[j], cm.pointB[j], cm.infoPoint[j]);
                solveImpulseFriction(cm.mu, cm.bodyA, cm.bodyB, cm.pointA[j], cm.pointB[j], cm.infoPoint[j]);
            }
        } else {
            for (j = 0; j < cm.countPoints; ++j) {
                solveImpulse_static(cm.e, cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
                solveImpulseFriction_static(cm.mu, cm.bodyA, cm.pointA[j], cm.infoPoint[j]);
            }
        }
    }
}

void Solver::solvePseudoContacts()
{
    int j;
    for (std::size_t i = 0; i < m_contactManifolds.size(); ++i) {
        ContactManifold& cm = m_contactManifolds[i];
        if (cm.notStatB) {
            cm.bodyA->_mergeCollisionGroup(cm.bodyB);
            for (j = 0; j < cm.countPoints; ++j) {
                solvePseudoImpulse(cm.bodyA, cm.bodyB, cm.normal, cm.pointA[j], cm.pointB[j], cm.infoPoint[j]);
            }
        } else {
            for (j = 0; j < cm.countPoints; ++j) {
                solvePseudoImpulse_static(cm.bodyA, cm.normal, cm.pointA[j], cm.infoPoint[j]);
            }
        }
    }
}

void Solver::solve()
{
	int i;
    preSolve();
    for (i = 1; i < m_solverCountIterations; ++i)
        solveContacts();
    for (i = 0; i < m_splitImpulsesIterations; ++i)
        solvePseudoContacts();
}

} // namespace PE
