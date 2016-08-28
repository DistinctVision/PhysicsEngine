#include "ContactsContainer.h"
#include <utility>

namespace PE {

ContactsContainer::ContactsContainer()
{
    m_ERP_a = 0.15f;
    m_ERP_b = 0.3f;
    m_countUsedPrevContacts = m_countNotUsedPrevContacts = 0;
}

void ContactsContainer::setERP(float a, float b)
{
    m_ERP_a = a;
    m_ERP_b = b;
}

void ContactsContainer::clearCMPoints()
{
    m_countCMPoints = 0;
}

int ContactsContainer::addContactManifold(Body* bodyA, Body* bodyB, const Vector3& normal, const Material& material)
{
    m_contactManifolds.resize(m_contactManifolds.size() + 1);
    int index = (int)m_contactManifolds.size() - 1;
    ContactManifold& cm = m_contactManifolds[index];
    cm.solved = false;
    cm.bodyA = bodyA;
    cm.bodyB = bodyB;
    cm.e = 1.0f + material.e();
    cm.mu = material.mu();
    cm.normal = normal;
    //if (sd) --cm.normal;
    cm.notStatB = bodyB->isDynamic();
    cm.countPoints = 0;
    if (cm.notStatB)
        cm.bodyA->_mergeCollisionGroupAndAwake(cm.bodyB);
    return index;
}

void ContactsContainer::computeBinormalOnCM_notStatB(int nCM, int nPoint)
{
    ContactManifold& cm = m_contactManifolds[nCM];
    cm.infoPoint[nPoint].binormal = cm.bodyA->velocity() + cross(cm.bodyA->angularVelocity(), cm.pointA[nPoint].r) -
        (cm.bodyB->velocity() + cross(cm.bodyB->angularVelocity(), cm.pointB[nPoint].r));
    cm.infoPoint[nPoint].binormal -= cm.normal * dot(cm.normal, cm.infoPoint[nPoint].binormal);
    if (cm.infoPoint[nPoint].binormal.normalize() == 0.0f)
	{
        if (nPoint % 2)
            cm.infoPoint[nPoint].binormal = cross(Vector3(1.0f, 0.0f, 0.0f), cm.normal);
		else
            cm.infoPoint[nPoint].binormal = cross(Vector3(0.0f, 0.0f, 1.0f), cm.normal);
        if (cm.infoPoint[nPoint].binormal.inBound(PE_EPSf))
            cm.infoPoint[nPoint].binormal = Vector3(0.0f, 1.0f, 0.0f);
		else
            cm.infoPoint[nPoint].binormal.normalize();
	}
    cm.pointA[nPoint].rb = cross(cm.pointA[nPoint].r, cm.infoPoint[nPoint].binormal);
    cm.pointB[nPoint].rb = cross(cm.pointB[nPoint].r, cm.infoPoint[nPoint].binormal);
}

void ContactsContainer::solveBinormalOnCM_statB(int nCM, int nPoint)
{
    ContactManifold& cm = m_contactManifolds[nCM];
    cm.infoPoint[nPoint].binormal = cm.bodyA->velocity() + cross(cm.bodyA->angularVelocity(), cm.pointA[nPoint].r);
    cm.infoPoint[nPoint].binormal -= cm.normal * dot(cm.normal, cm.infoPoint[nPoint].binormal);
    if (cm.infoPoint[nPoint].binormal.normalize() < PE_EPSf) {
        if (nPoint % 2)
            cm.infoPoint[nPoint].binormal = cross(Vector3(1.0f, 0.0f, 0.0f), cm.normal);
		else
            cm.infoPoint[nPoint].binormal = cross(Vector3(0.0f, 0.0f, 1.0f), cm.normal);
        if (cm.infoPoint[nPoint].binormal.lengthSquared() < PE_EPSf)
            cm.infoPoint[nPoint].binormal = Vector3(0.0f,1.0f,0.0f);
		else
            cm.infoPoint[nPoint].binormal.normalize();
	}
    cm.pointA[nPoint].rb = cross(cm.pointA[nPoint].r, cm.infoPoint[nPoint].binormal);
}

void ContactsContainer::addContact(int nCM, ContactPoint& contactPoint, float xdt)
{
    ContactManifold& cm = m_contactManifolds[nCM];
    int nPoint = cm.countPoints;
	//contactPoint.pointOnBodyA = contactPoint.point;
	//contactPoint.pointOnBodyB = contactPoint.point;
    cm.pointA[nPoint].r = contactPoint.pointOnBodyA - cm.bodyA->position();
    cm.pointA[nPoint].rn = cross(cm.pointA[nPoint].r, cm.normal);
    if (cm.notStatB) {
        cm.pointB[nPoint].r = contactPoint.pointOnBodyB - cm.bodyB->position();
        cm.pointB[nPoint].rn = cross(cm.pointB[nPoint].r, cm.normal);
        computeBinormalOnCM_notStatB(nCM, nPoint);
    } else {
		solveBinormalOnCM_statB(nCM, nPoint);
    }
    contactPoint.depth -= PE_MAIN_DEPTH;
	/*if (contactPoint.depth > 0.0f)
	{*/
		contactPoint.depth *= xdt;
        cm.infoPoint[nPoint].depthA = contactPoint.depth * m_ERP_a;
        cm.infoPoint[nPoint].depthB = contactPoint.depth * m_ERP_b;
    /*} else {
        cm.infoPoint[nPoint].depthA = 0.0f;
        cm.infoPoint[nPoint].depthB = 0.0f;
	}*/
    ++cm.countPoints;
}

void ContactsContainer::addContact_static(int nCM, const Vector3& contactPoint, float depth, float xdt)//static
{
    ContactManifold& cm = m_contactManifolds[nCM];
    //cm.notStatB = false;
    int nPoint = cm.countPoints;
    cm.pointA[nPoint].r = contactPoint - cm.bodyA->position();
    cm.pointA[nPoint].rn = cross(cm.pointA[nPoint].r, cm.normal);
	solveBinormalOnCM_statB(nCM, nPoint);
    depth -= PE_MAIN_DEPTH;
    //if (contactPoint.depth > 0.0f) {
		depth *= xdt;
        cm.infoPoint[nPoint].depthA = depth * m_ERP_a;
        cm.infoPoint[nPoint].depthB = depth * m_ERP_b;
    /*} else {
        cm.infoPoint[nPoint].depthA = 0.0f;
        cm.infoPoint[nPoint].depthB = 0.0f;
	}*/
    ++cm.countPoints;
}

void ContactsContainer::addTempContactPoint(const Vector3& pointOnBodyA, const Vector3& pointOnBodyB, float depth)
{
    m_tempCMPoints[m_countCMPoints].pointOnBodyA = pointOnBodyA;
    m_tempCMPoints[m_countCMPoints].pointOnBodyB = pointOnBodyB;
    m_tempCMPoints[m_countCMPoints].point = (pointOnBodyA + pointOnBodyB) * 0.5f;
    m_tempCMPoints[m_countCMPoints].depth = depth;
    ++m_countCMPoints;
}

void ContactsContainer::addTempContactPoint_static(const Vector3& pointOnBodyA, float depth)//static
{
    m_tempCMPoints[m_countCMPoints].pointOnBodyA = pointOnBodyA;
    m_tempCMPoints[m_countCMPoints].point = pointOnBodyA;
    m_tempCMPoints[m_countCMPoints].depth = depth;
    ++m_countCMPoints;
}

bool ContactsContainer::optimizeContactPoints(int nCM, const Vector3& normal, float xdt)
{
    if (m_countCMPoints == 0) {
        m_contactManifolds.resize(m_contactManifolds.size() - 1);
		return false;
    } else if (m_countCMPoints < 5) {
        for (int k = 0; k < m_countCMPoints; ++k)
            addContact(nCM, m_tempCMPoints[k], xdt);
		return true;
	}
    Vector3 normalX, normalY;
	createNormal12(normal, normalX, normalY);
    float min_1 = PE_MAXNUMBERf, max_1 = PE_MINNUMBERf, min_2 = PE_MAXNUMBERf, max_2 = PE_MINNUMBERf, set;
	int i, j;
    m_contacts[0] = m_contacts[1] = m_contacts[2] = m_contacts[3] = nullptr;
	bool b;
    for (i = 0; i < m_countCMPoints; ++i) {
        set = dot(m_tempCMPoints[i].point, normalX);
        if (set > max_1) {
			max_1 = set;
            m_contacts[0] = &m_tempCMPoints[i];
		}
        if (set < min_1) {
			min_1 = set;
            m_contacts[1] = &m_tempCMPoints[i];
		}
        set = dot(m_tempCMPoints[i].point, normalY);
        if (set > max_2) {
			max_2 = set;
            m_contacts[2] = &m_tempCMPoints[i];
		}
        if (set < min_2) {
			min_2 = set;
            m_contacts[3] = &m_tempCMPoints[i];
		}
	}
    for (i = 0; i < 4; ++i) {
		b = true;
        for (j = i + 1; j < 4; ++j) {
            if (m_contacts[i] == m_contacts[j]) {
				b = false;
				break;
			}
		}
        if (b)
            addContact(nCM, (*m_contacts[i]), xdt);
	}
	return true;
}

void ContactsContainer::compareContacts(int nCM)
{
    ContactManifold& cm = m_contactManifolds[nCM];
	int i, j, prevCM = -1;
	bool flag, inv;
    Vector3 sumImpulseL, sumImpulseA;
    Body* bodyA = cm.bodyA;
    Body* bodyB = cm.bodyB;
    if (bodyB->isStatic()) {
        cm.notStatB = false;
    } else if (bodyA->isStatic()) {
        cm.bodyA = bodyB;
        bodyB = cm.bodyB = bodyA;
        bodyA = cm.bodyB;
        R_ContactPoint t;
        for (int i = 0; i < cm.countPoints; ++i) {
            t = cm.pointA[i];
            cm.pointA[i] = cm.pointB[i];
            cm.pointB[i] = t;
            cm.pointA[i].rn = - cm.pointA[i].rn;
            cm.pointB[i].rn = - cm.pointB[i].rn;
            cm.pointA[i].rb = - cm.pointA[i].rb;
            cm.pointB[i].rb = - cm.pointB[i].rb;
		}
        cm.normal = - cm.normal;
        cm.notStatB = false;
    } else {
        cm.notStatB = true;
    }
    bodyA->addContact(nCM);
	bodyB->addContact(nCM);
    for (i = 0; i < (int)bodyA->m_prev_contacts.size(); ++i) {
        if (m_prev_contactManifolds[bodyA->m_prev_contacts[i]].bodyB == bodyB) {
            prevCM = bodyA->m_prev_contacts[i];
			inv = false;
			break;
        } else if (m_prev_contactManifolds[bodyA->m_prev_contacts[i]].bodyA == bodyB) {
            prevCM = bodyA->m_prev_contacts[i];
			inv = true;
			break;
		}
	}
    if ((prevCM >= 0)) {// && (bWS)
        ContactManifold& prev_cm = m_prev_contactManifolds[prevCM];
        if (inv) {
            for (i = 0; i < cm.countPoints; ++i) {
				flag = true;
                for (j = 0; j < prev_cm.countPoints; ++j) {
                    if ((cm.pointA[i].r - prev_cm.pointB[j].r).inBound(PE_EPS_forContact)) {
                        //--cm.infoPoint[i].binormal;
                        cm.infoPoint[i].impulse = prev_cm.infoPoint[j].impulse;
                        cm.infoPoint[i].impulseFriction = - prev_cm.infoPoint[j].impulseFriction;
                        cm.infoPoint[i].pseudoImpulse = prev_cm.infoPoint[j].pseudoImpulse;
                        sumImpulseL = (cm.normal * cm.infoPoint[i].impulse) + (cm.infoPoint[i].binormal * cm.infoPoint[i].impulseFriction);
						bodyA->applyLinearImpulse(sumImpulseL);
                        sumImpulseA = (cm.pointA[i].rn * cm.infoPoint[i].impulse) + (cm.pointA[i].rb * cm.infoPoint[i].impulseFriction);
						bodyA->applyAngularImpulse(sumImpulseA);
                        bodyA->applyLinearPseudoImpulse(cm.normal, cm.infoPoint[i].pseudoImpulse);
                        bodyA->applyAngularPseudoImpulse(cm.pointA[i].rn, cm.infoPoint[i].pseudoImpulse);
                        if (cm.notStatB) {
                            sumImpulseL = - sumImpulseL;
							bodyB->applyLinearImpulse(sumImpulseL);
                            sumImpulseA = (cm.pointB[i].rn * (- cm.infoPoint[i].impulse)) + (cm.pointB[i].rb * (-cm.infoPoint[i].impulseFriction));
							bodyB->applyAngularImpulse(sumImpulseA);
                            bodyB->applyLinearPseudoImpulse(cm.normal, (-cm.infoPoint[i].pseudoImpulse));
                            bodyB->applyAngularPseudoImpulse(cm.pointB[i].rn, (-cm.infoPoint[i].pseudoImpulse));
						}
                        ++m_countUsedPrevContacts;
						flag = false;
						break;
					}
				}
                if (flag) {
                    cm.infoPoint[i].impulse = 0.0f;
                    cm.infoPoint[i].impulseFriction = 0.0f;
                    cm.infoPoint[i].pseudoImpulse = 0.0f;
                    m_countNotUsedPrevContacts++;
				}
			}
        } else {
            for (i = 0; i < cm.countPoints; ++i) {
				flag = true;
                for (j = 0; j < prev_cm.countPoints; ++j) {
                    if ((cm.pointA[i].r - prev_cm.pointA[j].r).inBound(PE_EPS_forContact)) {
                        cm.infoPoint[i].impulse = prev_cm.infoPoint[j].impulse;
                        cm.infoPoint[i].impulseFriction = prev_cm.infoPoint[j].impulseFriction;
                        cm.infoPoint[i].pseudoImpulse = prev_cm.infoPoint[j].pseudoImpulse;
                        sumImpulseL = (cm.normal * cm.infoPoint[i].impulse) + (cm.infoPoint[i].binormal * cm.infoPoint[i].impulseFriction);
						bodyA->applyLinearImpulse(sumImpulseL);
                        sumImpulseA = (cm.pointA[i].rn * cm.infoPoint[i].impulse) + (cm.pointA[i].rb * cm.infoPoint[i].impulseFriction);
						bodyA->applyAngularImpulse(sumImpulseA);
                        bodyA->applyLinearPseudoImpulse(cm.normal, cm.infoPoint[i].pseudoImpulse);
                        bodyA->applyAngularPseudoImpulse(cm.pointA[i].rn, cm.infoPoint[i].pseudoImpulse);
                        if (cm.notStatB) {
                            sumImpulseL = - sumImpulseL;
							bodyB->applyLinearImpulse(sumImpulseL);
                            sumImpulseA = (cm.pointB[i].rn * (- cm.infoPoint[i].impulse)) + (cm.pointB[i].rb * (- cm.infoPoint[i].impulseFriction));
							bodyB->applyAngularImpulse(sumImpulseA);
                            bodyB->applyLinearPseudoImpulse(cm.normal, (-cm.infoPoint[i].pseudoImpulse));
                            bodyB->applyAngularPseudoImpulse(cm.pointB[i].rn, (-cm.infoPoint[i].pseudoImpulse));
						}
                        ++m_countUsedPrevContacts;
						flag = false;
						break;
					}
				}
                if (flag) {
                    cm.infoPoint[i].impulse = 0.0f;
                    cm.infoPoint[i].impulseFriction = 0.0f;
                    cm.infoPoint[i].pseudoImpulse = 0.0f;
                    ++m_countNotUsedPrevContacts;
				}
			}
		}
    } else {
        for (i = 0; i < cm.countPoints; ++i) {
            cm.infoPoint[i].impulse = 0.0f;
            cm.infoPoint[i].impulseFriction = 0.0f;
            cm.infoPoint[i].pseudoImpulse = 0.0f;
            ++m_countNotUsedPrevContacts;
		}
	}
}

void ContactsContainer::deleteAllContacts()
{
    m_contactManifolds.swap(m_prev_contactManifolds);
    m_contactManifolds.resize(0);
    m_countUsedPrevContacts = 0;
    m_countNotUsedPrevContacts = 0;
}

int ContactsContainer::countUsedPrevContacts() const
{
    return m_countUsedPrevContacts;
}

int ContactsContainer::countNotUsedPrevContacts() const
{
    return m_countNotUsedPrevContacts;
}

void ContactsContainer::updateCollisionGroups()
{
    for (auto it = m_contactManifolds.begin(); it != m_contactManifolds.end(); ++it)
        it->bodyA->_mergeCollisionGroup(it->bodyB);
}

std::size_t ContactsContainer::countContactManifolds() const
{
    return m_contactManifolds.size();
}

const ContactsContainer::ContactManifold& ContactsContainer::contactManifold(std::size_t index) const
{
    return m_contactManifolds[index];
}

} // namespace PE
