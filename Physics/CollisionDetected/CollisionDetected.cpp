#include "CollisionDetected.h"
#include <cmath>
#include <utility>
#include "../Bodies/Sphere.h"
#include "../Bodies/Capsule.h"
#include "../Bodies/Hull.h"

namespace PE {

CollisionDetected::CollisionDetected():
    ContactsContainer()
{
}

void CollisionDetected::collision(Sphere* sphereA, Sphere* sphereB, float xdt)
{
    Vector3 normal = sphereB->position() - sphereA->position();
    float dis = normal.lengthSquared(), rSum = sphereA->radius() + sphereB->radius();
    if (dis <= rSum * rSum) {
        if (dis < PE_EPSf)
            return;
        dis = std::sqrt(dis);
		normal /= dis;
        ContactPoint contact;
		contact.depth = (rSum - dis);
        contact.pointOnBodyA = sphereA->m_global_vertices[0] + (normal * sphereA->radius());
        contact.pointOnBodyB = sphereB->m_global_vertices[0] - (normal * sphereB->radius());
        int nCM = addContactManifold(sphereB->body(), sphereA->body(), normal,
                                     sphereA->material().mixed(sphereB->material()));
		addContact(nCM, contact, xdt);
        compareContacts(nCM);
	}
}

void CollisionDetected::collision(Capsule* capsule, Sphere* sphere, float xdt)
{
    //if (!collision(capsule->bounds(), sphere->bounds()))
    //    return;
	int nS;
    float length = capsule->length();
    float proj = dot(sphere->m_global_vertices[0] - capsule->m_global_vertices[0], capsule->dir()),
            rSum = sphere->radius() + capsule->radius();
    ContactPoint contact;
    Vector3 normal;
    if ((proj >= 0.0f) && (proj <= length)) {
        contact.pointOnBodyA = capsule->m_global_vertices[0] + (capsule->dir() * proj);
        normal = contact.pointOnBodyA - sphere->m_global_vertices[0];
        length = normal.lengthSquared();
        if (length <= rSum * rSum) {
            length = std::sqrt(length);
			normal /= length;
            contact.pointOnBodyA -= normal * capsule->radius();
            contact.pointOnBodyB = sphere->m_global_vertices[0] + (normal * sphere->radius());
			contact.depth = rSum - length;
            int nCM = addContactManifold(capsule->body(), sphere->body(), normal,
                                         capsule->material().mixed(sphere->material()));
			addContact(nCM, contact, xdt);
            compareContacts(nCM);
		}
		return;
    } else if (proj <= 0.0f) {
        nS = 0;
    } else {
        nS = 1;
    }
    normal = capsule->m_global_vertices[nS] - sphere->m_global_vertices[0];
    length = normal.lengthSquared();
    if (length <= rSum * rSum) {
        length = std::sqrt(length);
		normal /= length;
        contact.pointOnBodyA = capsule->m_global_vertices[nS] - (normal * capsule->radius());
        contact.pointOnBodyB = sphere->m_global_vertices[0] + (normal * sphere->radius());
		contact.depth = rSum - length;
        int nCM = addContactManifold(capsule->body(), sphere->body(), normal,
                                     capsule->material().mixed(sphere->material()));
		addContact(nCM, contact, xdt);
        compareContacts(nCM);
	}
}

void CollisionDetected::collision(Sphere* sphere, Capsule* capsule, float xdt)
{
	collision(capsule, sphere, xdt);
}

void CollisionDetected::collision(Capsule* capsuleA, Capsule* capsuleB, float xdt)
{
    //if (!collision(capsuleA->bounds(), capsuleB->bounds()))
    //    return;
    float rSum = capsuleA->radius() + capsuleB->radius();
    Vector3 normal;
    ContactPoint contact;
    if (!m_gjk.compute(normal, capsuleA, capsuleB)) {
        contact.depth = normal.lengthSquared();
        if (contact.depth > (rSum * rSum))
            return;
        contact.depth = std::sqrt(contact.depth);
        if (contact.depth <= PE_EPSf)
            return;
		normal /= contact.depth;
		contact.depth = rSum - contact.depth;
    } else {
        m_epa.compute(normal, capsuleA, capsuleB, m_gjk.getSimplex(), m_gjk.getNSimplex());
        normal = - normal;
        contact.depth = rSum - m_epa.minDepth;
	}
    int nCM = addContactManifold(capsuleA->body(), capsuleB->body(), normal,
                                 capsuleA->material().mixed(capsuleB->material()));
    float mS1 = dot(capsuleA->dir(), normal),
          mS2 = dot(capsuleB->dir(), normal);
    bool bAxisA = (fabs(mS1) < PE_EPSf),
         bAxisB = (fabs(mS2) < PE_EPSf);
    if((bAxisA) && (bAxisB)) {
        Vector3 dir = cross(capsuleB->dir(), normal);
        if (collisionPlaneRay(contact.pointOnBodyA, mS1, dir, capsuleB->m_global_vertices[0], capsuleA->m_global_vertices[0], capsuleA->dir())) {
            contact.pointOnBodyA -= normal * capsuleA->radius();
			contact.pointOnBodyB = contact.pointOnBodyA + (normal * (contact.depth));
			addContact(nCM, contact, xdt);
            compareContacts(nCM);
			return;
		}
	}
    if (bAxisA) {
        float pr1 = dot(capsuleB->m_global_vertices[0] - capsuleA->m_global_vertices[0], capsuleA->dir()),
              pr2 = dot(capsuleB->m_global_vertices[1] - capsuleA->m_global_vertices[0], capsuleA->dir()),
              dpr, lB = capsuleB->length();
		bool bSwap;
        if (pr1 > pr2) {
            std::swap(pr1, pr2);
			bSwap = true;
        } else {
            bSwap = false;
        }
		dpr = pr2 - pr1;
        if (dpr < PE_EPSf) {
            contact.pointOnBodyA = capsuleA->m_global_vertices[0] + (capsuleA->dir() * pr1) - (normal * capsuleA->radius());
            contact.depth = dot(capsuleA->m_global_vertices[0] - capsuleB->m_global_vertices[0], normal);
            pr1 = dot(capsuleA->m_global_vertices[0] - capsuleB->m_global_vertices[1], normal);
            if (std::fabs(pr1) < std::fabs(contact.depth)) {
				contact.depth = rSum - pr1;
                contact.pointOnBodyB = capsuleB->m_global_vertices[1] + (normal * capsuleB->radius());
            } else {
                contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (normal * capsuleB->radius());
				contact.depth = rSum - contact.depth;
			}
			addContact(nCM, contact, xdt);
        } else {
            Vector3 temp;
            float rSumSquared = rSum * rSum;
            if (pr1 > - PE_EPSf) {
                contact.pointOnBodyA = capsuleA->m_global_vertices[0] + (capsuleA->dir() * pr1);
                if (bSwap)
                    contact.pointOnBodyB = capsuleB->m_global_vertices[1];
                else
                    contact.pointOnBodyB = capsuleB->m_global_vertices[0];
            } else {
                contact.pointOnBodyA = capsuleA->m_global_vertices[0];
				mS1 = ( - pr1 / dpr) * lB;
                if (bSwap)
                    contact.pointOnBodyB = capsuleB->m_global_vertices[1] - (capsuleB->dir() * mS1);
                else
                    contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (capsuleB->dir() * mS1);
			}
			temp = contact.pointOnBodyA - contact.pointOnBodyB;
            if (temp.lengthSquared() <= rSumSquared) {
                contact.pointOnBodyA -= normal * capsuleA->radius();
                contact.pointOnBodyB += normal * capsuleB->radius();
				contact.depth = rSum - dot(temp, normal);
				addContact(nCM, contact, xdt);
			}
            mS2 = capsuleA->length();
            if (pr2 < mS2 + PE_EPSf) {
                contact.pointOnBodyA = capsuleA->m_global_vertices[0] + (capsuleA->dir() * pr2);
                if (bSwap)
                    contact.pointOnBodyB = capsuleB->m_global_vertices[0];
                else
                    contact.pointOnBodyB = capsuleB->m_global_vertices[1];
            } else {
                contact.pointOnBodyA = capsuleA->m_global_vertices[1];
				mS1 = ((mS2 - pr1) / dpr) * lB;
                if (bSwap)
                    contact.pointOnBodyB = capsuleB->m_global_vertices[1] - (capsuleB->dir() * mS1);
                else
                    contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (capsuleB->dir() * mS1);
			}
			temp = contact.pointOnBodyA - contact.pointOnBodyB;
            if (temp.lengthSquared() <= rSumSquared) {
                contact.pointOnBodyA -= normal * capsuleA->radius();
                contact.pointOnBodyB += normal * capsuleB->radius();
				contact.depth = rSum - dot(temp, normal);
				addContact(nCM, contact, xdt);
			}
		}
    } else if (bAxisB) {
        float pr1 = dot(capsuleA->m_global_vertices[0] - capsuleB->m_global_vertices[0], capsuleB->dir()),
              pr2 = dot(capsuleA->m_global_vertices[1] - capsuleB->m_global_vertices[0], capsuleB->dir()), dpr, lA = capsuleA->length();
		bool bSwap;
        if (pr1 > pr2) {
            std::swap(pr1, pr2);
			bSwap = true;
		}
		else bSwap = false;
		dpr = pr2 - pr1;
        if (dpr < PE_EPSf) {
            contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (capsuleB->dir() * pr1) + (normal * capsuleB->radius());
            contact.depth = dot(capsuleA->m_global_vertices[0] - capsuleB->m_global_vertices[0], normal);
            pr1 = dot(capsuleA->m_global_vertices[1] - capsuleB->m_global_vertices[0], normal);
            if (std::fabs(pr1) < std::fabs(contact.depth)) {
				contact.depth = rSum - pr1;
                contact.pointOnBodyA = capsuleA->m_global_vertices[1] - (normal * capsuleA->radius());
            } else {
				contact.depth = rSum - contact.depth;
                contact.pointOnBodyA = capsuleA->m_global_vertices[0] - (normal * capsuleA->radius());
			}
			addContact(nCM, contact, xdt);
        } else {
            Vector3 temp;
            float rSumSquared = rSum * rSum;
            if (pr1 > - PE_EPSf) {
                contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (capsuleB->dir() * pr1);
                if (bSwap)
                    contact.pointOnBodyA = capsuleA->m_global_vertices[1];
                else
                    contact.pointOnBodyA = capsuleA->m_global_vertices[0];
            } else {
                contact.pointOnBodyB = capsuleB->m_global_vertices[0];
				mS1 = ( - pr1 / dpr) * lA;
                if (bSwap)
                    contact.pointOnBodyA = capsuleA->m_global_vertices[1] - (capsuleA->dir() * mS1);
                else
                    contact.pointOnBodyA = capsuleA->m_global_vertices[0] + (capsuleA->dir() * mS1);
			}
			temp = contact.pointOnBodyA - contact.pointOnBodyB;
            if (temp.lengthSquared() <= rSumSquared) {
                contact.pointOnBodyA -= normal * capsuleA->radius();
                contact.pointOnBodyB += normal * capsuleB->radius();
				contact.depth = rSum - dot(temp, normal);
				addContact(nCM, contact, xdt);
			}
            mS2 = capsuleB->length();
            if (pr2 < mS2 + PE_EPSf) {
                contact.pointOnBodyB = capsuleB->m_global_vertices[0] + (capsuleB->dir() * pr2);
                if (bSwap)
                    contact.pointOnBodyA = capsuleA->m_global_vertices[0];
                else
                    contact.pointOnBodyA = capsuleA->m_global_vertices[1];
            } else {
                contact.pointOnBodyB = capsuleB->m_global_vertices[1];
				mS1 = ((mS2 - pr1) / dpr) * lA;
                if (bSwap)
                    contact.pointOnBodyA = capsuleA->m_global_vertices[1] - (capsuleA->dir() * mS1);
                else
                    contact.pointOnBodyA = capsuleA->m_global_vertices[0] + (capsuleA->dir() * mS1);
			}
			temp = contact.pointOnBodyA - contact.pointOnBodyB;
            if (temp.lengthSquared() <= rSumSquared) {
                contact.pointOnBodyA -= normal * capsuleA->radius();
                contact.pointOnBodyB += normal * capsuleB->radius();
				contact.depth = rSum - dot(temp, normal);
				addContact(nCM, contact, xdt);
			}
		}
    } else {
		int nSA, nSB;
        if (mS1 > 0.0f)
            nSA = 0;
        else
            nSA = 1;
        if (mS2 < 0.0f)
            nSB = 0;
        else
            nSB = 1;
        contact.pointOnBodyA = capsuleA->m_global_vertices[nSA] - (normal * capsuleA->radius());
        contact.pointOnBodyB = capsuleB->m_global_vertices[nSB] + (normal * capsuleB->radius());
		addContact(nCM, contact, xdt);
	}
    compareContacts(nCM);
}

void CollisionDetected::collision(Hull* hull, Sphere* sphere, float xdt)
{
    //if (!collision(hull->bounds(), sphere->bounds()))
    //    return;
	float depth;
    Vector3 normal;
    if (!m_gjk.compute(normal, hull, sphere)) {
        depth = normal.lengthSquared();
        if (depth > (sphere->radius() * sphere->radius()))
            return;
        depth = std::sqrt(depth);
        if (depth <= PE_EPSf)
            return;
		normal /= depth;
        depth = (sphere->radius() - depth);
    } else {
        m_epa.compute(normal, hull, sphere, m_gjk.getSimplex(), m_gjk.getNSimplex());
        normal = - normal;
        depth = sphere->radius() - m_epa.minDepth;
	}
    Body* bodyA = hull->body();
    ContactTypes::ContactPoint contact;
	contact.depth = depth;
	int nCM;
    if (bodyA->isDynamic()) {
        contact.pointOnBodyB = sphere->m_global_vertices[0] + (normal * sphere->radius());
		contact.pointOnBodyA = contact.pointOnBodyB - (normal * depth);
        nCM = addContactManifold(bodyA, sphere->body(), normal,
                                 hull->material().mixed(sphere->material()));
		addContact(nCM, contact, xdt);
    } else {
        contact.pointOnBodyA = sphere->m_global_vertices[0] + (normal * sphere->radius());
		contact.pointOnBodyB = contact.pointOnBodyA - (normal * depth);
        nCM = addContactManifold(sphere->body(), bodyA, (-normal),
                                 hull->material().mixed(sphere->material()));
		addContact(nCM, contact, xdt);
	}
    compareContacts(nCM);
}

void CollisionDetected::collision(Sphere* sphere, Hull* hull, float xdt)
{
	collision(hull, sphere, xdt);
}


inline bool CollisionDetected::vertexInPolygon(const Vector3& vertex_pos, const std::vector<Vector3>& vertexBuffer, const Polygon& polygon) const
{
    int j;
	float dis;
    for (j = 0; j < (int)polygon.vertices.size() - 1; ++j) {
        dis = dot(vertex_pos - vertexBuffer[polygon.vertices[j]],
                cross((vertexBuffer[polygon.vertices[j + 1]] - vertexBuffer[polygon.vertices[j]]), polygon.normal));
        if (dis > - PE_EPSf)
            return false;
    }
    dis = dot(vertex_pos - vertexBuffer[polygon.vertices[j]],
            cross((vertexBuffer[polygon.vertices[0]] - vertexBuffer[polygon.vertices[j]]), polygon.normal));
    if (dis > - PE_EPSf)
        return false;
	return true;
}

bool CollisionDetected::vertexToPolygon(Vector3& result, const Vector3& vertex_pos,
                                        const std::vector<Vector3>& vertexBuffer, const Polygon& polygon) const
{
    int j;
    float dis, minDis = PE_MAXNUMBERf;
    Vector3 dir, point;
    for (j = 0; j < (int) polygon.vertices.size(); ++j) {
        dis = (vertex_pos - vertexBuffer[polygon.vertices[j]]).lengthSquared();
        if (dis < minDis) {
			minDis = dis;
            result = vertexBuffer[polygon.vertices[j]];
		}
	}
    for (j = 0; j < (int) polygon.vertices.size() - 1; ++j) {
        dir = vertexBuffer[polygon.vertices[j + 1]] - vertexBuffer[polygon.vertices[j]];
        dis = dot(vertex_pos - vertexBuffer[polygon.vertices[j]], dir) / dir.lengthSquared();
        if ((dis >= 0.0f) && (dis <= 1.0f)) {
            point = (dir * dis) + vertexBuffer[polygon.vertices[j]];
            dis = (vertex_pos - point).lengthSquared();
            if (dis < minDis) {
				minDis = dis;
				result = point;
				return true;
			}
		}
    }
    dir = vertexBuffer[polygon.vertices[0]] - vertexBuffer[polygon.vertices[j]];
    dis = dot(vertex_pos - vertexBuffer[polygon.vertices[j]], dir) / dir.lengthSquared();
    if ((dis >= 0.0f) && (dis <= 1.0f)) {
        point = (dir * dis) + vertexBuffer[polygon.vertices[j]];
        dis = (vertex_pos - point).lengthSquared();
        if (dis < minDis) {
			minDis = dis;
			result = point;
			return true;
		}
	}
	return false;
}

void CollisionDetected::collision(Hull* hull, Capsule* capsule, float xdt)
{
    //if (!collision(hull->bounds(), capsule->bounds()))
    //    return;
    Vector3 normal;
    ContactPoint contact;
    if (!m_gjk.compute(normal, hull, capsule)) {
        contact.depth = normal.lengthSquared();
        if (contact.depth > (capsule->radius() * capsule->radius()))
            return;
        contact.depth = std::sqrt(contact.depth);
        if (contact.depth <= PE_EPSf)
            return;
		normal /= - contact.depth;
        contact.depth = (capsule->radius() - contact.depth);
    } else {
        m_epa.compute(normal, hull, capsule, m_gjk.getSimplex(), m_gjk.getNSimplex());
        contact.depth = capsule->radius() - m_epa.minDepth;
	}
    Body* hullBody = hull->body();
    Body* capsuleBody = capsule->body();
    const std::vector<Vector3>& hullVertices = hull->m_global_vertices;
    const std::vector<Vector3>& capsuleVertices = capsule->m_global_vertices;
    int nCM = addContactManifold(hullBody, capsuleBody, (-normal),
                                 hull->material().mixed(capsule->material()));
    Vector3 temp = hullBody->rotation().vectorToAxis(normal);
    const Polygon* polygon = &hull->polygon(0);
    int i, cP = hull->countPolygons();
    float dis, maxA = dot(temp, polygon->normal);
    for(i = 1; i < cP; ++i) {
        dis = dot(temp, hull->polygon(i).normal);
        if (dis > maxA) {
			maxA = dis;
            polygon = &hull->polygon(i);
		}
	}
    if (std::fabs(1.0f - maxA) < PE_EPSf) {
        normal = hullBody->rotation().vectorRotated(polygon->normal);
        int countContacts = 0;
		float t1, t2;
        Vector3 pA = hullVertices[polygon->vertices[0]];
        contact.depth = dot(capsuleVertices[0] - pA, normal);
        if (contact.depth <= capsule->radius()) {
            if (vertexInPolygon(capsuleVertices[0], hull->m_global_vertices, *polygon)) {
                contact.pointOnBodyA = capsuleVertices[0] - (normal * contact.depth);
                contact.pointOnBodyB = capsuleVertices[0] - (normal * capsule->radius());
                contact.depth = capsule->radius() - contact.depth;
				addContact(nCM, contact, xdt);
				countContacts++;
			}
		}
        contact.depth = dot(capsuleVertices[1] - pA, normal);
        if (contact.depth <= capsule->radius()) {
            if (vertexInPolygon(capsuleVertices[1], hullVertices, *polygon)) {
                contact.pointOnBodyA = capsuleVertices[1] - (normal * contact.depth);
                contact.pointOnBodyB = capsuleVertices[1] - (normal * capsule->radius());
                contact.depth = capsule->radius() - contact.depth;
				addContact(nCM, contact, xdt);
				countContacts++;
			}
		}
        if (countContacts < 2) {
            Vector3 p1 = projectionToPlane_n(normal, pA, capsuleVertices[0]),
                p2 = projectionToPlane_n(normal, pA, capsuleVertices[1]);
            for (i = 0; i < (int)polygon->vertices.size() - 1; ++i) {
                if (collisionLinesOnPlane(contact.pointOnBodyA, t1, t2, p1, p2,
                                          hullVertices[polygon->vertices[i]], hullVertices[polygon->vertices[i+1]], normal))
				{
                    contact.pointOnBodyB = capsuleVertices[0] + ((capsuleVertices[1] - capsuleVertices[0]) * t2);
					contact.depth = dot(contact.pointOnBodyB - contact.pointOnBodyA, normal);
                    if (contact.depth <= capsule->radius()) {
                        contact.depth = capsule->radius() - contact.depth;
                        contact.pointOnBodyB -= normal * capsule->radius();
						addContact(nCM, contact, xdt);
                        ++countContacts;
                        if (countContacts >= 2)
                            break;
					}
				}
			}
            if (countContacts < 2) {
                if (collisionLinesOnPlane(contact.pointOnBodyA, t1, t2, p1, p2, hullVertices[polygon->vertices[i]],
                                          hullVertices[polygon->vertices[0]], normal))
				{
                    contact.pointOnBodyB = capsuleVertices[0] + ((capsuleVertices[1] - capsuleVertices[0]) * t2);
					contact.depth = dot(contact.pointOnBodyB - contact.pointOnBodyA, normal);
                    if (contact.depth <= capsule->radius()) {
                        contact.depth = capsule->radius() - contact.depth;
                        contact.pointOnBodyB -= normal * capsule->radius();
						addContact(nCM, contact, xdt);
                        ++countContacts;
					}
				}
			}
		}
    } else {
		int vP[2];
        vP[0] = polygon->vertices[0];
        float maxB = PE_MINNUMBERf;
        maxA = dot(hullVertices[vP[0]], normal);
        for(i = 1; i < (int)polygon->vertices.size(); ++i) {
            dis = dot(hullVertices[polygon->vertices[i]], normal);
            if (dis > maxA) {
				maxB = maxA;
				vP[1] = vP[0];
				maxA = dis;
                vP[0] = polygon->vertices[i];
            } else if (dis > maxB) {
				maxB = dis;
                vP[1] = polygon->vertices[i];
			}
		}
        Vector3 edge = hullVertices[vP[1]] - hullVertices[vP[0]];
        float lengthEdge = edge.normalize(), mS1 = dot(edge, normal), mS2 = dot(capsule->dir(), normal);
        bool bAxisA = (std::fabs(mS1) < PE_EPSf);
        bool bAxisB = (std::fabs(mS2) < PE_EPSf);
        if((bAxisA) && (bAxisB)) {
            Vector3 dirP = cross(capsule->dir(), normal);
            if (collisionPlaneRay(contact.pointOnBodyA, mS1, dirP, capsuleVertices[0], hullVertices[vP[0]], edge)) {
				contact.pointOnBodyB = contact.pointOnBodyA - (normal * (contact.depth));
				addContact(nCM, contact, xdt);
                compareContacts(nCM);
				return;
			}
		}
        if (bAxisA) {
            float pr1 = dot(capsuleVertices[0] - hullVertices[vP[0]], edge),
                  pr2 = dot(capsuleVertices[1] - hullVertices[vP[0]], edge), dpr, lC = capsule->length();
			bool bSwap;
            if (pr1 > pr2) {
                std::swap(pr1, pr2);
				bSwap = true;
            } else {
                bSwap = false;
            }
			dpr = pr2 - pr1;
            if (dpr < PE_EPSf) {
                contact.pointOnBodyA = hullVertices[vP[0]] + (edge * pr1);
                contact.depth = dot(capsuleVertices[0] - hullVertices[vP[0]], normal);
                pr1 = dot(capsuleVertices[1] - hullVertices[vP[0]], normal);
                if (fabs(pr1) < fabs(contact.depth)) {
                    contact.depth = capsule->radius() - pr1;
                    contact.pointOnBodyB = capsuleVertices[1] - (normal * capsule->radius());
                } else {
                    contact.depth = capsule->radius() - contact.depth;
                    contact.pointOnBodyB = capsuleVertices[0] - (normal * capsule->radius());
				}
				addContact(nCM, contact, xdt);
            } else {
                float rSquared = capsule->radius() * capsule->radius();
                if (pr1 > - PE_EPSf) {
                    contact.pointOnBodyA = hullVertices[vP[0]] + (edge * pr1);
                    if (bSwap)
                        contact.pointOnBodyB = capsuleVertices[1];
                    else
                        contact.pointOnBodyB = capsuleVertices[0];
                } else {
                    contact.pointOnBodyA = hullVertices[vP[0]];
					mS1 = (- pr1 / dpr) * lC;
                    if (bSwap)
                        contact.pointOnBodyB = capsuleVertices[1] - (capsule->dir() * mS1);
                    else
                        contact.pointOnBodyB = capsuleVertices[0] + (capsule->dir() * mS1);
				}
				temp = contact.pointOnBodyB - contact.pointOnBodyA;
                if (temp.lengthSquared() <= rSquared) {
                    contact.pointOnBodyB -= normal * capsule->radius();
                    contact.depth = capsule->radius() - dot(temp, normal);
					addContact(nCM, contact, xdt);
				}
                if (pr2 < lengthEdge + PE_EPSf) {
                    contact.pointOnBodyA = hullVertices[vP[0]] + (edge * pr2);
                    if (bSwap)
                        contact.pointOnBodyB = capsuleVertices[0];
                    else
                        contact.pointOnBodyB = capsuleVertices[1];
                } else {
                    contact.pointOnBodyA = hullVertices[vP[1]];
					mS1 = ((lengthEdge - pr1) / dpr) * lC;
                    if (bSwap)
                        contact.pointOnBodyB = capsuleVertices[1] - (capsule->dir() * mS1);
                    else
                        contact.pointOnBodyB = capsuleVertices[0] + (capsule->dir() * mS1);
				}
				temp = contact.pointOnBodyB - contact.pointOnBodyA;
                if (temp.lengthSquared() <= capsule->radius()) {
                    contact.pointOnBodyB -= normal * capsule->radius();
                    contact.depth = capsule->radius() - dot(temp, normal);
					addContact(nCM, contact, xdt);
				}
			}
        } else if (bAxisB) {
            float pr1 = dot(hullVertices[vP[0]] - capsuleVertices[0], capsule->dir()),
                  pr2 = dot(hullVertices[vP[1]] - capsuleVertices[0], capsule->dir()), dpr;
			bool bSwap;
            if (pr1 > pr2) {
                std::swap(pr1, pr2);
				bSwap = true;
            } else {
                bSwap = false;
            }
			dpr = pr2 - pr1;
            if (dpr < PE_EPSf) {
                contact.pointOnBodyB = capsuleVertices[0] + (capsule->dir() * pr1);
                contact.depth = dot(capsuleVertices[0] - hullVertices[vP[0]], normal);
                pr1 = dot(capsuleVertices[0] - hullVertices[vP[1]], normal);
                if (std::fabs(pr1) < std::fabs(contact.depth)) {
                    contact.depth = capsule->radius() - pr1;
                    contact.pointOnBodyA = hullVertices[vP[1]];
                } else {
                    contact.depth = capsule->radius() - contact.depth;
                    contact.pointOnBodyA = hullVertices[vP[0]];
				}
                contact.pointOnBodyB -= normal * capsule->radius();
				addContact(nCM, contact, xdt);
            } else {
                float rSquared = capsule->radius() * capsule->radius();
                if (pr1 > -PE_EPSf) {
                    contact.pointOnBodyB = capsuleVertices[0] + (capsule->dir() * pr1);
                    if (bSwap)
                        contact.pointOnBodyA = hullVertices[vP[1]];
                    else
                        contact.pointOnBodyA = hullVertices[vP[0]];
                } else {
                    contact.pointOnBodyB = capsuleVertices[0];
					mS1 = ( - pr1 / dpr) * lengthEdge;
                    if (bSwap)
                        contact.pointOnBodyA = hullVertices[vP[1]] - (edge * mS1);
                    else
                        contact.pointOnBodyA = hullVertices[vP[0]] + (edge * mS1);
				}
				temp = contact.pointOnBodyB - contact.pointOnBodyA;
                if (temp.lengthSquared() <= rSquared) {
                    contact.pointOnBodyB -= normal * capsule->radius();
                    contact.depth = capsule->radius() - dot(temp, normal);
					addContact(nCM, contact, xdt);
				}
                mS2 = capsule->length();
                if (pr2 < mS2 + PE_EPSf) {
                    contact.pointOnBodyB = capsuleVertices[0] + (capsule->dir() * pr2);
                    if (bSwap)
                        contact.pointOnBodyA = hullVertices[vP[0]];
                    else
                        contact.pointOnBodyA = hullVertices[vP[1]];
                } else {
                    contact.pointOnBodyB = capsuleVertices[1];
					mS1 = ((mS2 - pr1) / dpr) * lengthEdge;
                    if (bSwap)
                        contact.pointOnBodyA = hullVertices[vP[1]] - (edge * mS1);
                    else
                        contact.pointOnBodyA = hullVertices[vP[0]] + (edge * mS1);
				}
				temp = contact.pointOnBodyB - contact.pointOnBodyA;
                if (temp.lengthSquared() <= rSquared) {
                    contact.pointOnBodyB -= normal * capsule->radius();
                    contact.depth = capsule->radius() - dot(temp, normal);
					addContact(nCM, contact, xdt);
				}
			}
        } else {
			int nSA, nSB;
            if (mS1 > 0.0f)
                nSA = vP[1];
            else
                nSA = vP[0];
            if (mS2 > 0.0f)
                nSB = 0;
            else
                nSB = 1;
            contact.pointOnBodyA = hullVertices[nSA];
            contact.pointOnBodyB = capsuleVertices[nSB] - (normal * capsule->radius());
			contact.depth = dot(contact.pointOnBodyA - contact.pointOnBodyB, normal);
			addContact(nCM, contact, xdt);
		}
	}
    compareContacts(nCM);
}

void CollisionDetected::collision(Capsule* capsule, Hull* hull, float xdt)
{
	collision(hull, capsule, xdt);
}

void CollisionDetected::collisionPolygonToPolygon(int nCM, const Polygon& poligonA, const Vector3& normalA, const Polygon& poligonB, const Vector3& normalB,
                                                  const Vector3& pA, const Vector3& pB,
                                                  const std::vector<Vector3>& vertexBufferA, const std::vector<Vector3>& vertexBufferB,
                                                  const Vector3& normal, float xdt)
{
    clearCMPoints();
	int i, j, previ, indexVertex0_A, indexVertex1_A, indexVertex0_B, indexVertex1_B;
	float t1, t2, depth;
    Vector3 temp, temp1;
    previ = (int)poligonA.vertices.size() - 1;
    indexVertex0_A = poligonA.vertices[previ];
    Vector3 vp, vn = projectionToPlane_n(normal, pB, vertexBufferA[indexVertex0_A]);
    for (i = 0; i < (int)poligonA.vertices.size(); ++i) {
		vp = vn;
        indexVertex1_A = poligonA.vertices[i];
        vn = projectionToPlane_n(normal, pB, vertexBufferA[indexVertex1_A]);
        for (j = 0; j < (int)poligonB.vertices.size() - 1; ++j) {
            indexVertex0_B = poligonB.vertices[j];
            indexVertex1_B = poligonB.vertices[j+1];
            if (collisionLinesOnPlane(temp, t1, t2, vp, vn, vertexBufferB[indexVertex0_B], vertexBufferB[indexVertex1_B], normal)) {
				temp1 = vertexBufferA[indexVertex0_A] + ((vertexBufferA[indexVertex1_A] - vertexBufferA[indexVertex0_A]) * t1);
				depth = dot(temp1 - pB, normalB);
                if (depth <= 0.0f) {
                    depth = std::fabs(dot(normalB * depth, normal));
					addTempContactPoint(temp1, temp, depth);
				}
			}
		}
        indexVertex0_B = poligonB.vertices[j];
        indexVertex1_B = poligonB.vertices[0];
        if (collisionLinesOnPlane(temp, t1, t2, vp, vn, vertexBufferB[indexVertex0_B], vertexBufferB[indexVertex1_B], normal)) {
			temp1 = vertexBufferA[indexVertex0_A] + ((vertexBufferA[indexVertex1_A] - vertexBufferA[indexVertex0_A]) * t1);
			depth = dot(temp1 - pB, normalB);
            if (depth <= 0.0f) {
                depth = std::fabs(dot(normalB * depth, normal));
				addTempContactPoint(temp1, temp, depth);
			}
		}
		indexVertex0_A = indexVertex1_A;
		previ = i;
	}
    for (i = 0; i < (int)poligonA.vertices.size(); ++i) {
        indexVertex0_A = poligonA.vertices[i];
		depth = dot(vertexBufferA[indexVertex0_A] - pB, normalB);
        if (depth <= 0.0f) {
            if (vertexInPolygon(vertexBufferA[indexVertex0_A], vertexBufferB, poligonB)) {
				temp = vertexBufferA[indexVertex0_A] - (normalB * depth);
                depth = std::fabs(dot(normalB * depth, normal));
				addTempContactPoint(vertexBufferA[indexVertex0_A], temp, depth);
			}
        }
	}
    for (i = 0; i < (int)poligonB.vertices.size(); ++i)
	{
        indexVertex0_B = poligonB.vertices[i];
		depth = dot(vertexBufferB[indexVertex0_B] - pA, normalA);
        if (depth <= 0.0f) {
            if (vertexInPolygon(vertexBufferB[indexVertex0_B], vertexBufferA, poligonA)) {
				temp = vertexBufferB[indexVertex0_B] - (normalA * depth);
                depth = std::fabs(dot(normalA * depth, normal));
				addTempContactPoint(temp, vertexBufferB[indexVertex0_B], depth);
			}
        }
	}
    if (optimizeContactPoints(nCM, normal, xdt))
        compareContacts(nCM);
}

void CollisionDetected::generateContactManifold(Hull* hullA, Hull* hullB, const Vector3& normal, float xdt)
{
    Body* bodyA = hullA->body();
    Body* bodyB = hullB->body();
    const std::vector<Vector3>& vertexBufferA = hullA->m_global_vertices;
    const std::vector<Vector3>& vertexBufferB = hullB->m_global_vertices;
    const RotationMatrix& rotationA = bodyA->rotation();
    const RotationMatrix& rotationB = bodyB->rotation();
    Vector3 tempnormal = rotationA.vectorToAxis(normal);
    const Polygon* polygonA = &hullA->polygon(0);
    int i, cP = hullA->countPolygons();
    float dis, maxA = dot(tempnormal, polygonA->normal);
    for(i = 1; i < cP; ++i) {
        dis = dot(tempnormal, hullA->polygon(i).normal);
        if (dis > maxA) {
			maxA = dis;
            polygonA = &hullA->polygon(i);
		}
	}
    tempnormal = - rotationB.vectorToAxis(normal);
    const Polygon* polygonB = &hullB->polygon(0);
    cP = hullB->countPolygons();
    float maxB = dot(tempnormal, polygonB->normal);
    for (i = 1; i < cP; ++i) {
        dis = dot(tempnormal, hullB->polygon(i).normal);
        if (dis > maxB) {
			maxB = dis;
            polygonB = &hullB->polygon(i);
		}
	}
    Vector3 normalA = rotationA.vectorRotated(polygonA->normal);
    Vector3 normalB = rotationB.vectorRotated(polygonB->normal);
    int nCM = addContactManifold(bodyA, bodyB, - normal,
                                 hullA->material().mixed(hullB->material()));
    int indexVertex;
    /*Vector3 pdir = cross(normalA, normalB);
    Vector3 p = vertexBufferA[poligonA->vertices[0]];
    p = p + dot(p - vertexBufferB[poligonB->vertices[0]], normalB);
    bool faceA = (std::fabs(1.0f - maxA) < PE_EPSf);
    bool faceB = (std::fabs(1.0f - maxB) < PE_EPSf);
    if ((normalA + normalB).inBound(PE_EPSf))
	{
        collisionPoligonToPoligon(poligonA, normalA, poligonB, normalB,
                        poligonA->vertex[0]->pos, poligonB->vertex[0]->pos, normal, Vector3(1,0,0), Vector3(0,0,1), xdt);
	}else{*/
        Vector3 smaxA, smaxB;
        float max = PE_MINNUMBERf;
        for (i = 0; i < (int)polygonA->vertices.size(); ++i) {
            indexVertex = polygonA->vertices[i];
			dis = dot(normal, vertexBufferA[indexVertex]);
            if (dis > max) {
				max = dis;
				smaxA = vertexBufferA[indexVertex];
			}
		}
        max = PE_MINNUMBERf;
		tempnormal = - normal;
        for (i = 0; i < (int)polygonB->vertices.size(); ++i) {
            indexVertex = polygonB->vertices[i];
			dis = dot(tempnormal, vertexBufferB[indexVertex]);
			if (dis > max)
			{
				max = dis;
				smaxB = vertexBufferB[indexVertex];
			}
		}
		//if (abs(1.0f - maxA) < abs(1.0f - maxB))
            collisionPolygonToPolygon(nCM, *polygonA, normalA, *polygonB, normalB, smaxA, smaxB, vertexBufferA, vertexBufferB, normal, xdt);
		//else
		//	collisionPoligonToPoligon(poligonB, normalB, poligonA, normalA, smaxB, smaxA, -normal, xdt);
	//}
}


void CollisionDetected::collision(Hull* hullA, Hull* hullB, float xdt)
{
    //if (!collision(hullA->bounds(), hullB->bounds()))
    //    return;
    Vector3 normal;
    if (m_gjk.compute(normal, hullA, hullB)) {
        m_epa.compute(normal, hullA, hullB, m_gjk.getSimplex(), m_gjk.getNSimplex());
        generateContactManifold(hullA, hullB, normal, xdt);
    }
}

}
