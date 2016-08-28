#ifndef PE_COLLISIONDETECTED_H
#define PE_COLLISIONDETECTED_H

#include "../VectorMath/Vector3.h"
#include "../Bodies/Body.h"
#include "../Bodies/Hull.h"
#include "../Dynamic/ContactTypes.h"
#include "../Dynamic/ContactsContainer.h"
#include "GJK.h"
#include "EPA.h"

namespace PE {

class CollisionDetected:
        public ContactsContainer
{
public:
    CollisionDetected();

    void collision(Sphere* sphereA, Sphere* sphereB, float xdt);
    void collision(Capsule* capsule, Sphere* sphere, float xdt);
    void collision(Sphere* sphere, Capsule* capsule, float xdt);
    void collision(Capsule* capsuleA, Capsule* capsuleB, float xdt);
    void collision(Hull* hull, Sphere* sphere, float xdt);
    void collision(Sphere* sphere, Hull* hull, float xdt);
    void collision(Hull* hull, Capsule* capsule, float xdt);
    void collision(Capsule* capsule, Hull* hull, float xdt);
    bool vertexInPolygon(const Vector3& vertex_pos, const std::vector<Vector3>& vertexBuffer, const Polygon& polygon) const;
    bool vertexToPolygon(Vector3& result, const Vector3& vertex_pos, const std::vector<Vector3>& vertexBuffer, const Polygon& polygon) const;
    void collisionPolygonToPolygon(int nCM, const Polygon& poligonA, const Vector3& normalA,
                                   const Polygon& poligonB, const Vector3& normalB, const Vector3& pA, const Vector3& pB,
                                   const std::vector<Vector3>& vertexBufferA, const std::vector<Vector3>& vertexBufferB,
                                   const Vector3& normal, float xdt);
    void generateContactManifold(Hull* hullA, Hull* hullB, const Vector3& normal, float xdt);
    void collision(Hull* hullA, Hull* hullB, float xdt);

private:
    GJK m_gjk;
    EPA m_epa;
};

} // namespace PE

#endif
