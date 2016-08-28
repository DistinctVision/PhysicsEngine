#include "Vector3.h"

namespace PE {

float dot(const Vector3& a, const Vector3& b)
{
    return (a.x * b.x + a.y * b.y + a.z * b.z);
}

Vector3 cross(const Vector3& a, const Vector3& b)
{
    return Vector3(((a.y * b.z) - (a.z * b.y)),
                   ((a.z * b.x) - (a.x * b.z)),
                   ((a.x * b.y) - (a.y * b.x)));
}

Vector3 projection_n(const Vector3& rayNormal, const Vector3& rayPoint, const Vector3& point)
{
    float dpi = dot(point - rayPoint, rayNormal);
    return (rayPoint + (rayNormal * dpi));
}

Vector3 projection(const Vector3& rayDir, const Vector3& rayPoint, const Vector3& point)
{
    float lengthSquared = rayDir.lengthSquared();
    if (lengthSquared > PE_EPSf) {
        float dpi = (dot(point - rayPoint, rayDir)) / lengthSquared;
        return (rayPoint + (rayDir * dpi));
    }
    return point;
}

Vector3 projectionToPlane_n(const Vector3& planeNormal, const Vector3& planePoint, const Vector3& point)
{
    return (point - (planeNormal * dot(point - planePoint, planeNormal)));
}

Vector3 projectionToPlane(const Vector3& planeDir, const Vector3& planePoint, const Vector3& point)
{
    float lengthSquared = planeDir.lengthSquared();
    if (lengthSquared > PE_EPSf) {
        return (point - (planeDir * (dot(point - planePoint, planeDir) / lengthSquared)));
    }
    return point;
}

Vector3 sin(const Vector3& v)
{
    return Vector3(std::sin(v.x), std::sin(v.y), std::sin(v.z));
}

Vector3 cos(const Vector3& v)
{
    return Vector3(std::cos(v.x), std::cos(v.y), std::cos(v.z));
}

Vector3 rotateAroundVector(const Vector3& B, const Vector3& n, float angle)
{
    Vector3 dz = n * dot(n , B);
    Vector3 dx = B - dz;
    Vector3 dy = cross(n, dx);
    return ((dx * std::cos(angle) + dy * std::sin(angle)) + dz);
}

/*inline bool getParametrT(vector3& pdir, vector3& dir, vector3& point, float& t)
{
    if (abs(dir.x) > EPS) t = (point.x - pdir.x) / dir.x;
    else if (abs(dir.y) > EPS) t = (point.y - pdir.y) / dir.y;
    else if (abs(dir.z) > EPS) t = (point.z - pdir.z) / dir.z;
    else return false;
    return true;
}*/

bool collisionPlaneRay(Vector3& result, float& t, const Vector3& planeNormal, float planeD,
                       const Vector3& rayPoint, const Vector3& rayDir)
{
    t = dot(rayDir, planeNormal);
    if (std::fabs(t) < PE_EPSf)
        return false;
    t = - ((dot(rayPoint, planeNormal) + planeD) / t);
    result = rayPoint + rayDir * t;
    return true;
}

bool collisionPlaneRay(Vector3& result, float& t,
                       const Vector3& planeNormal, const Vector3& planePoint,
                       const Vector3& rayPoint, const Vector3& rayDir)
{
    t = dot(rayDir, planeNormal);
    if (std::fabs(t) < PE_EPSf)
        return false;
    t = - ((dot(rayPoint - planePoint, planeNormal)) / t);
    result = rayPoint + rayDir * t;
    return true;
}

bool collisionLinesOnPlane(Vector3& result, float& tA, float& tB,
                           const Vector3& pointA_1, const Vector3& pointA_2,
                           const Vector3& pointB_1, const Vector3& pointB_2,
                           const Vector3& planeNormal)
{
    bool b = false;
    Vector3 pA_dir = pointA_2 - pointA_1;
    float lengthSquared = pA_dir.lengthSquared();
    if (lengthSquared <= PE_EPSf)
        return false;
    tA = dot(pointB_1 - pointA_1, pA_dir) / lengthSquared;
    if ((tA >= 0.0f) && (tA <= 1.0f))
        b = true;
    if (!b) {
        tA = dot(pointB_2 - pointA_1, pA_dir) / lengthSquared;
        if ((tA >= 0.0f) && (tA <= 1.0f))
            b = true;
    }
    if (b) {
        Vector3 pB_dir = pointB_2 - pointB_1;
        Vector3 dir = cross(pB_dir, planeNormal);
        if (!collisionPlaneRay(result, tA, dir, (- dot(pointB_1, dir)), pointA_1, pA_dir))
            return false;
        if ((tA >= 0.0f) && (tA <= 1.0f)) {
            if (abs(pB_dir.x) > PE_EPSf)
                tB = (result.x - pointB_1.x) / pB_dir.x;
            else if (abs(pB_dir.y) > PE_EPSf)
                tB = (result.y - pointB_1.y) / pB_dir.y;
            else if (abs(pB_dir.z) > PE_EPSf)
                tB = (result.z - pointB_1.z) / pB_dir.z;
            else
                return false;
            if ((tB >= 0.0f) && (tB <= 1.0f))
                return true;
            return false;
        } else {
            return false;
        }
    }
    return false;
}

/*inline bool collisionLineOnPlane1(Vector3& result, Vector3& p1, vector3 &p1dir, vector3 &p2, vector3 &p2dir, vector3 &planeNormal, float &t1, float &t2)
{
    bool b = false;
    float lengthSquare = p1dir.lengthSquare();
    if (lengthSquare <= EPS) return false;
    t1 = dot(p2 - p1, p1dir) / lengthSquare;
    if ((t1 >= 0.0f) && (t1 <= 1.0f)) b = true;
    if (b == false)
    {
        t1 = dot((p2 + p2dir) - p1, p1dir) / lengthSquare;
        if ((t1 >= 0.0f) && (t1 <= 1.0f)) b = true;
    }
    if (b)
    {
        vector3 dir = p2dir ^ planeNormal;
        if (collisionPlaneLine(result, dir, (-dot(p2, dir)), p1, p1dir, t1) == false) return false;
        if ((t1 >= 0.0f) && (t1 <= 1.0f))
        {
            if (abs(p2dir.x) > EPS) t2 = (result.x - p2.x) / p2dir.x;
            else if (abs(p2dir.y) > EPS) t2 = (result.y - p2.y) / p2dir.y;
            else if (abs(p2dir.z) > EPS) t2 = (result.z - p2.z) / p2dir.z;
            else return false;
            if ((t2 >= 0.0f) && (t2 <= 1.0f)) return true;
            return false;
        }
        else return false;
    }
    return false;
}

inline bool collisionLineOnPlane2(vector3 &result, vector3 &p1, vector3 &p1dir, vector3 &p2, vector3 &p2dir, vector3 &planeNormal, float& t1)
{
    bool b = false;
    float lengthSquare = p1dir.lengthSquare();
    if (lengthSquare <= EPS) return false;
    vector3 dir = p2dir ^ planeNormal;
    return collisionPlaneLine(result, dir, p2, p1, p1dir, t1);
}*/

void createNormal12(const Vector3& normal, Vector3& normalX, Vector3& normalY)
{
    normalX.set(1.0f, 0.0f, 2.0f);
    if (normalX.equal(normal))
        normalX.set(2.0f, 0.0f, - 1.0f);
    normalY = cross(normal, normalX);
    normalX = cross(normal, normalY);
}

} // namespace PE
