#include "Vector2.h"

namespace PE {

float dot(const Vector2& a, const Vector2& b)
{
    return (a.x * b.x + a.y * b.y);
}

float cross(const Vector2& a, const Vector2& b)
{
    return (a.x * b.y - a.y * b.x);
}

Vector2 sin(const Vector2& v)
{
    return Vector2(std::sin(v.x), std::sin(v.y));
}

Vector2 cos(const Vector2& v)
{
    return Vector2(std::cos(v.x), std::cos(v.y));
}

Vector2 projection(const Vector2& rayDir, const Vector2& rayPoint, const Vector2& point)
{
    float lengthSquared = rayDir.lengthSquared();
    if (lengthSquared > PE_EPSf) {
        float dpi = (dot(point - rayPoint, rayDir)) / lengthSquared;
        return (rayDir * dpi + rayPoint);
    }
    return rayPoint;
}

bool collisionRays(Vector2& result, float& rayA_t, float& rayB_t,
                   const Vector2& rayA_point, const Vector2& rayA_dir,
                   const Vector2& rayB_point, const Vector2& rayB_dir)
{
    if (std::fabs(rayB_dir.x) <= PE_EPSf) {
        if (std::fabs(rayA_dir.x) <= PE_EPSf)
            return false;
        result.y = rayA_dir.y / rayA_dir.x;
        rayA_t = rayB_dir.y - rayB_dir.x * result.y;
        if (std::fabs(rayA_t) <= PE_EPSf)
            return false;
        result.x = rayB_point.x - rayA_point.x;
        rayB_t = (result.x * result.y + rayA_point.y - rayB_point.y) / rayA_t;
        if ((rayB_t < -PE_EPSf) || (rayB_t > (1.0f + PE_EPSf)))
            return false;
        rayA_t = (result.x + rayB_dir.x * rayB_t) / rayA_dir.x;
        if ((rayA_t < -PE_EPSf) || (rayA_t > (1.0f + PE_EPSf)))
            return false;
        result = rayA_point + (rayA_dir * rayA_t);
        return true;
    }
    //if (std::fabs(rayA_dir.x) <= PE_EPSf)
    //    return false;
    result.y = rayB_dir.y / rayB_dir.x;
    rayB_t = rayA_dir.y - rayA_dir.x * result.y;
    if (std::fabs(rayB_t) <= PE_EPSf)
        return false;
    result.x = rayA_point.x - rayB_point.x;
    rayA_t = (result.x * result.y + rayB_point.y - rayA_point.y) / rayB_t;
    if ((rayA_t < -PE_EPSf) || (rayA_t > (1.0f + PE_EPSf)))
        return false;
    rayB_t = (result.x + rayA_dir.x * rayA_t) / rayB_dir.x;
    if ((rayB_t < -PE_EPSf) || (rayB_t > (1.0f + PE_EPSf)))
        return false;
    result = rayB_point + (rayB_dir * rayB_t);
    return true;
}

bool collisionLines(Vector2& result, float& rayA_t, float& rayB_t,
                    const Vector2& rayA_point, const Vector2& rayA_dir,
                    const Vector2& rayB_point, const Vector2& rayB_dir)
{
    if (std::fabs(rayB_dir.x) <= PE_EPSf) {
        if (std::fabs(rayA_dir.x) <= PE_EPSf)
            return false;
        result.y = rayA_dir.y / rayA_dir.x;
        rayA_t = rayB_dir.y - rayB_dir.x * result.y;
        if (std::fabs(rayA_t) <= PE_EPSf)
            return false;
        result.x = rayB_point.x - rayA_point.x;
        rayB_t = (result.x * result.y + rayA_point.y - rayB_point.y) / rayA_t;
        rayA_t = (result.x + rayB_dir.x * rayB_t) / rayA_dir.x;
        result = rayA_point + (rayA_dir * rayA_t);
        return true;
    }
    //if (std::fabs(rayA_dir.x) <= PE_EPSf)
    //    return false;
    result.y = rayB_dir.y / rayB_dir.x;
    rayB_t = rayA_dir.y - rayA_dir.x * result.y;
    if (std::fabs(rayB_t) <= PE_EPSf)
        return false;
    result.x = rayA_point.x - rayB_point.x;
    rayA_t = (result.x * result.y + rayB_point.y - rayA_point.y) / rayB_t;
    rayB_t = (result.x + rayA_dir.x * rayA_t) / rayB_dir.x;
    result = rayB_point + (rayB_dir * rayB_t);
    return true;
}

} // namespace PE
