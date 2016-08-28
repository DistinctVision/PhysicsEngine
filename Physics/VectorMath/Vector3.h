#ifndef PE_VECTOR3_H
#define PE_VECTOR3_H

#include "Vector2.h"

namespace PE {

class Vector3
{
public:
    float x, y, z;

    Vector3()
    {
        //x = y = z = 0.0f;
    }

    Vector3(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector3(float a)
    {
        x = y = z = a;
    }

    inline void set(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector3 operator + (const Vector3& b) const
    {
        return Vector3(x + b.x,
                       y + b.y,
                       z + b.z);
    }

    Vector3 operator - (const Vector3& b) const
    {
        return Vector3(x - b.x,
                       y - b.y,
                       z - b.z);
    }

    void operator += (const Vector3& b)
    {
        x += b.x;
        y += b.y;
        z += b.z;
    }

    void operator -= (const Vector3& b)
    {
        x -= b.x;
        y -= b.y;
        z -= b.z;
    }

    void operator *= (float a)
    {
        x *= a;
        y *= a;
        z *= a;
    }

    void operator /= (float a)
    {
        x /= a;
        y /= a;
        z /= a;
    }

    Vector3 operator * (float a) const
    {
        return Vector3(x * a, y * a, z * a);
    }

    Vector3 operator / (float a) const
    {
        return Vector3(x / a, y / a, z / a);
    }

    const float& operator[] (int n) const
    {
        return (&x)[n];
    }

    float& operator[] (int n)
    {
        return (&x)[n];
    }

    Vector3 operator - () const
    {
        return Vector3( - x, - y, - z);
    }

    float lengthSquared() const
    {
        return (x * x + y * y + z * z);
    }

    float length() const
    {
        return (std::sqrt(x * x + y * y + z * z));
    }

    float normalize()
    {
        float l = length();
        if (std::fabs(l) < PE_EPSf)
        {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
            return 0.0f;
        }
        x /= l;
        y /= l;
        z /= l;
        return l;
    }

    Vector3 rotateFromEulerAngle(const Vector3& sinAngle, const Vector3& cosAngle) const
    {
        //roll->pitch->yaw
        Vector3 p(x, y, z);

        //roll
        float t = p.x;
        p.x = t * cosAngle.z - p.y * sinAngle.z;
        p.y = p.y * cosAngle.z + t * sinAngle.z;
        //pitch
        t = p.z;
        p.z = t * cosAngle.x + p.y * sinAngle.x;
        p.y = p.y * cosAngle.x - t * sinAngle.x;
        //yaw
        t = p.x;
        p.x = t * cosAngle.y + p.z * sinAngle.y;
        p.z = p.z * cosAngle.y - t * sinAngle.y;

        return p;
    }

    void minAxis(const Vector3& b)
    {
        if (b.x < x)
            x = b.x;
        if (b.y < y)
            y = b.y;
        if (b.z < z)
            z = b.z;
    }

    void maxAxis(const Vector3& p)
    {
        if (p.x > x)
            x = p.x;
        if (p.y > y)
            y = p.y;
        if (p.z > z)
            z = p.z;
    }

    bool equalDir(const Vector3& p) const
    {
        float kx, ky, kz;
        if (std::fabs(x) < PE_EPSf) {
            if (std::fabs(p.x) < PE_EPSf)
                kx = 0.0f;
            else
                return false;
        } else {
            kx = p.x / x;
        }
        if (std::fabs(y) < PE_EPSf) {
            if (std::fabs(p.y) < PE_EPSf)
                ky = 0.0f;
            else
                return false;
        } else {
            ky = p.y / y;
            if (kx != 0.0f)
                if (std::fabs(kx - ky) >= PE_EPSf)
                    return false;
        }
        if (std::fabs(z) < PE_EPSf) {
            if (std::fabs(p.z) < PE_EPSf)
                kz = 0.0f;
            else
                return false;
        } else {
            kz = p.z / z;
            if (kx != 0.0f)
                if (std::fabs(kx - kz) >= PE_EPSf)
                    return false;
            if (ky != 0.0f)
                if (std::fabs(ky - kz) >= PE_EPSf)
                    return false;
        }
        return true;
    }

    bool equal(const Vector3& p) const
    {
        if (std::fabs(p.x - x) > PE_EPSf)
            return false;
        if (std::fabs(p.y - y) > PE_EPSf)
            return false;
        if (std::fabs(p.z - z) > PE_EPSf)
            return false;
        return true;
    }

    bool equal(float a) const
    {
        if (std::fabs(x - a) > PE_EPSf)
            return false;
        if (std::fabs(y - a) > PE_EPSf)
            return false;
        if (std::fabs(z - a) > PE_EPSf)
            return false;
        return true;
    }

    bool inBound(float a) const
    {
        return ((std::fabs(x) < a) && (std::fabs(y) < a) && (std::fabs(z) < a));
    }
};

float dot(const Vector3& a, const Vector3& b);

Vector3 cross(const Vector3& a, const Vector3& b);

Vector3 projection_n(const Vector3& rayNormal, const Vector3& rayPoint, const Vector3& point);

Vector3 projection(const Vector3& rayDir, const Vector3& rayPoint, const Vector3& point);

Vector3 projectionToPlane_n(const Vector3& planeNormal, const Vector3& planePoint, const Vector3& point);

Vector3 projectionToPlane(const Vector3& planeDir, const Vector3& planePoint, const Vector3& point);

Vector3 sin(const Vector3& v);

Vector3 cos(const Vector3& v);

Vector3 rotateAroundVector(const Vector3& B, const Vector3& n, float angle);

bool collisionPlaneRay(Vector3& result, float& t, const Vector3& planeNormal, float planeD,
                       const Vector3& rayPoint, const Vector3& rayDir);

bool collisionPlaneRay(Vector3& result, float& t,
                       const Vector3& planeNormal, const Vector3& planePoint,
                       const Vector3& rayPoint, const Vector3& rayDir);

bool collisionLinesOnPlane(Vector3& result, float& tA, float& tB,
                           const Vector3& pointA_1, const Vector3& pointA_2,
                           const Vector3& pointB_1, const Vector3& pointB_2,
                           const Vector3& planeNormal);

void createNormal12(const Vector3& normal, Vector3& normalX, Vector3& normalY);

} // namespace PE

#endif // PE_VECTOR3_H
