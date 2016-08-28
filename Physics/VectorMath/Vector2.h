#ifndef PE_VECTOR2_H
#define PE_VECTOR2_H

#include <cmath>
#include <climits>
#include <limits>

#if defined(QT_CORE_LIB)
#include <QtMath>
#endif

#define PE_PIf 3.14159265359f
#define PE_EPSf 5e-4f
#define PE_EPSf_SQUARE PE_EPSf * PE_EPSf
#define PE_RAD2DEGf 57.295776f // (180/PI)
#define PE_DEG2RADf 0.017453292f // (PI/180)
#define PE_HALF_PIf 1.5707964f

#define PE_MAXNUMBERf std::numeric_limits<float>::max()
#define PE_MINNUMBERf - PE_MAXNUMBERf

namespace PE {

class Vector2
{
public:
    float x, y;

    Vector2()
    {
        //x = y = 0.0f;
    }

    Vector2(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    Vector2(float a)
    {
        x = y = a;
    }

    void set(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    Vector2 operator + (const Vector2& b) const
    {
        return Vector2(x + b.x, y + b.y);
    }

    Vector2 operator - (const Vector2& b) const
    {
        return Vector2(x - b.x, y - b.y);
    }

    void operator += (const Vector2& b)
    {
        x += b.x;
        y += b.y;
    }

    void operator -= (const Vector2& b)
    {
        x -= b.x;
        y -= b.y;
    }

    void operator *= (float a)
    {
        x *= a;
        y *= a;
    }

    void operator /= (float a)
    {
        x /= a;
        y /= a;
    }

    Vector2 operator * (float a) const
    {
        return Vector2(x * a, y * a);
    }

    Vector2 operator / (float a) const
    {
        return Vector2(x / a, y / a);
    }

    const float& operator[] (int n) const
    {
        return (&x)[n];
    }

    float& operator[] (int n)
    {
        return (&x)[n];
    }

    Vector2 operator - () const
    {
        return Vector2( - x, - y);
    }

    float lengthSquared() const
    {
        return (x * x + y * y);
    }

    float length() const
    {
        return (std::sqrt(x * x + y * y));
    }

    float normalize()
    {
        float l = length();
        if (std::fabs(l) < PE_EPSf) {
            x = 0.0f;
            y = 0.0f;
            return 0.0f;
        }
        x /= l;
        y /= l;
        return l;
    }

    Vector2 rotatedFromEulerAngle(float angle) const
    {
        return Vector2(x * std::cos(angle),
                       y * std::sin(angle));
    }

    void minAxis(const Vector2& b)
    {
        if (b.x < x)
            x = b.x;
        if (b.y < y)
            y = b.y;
    }

    void maxAxis(const Vector2& b)
    {
        if (b.x > x)
            x = b.x;
        if (b.y > y)
            y = b.y;
    }

    bool equalDir(const Vector2& p) const
    {
        float kx, ky;
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
        return true;
    }

    bool equal(const Vector2& b) const
    {
        if (std::fabs(b.x - x) > PE_EPSf)
            return false;
        if (std::fabs(b.y - y) > PE_EPSf)
            return false;
        return true;
    }

    bool equal(float a) const
    {
        if (std::fabs(x - a) > PE_EPSf)
            return false;
        if (std::fabs(y - a) > PE_EPSf)
            return false;
        return true;
    }

    bool inBound(float a) const
    {
        return ((std::fabs(x) < a) && (std::fabs(y) < a));
    }
};

float dot(const Vector2& a, const Vector2& b);

float cross(const Vector2& a, const Vector2& b);

Vector2 sin(const Vector2& v);

Vector2 cos(const Vector2& v);

Vector2 projection(const Vector2& rayDir, const Vector2& rayPoint, const Vector2& point);

bool collisionRays(Vector2& result, float& rayA_t, float& rayB_t,
                   const Vector2& rayA_point, const Vector2& rayA_dir,
                   const Vector2& rayB_point, const Vector2& rayB_dir);

bool collisionLines(Vector2& result, float& rayA_t, float& rayB_t,
                    const Vector2& rayA_point, const Vector2& rayA_dir,
                    const Vector2& rayB_point, const Vector2& rayB_dir);

} // namespace PE

#endif // PE_VECTOR2_H
