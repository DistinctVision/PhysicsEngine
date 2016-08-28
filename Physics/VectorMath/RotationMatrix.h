#ifndef PE_ROTATIONMATRIX_H
#define PE_ROTATIONMATRIX_H

#include "Vector3.h"

namespace PE {

class RotationMatrix
{
public:
    RotationMatrix()
    {
        setToIdentity();
    }

    void setToIdentity()
    {
        m_axis[0].set(1.0f, 0.0f, 0.0f);
        m_axis[1].set(0.0f, 1.0f, 0.0f);
        m_axis[2].set(0.0f, 0.0f, 1.0f);
    }

    Vector3 getX() const
    {
        return m_axis[0];
    }

    Vector3 getY() const
    {
        return m_axis[1];
    }

    Vector3 getZ() const
    {
        return m_axis[2];
    }

    Vector3 vectorToAxis(const Vector3& vector) const
    {
        return Vector3(dot(vector, m_axis[0]),
                       dot(vector, m_axis[1]),
                       dot(vector, m_axis[2]));
    }

    Vector3 vectorRotated(const Vector3& vector) const
    {
        return ((m_axis[0] * vector.x) + (m_axis[1] * vector.y) + (m_axis[2] * vector.z));
    }

    const Vector3& operator[] (int n) const
    {
        return m_axis[n];
    }

    Vector3& operator[] (int n)
    {
        return m_axis[n];
    }

    bool rotate(const Vector3& angle)
    {
        float length = angle.length();
        if (std::fabs(length) > PE_EPSf) {
            Vector3 baseAxis = angle / length;
            m_axis[0] = rotateAroundVector(m_axis[0], baseAxis, length);
            m_axis[1] = rotateAroundVector(m_axis[1], baseAxis, length);
            m_axis[1] -= m_axis[0] * dot(m_axis[0], m_axis[1]);
            m_axis[1].normalize();
            m_axis[2] = cross(m_axis[0], m_axis[1]);
            return true;
        }
        return false;
    }

    bool rotate(const Vector3& angle, float dt)
    {
        float length = angle.length();
        if (std::fabs(length) > PE_EPSf) {
            Vector3 baseAxis = angle / length;
            length *= dt;
            m_axis[0] = rotateAroundVector(m_axis[0], baseAxis, length);
            m_axis[1] = rotateAroundVector(m_axis[1], baseAxis, length);
            m_axis[1] -= m_axis[0] * dot(m_axis[0], m_axis[1]);
            m_axis[1].normalize();
            m_axis[2] = cross(m_axis[0], m_axis[1]);
            return true;
        }
        return false;
    }

    Vector3 getEulerAngle() const
    {
        using namespace std;

        Vector3 angle;
        /*if (axis[2].y > 1.0f) {
            angle.x = - 90.0f;
            angle.y = - std::atan2f( - axis[1].x, axis[0].x) * PE_RAD2DEGf;
            angle.z = 0.0f;
        } else if (axis[2].y < -1.0f) {
            angle.x = 90.0f;
            angle.y = - std::atan2f( - axis[1].x, axis[0].x) * PE_RAD2DEGf;
            angle.z = 0.0f;
        } else {*/
            angle.x = asin(max(- 1.0f, min(- m_axis[2].y, 1.0f))) * PE_RAD2DEGf;
            angle.y = atan2(m_axis[2].x, m_axis[2].z) * PE_RAD2DEGf;
            angle.z = atan2(m_axis[0].y, m_axis[1].y) * PE_RAD2DEGf;
        //}
        if (angle.x < 0.0f)
            angle.x += 360.0f;
        if (angle.y < 0.0f)
            angle.y += 360.0f;
        if (angle.z < 0.0f)
            angle.z += 360.0f;
        return angle;
    }

    void fromEulerAngle(const Vector3& angle)
    {
        using namespace std;

        Vector3 angleRad = angle * PE_DEG2RADf;
        Vector3 sinAngle = sin(angleRad), cosAngle = cos(angleRad);

        m_axis[0].x = cosAngle.z * cosAngle.y + sinAngle.z * sinAngle.x * sinAngle.y;
        m_axis[0].y = sinAngle.z * cosAngle.x;
        m_axis[0].z = sinAngle.z * sinAngle.x * cosAngle.y - cosAngle.z * sinAngle.y;

        m_axis[1].x = - sinAngle.z * cosAngle.y + cosAngle.z * sinAngle.x * sinAngle.y;
        m_axis[1].y = cosAngle.z * cosAngle.x;
        m_axis[1].z = cosAngle.z * sinAngle.x * cosAngle.y + sinAngle.z * sinAngle.y;

        m_axis[2].x = cosAngle.x * sinAngle.y;
        m_axis[2].y =  - sinAngle.x;
        m_axis[2].z = cosAngle.x * cosAngle.y;
    }

private:
    Vector3 m_axis[3];
};

} // namespace PE

#endif // PE_ROTATIONMATRIX_H
