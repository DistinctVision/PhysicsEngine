#ifndef PE_CAPSULE_H
#define PE_CAPSULE_H

#include "Shape.h"

namespace PE {

class Capsule:
        public Shape
{
public:
    Capsule(float length = 2.0f, float radius = 1.0f);
    Capsule(const Vector3& vertexA, const Vector3& vertexB, float radius = 1.0f);

    float radius() const;
    void setRadius(float radius);

    Vector3 localVertexA() const;
    Vector3 vertexA() const;
    Vector3 localVertexB() const;
    Vector3 vertexB() const;
    void setLocalVertices(const Vector3& vertexA, const Vector3& vertexB);

    Vector3 localDir() const;
    Vector3 dir() const;
    float length();

    Shape* copy() const override;
    void update() override;

private:
    float m_radius;
    Vector3 m_global_dir;
    Vector3 m_local_dir;
    float m_length;

    void _updateDir();
};

} // namespace PE

#endif // PE_CAPSULE_H
