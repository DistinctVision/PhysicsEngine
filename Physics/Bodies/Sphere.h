#ifndef SPHERE_H
#define SPHERE_H

#include "Shape.h"

namespace PE {

class Body;

class Sphere:
        public Shape
{
public:
    Sphere();
    Sphere(const Vector3& localPosition, float radius);

    Vector3 localPosition() const;
    void setLocalPosition(const Vector3& localPosition);

    Vector3 position() const;

    float radius() const;
    void setRadius(float radius);

    Shape* copy() const override;
    void update() override;

private:
    float m_radius;
};

} // namespace PE

#endif // SPHERE_H
