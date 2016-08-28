#include "Sphere.h"
#include "Body.h"
#include <climits>
#include <limits>

namespace PE {

Sphere::Sphere():
    Shape()
{
    m_type = TypeShape::Sphere;
    m_local_vertices.resize(1);
    m_global_vertices.resize(1);
    m_local_vertices[0].set(0.0f, 0.0f, 0.0f);
    m_radius = 0.0f;
    m_index = std::numeric_limits<std::size_t>::max();
}

Sphere::Sphere(const Vector3& localPosition, float radius):
    Shape()
{
    m_type = TypeShape::Sphere;
    m_local_vertices.resize(1);
    m_global_vertices.resize(1);
    m_local_vertices[0] = localPosition;
    m_radius = radius;
    m_index = std::numeric_limits<std::size_t>::max();
}

float Sphere::radius() const
{
    return m_radius;
}

void Sphere::setRadius(float radius)
{
    m_radius = radius;
}

Vector3 Sphere::localPosition() const
{
    return m_local_vertices[0];
}

void Sphere::setLocalPosition(const Vector3& localPosition)
{
    m_local_vertices[0] = localPosition;
}

Vector3 Sphere::position() const
{
    return m_global_vertices[0];
}

Shape* Sphere::copy() const
{
    Sphere* sphere = new Sphere();
    sphere->m_local_vertices = m_local_vertices;
    sphere->m_global_vertices = m_global_vertices;
    sphere->m_radius = m_radius;
    sphere->m_material = m_material;
    return sphere;
}

void Sphere::update()
{
    m_global_vertices[0] = m_body->position() + m_body->rotation().vectorRotated(m_local_vertices[0]);
    m_bounds.min = m_global_vertices[0] - Vector3(m_radius, m_radius, m_radius);
    m_bounds.max = m_global_vertices[0] + Vector3(m_radius, m_radius, m_radius);
}

} // namespace PE
