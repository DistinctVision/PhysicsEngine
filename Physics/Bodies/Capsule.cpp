#include "Capsule.h"
#include "Body.h"

namespace PE {

Capsule::Capsule(const Vector3& vertexA, const Vector3& vertexB, float radius):
    Shape()
{
    m_type = TypeShape::Capsule;
    m_local_vertices.resize(2);
    m_global_vertices.resize(2);
    m_local_vertices[0] = vertexA;
    m_local_vertices[1] = vertexB;
    m_radius = radius;
    m_index = std::numeric_limits<std::size_t>::max();
    _updateDir();
}

Capsule::Capsule(float length, float radius):
    Shape()
{
    m_type = TypeShape::Capsule;
    m_local_vertices.resize(2);
    m_global_vertices.resize(2);
    m_local_vertices[0].set(0.0f, 0.0f, - length * 0.5f);
    m_local_vertices[1].set(0.0f, 0.0f, length * 0.5f);
    m_radius = radius;
    m_index = std::numeric_limits<std::size_t>::max();
    _updateDir();
}

float Capsule::radius() const
{
    return m_radius;
}

void Capsule::setRadius(float radius)
{
    m_radius = radius;
}

Vector3 Capsule::localVertexA() const
{
    return m_local_vertices[0];
}

Vector3 Capsule::vertexA() const
{
    return m_global_vertices[0];
}

Vector3 Capsule::localVertexB() const
{
    return m_local_vertices[1];
}

Vector3 Capsule::vertexB() const
{
    return m_global_vertices[1];
}

void Capsule::setLocalVertices(const Vector3& vertexA, const Vector3& vertexB)
{
    m_local_vertices[0] = vertexA;
    m_local_vertices[1] = vertexB;
    _updateDir();
}

Vector3 Capsule::localDir() const
{
    return m_local_dir;
}

Vector3 Capsule::dir() const
{
    return m_global_dir;
}

float Capsule::length()
{
    return m_length;
}

void Capsule::_updateDir()
{
    m_local_dir = m_local_vertices[1] - m_local_vertices[0];
    m_length = m_local_dir.normalize();
}

Shape* Capsule::copy() const
{
    Capsule* capsule = new Capsule();
    capsule->m_local_vertices = m_local_vertices;
    capsule->m_global_vertices = m_global_vertices;
    capsule->m_local_dir = m_local_dir;
    capsule->m_global_dir = m_global_dir;
    capsule->m_radius = m_radius;
    capsule->m_material = m_material;
    return capsule;
}

void Capsule::update()
{
    m_global_vertices[0] = m_body->position() + m_body->rotation().vectorRotated(m_local_vertices[0]);
    m_global_vertices[1] = m_body->position()+ m_body->rotation().vectorRotated(m_local_vertices[1]);
    m_global_dir = m_body->rotation().vectorRotated(m_local_dir);
    m_bounds.min = m_global_vertices[0];
    m_bounds.min.minAxis(m_global_vertices[1]);
    m_bounds.min -= Vector3(m_radius, m_radius, m_radius);
    m_bounds.max = m_global_vertices[0];
    m_bounds.max.maxAxis(m_global_vertices[1]);
    m_bounds.max += Vector3(m_radius, m_radius, m_radius);
}

} // namespace PE
