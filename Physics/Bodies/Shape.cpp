#include "Shape.h"
#include "Body.h"
#include <climits>
#include <limits>

namespace PE {

Shape::Shape()
{
    m_type = TypeShape::Undefined;
    m_body = nullptr;
    m_index = std::numeric_limits<std::size_t>::max();
}

Shape::~Shape()
{
    if (m_body != nullptr)
        m_body->_removeShape(m_index);
}

Body* Shape::body() const
{
    return m_body;
}

void Shape::setBody(Body* body)
{
    if (m_body != nullptr)
        m_body->_removeShape(m_index);
    m_body = body;
    if (m_body != nullptr)
        m_index = m_body->_addShape(this);
}

TypeShape Shape::type() const
{
    return m_type;
}

int Shape::countVertices() const
{
    return (int)m_local_vertices.size();
}

Bounds Shape::bounds() const
{
    return m_bounds;
}

Material Shape::material() const
{
    return m_material;
}

void Shape::setMaterial(const Material& material)
{
    m_material = material;
}

Bounds Shape::getLocalBounds() const
{
    Bounds bounds;
    bounds.init();
    for (auto it = m_local_vertices.begin(); it != m_local_vertices.end(); ++it) {
        bounds.min.minAxis(*it);
        bounds.max.maxAxis(*it);
    }
    return bounds;
}

float Shape::support(Vector3& resultVertex, const Vector3& dir) const
{
    float max = PE_MINNUMBERf, set;
    for (int i = 0; i < (int)m_global_vertices.size(); ++i) {
        set = dot(m_global_vertices[i], dir);
        if (set > max) {
            max = set;
            resultVertex = m_global_vertices[i];
        }
    }
    return max;
}

float Shape::support_local(Vector3& resultVertex, const Vector3& dir) const
{
    float max = PE_MINNUMBERf, set;
    for (int i = 0; i < (int)m_global_vertices.size(); ++i) {
        set = dot(m_local_vertices[i], dir);
        if (set > max) {
            max = set;
            resultVertex = m_local_vertices[i];
        }
    }
    return max;
}

} // namespace PE
