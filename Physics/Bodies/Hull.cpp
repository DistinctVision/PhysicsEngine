#include "Hull.h"
#include "../VectorMath/RotationMatrix.h"
#include "QuickHull.h"
#include "Body.h"

namespace PE {

Hull::Hull():
    Shape()
{
    m_type = TypeShape::Hull;
}


void Hull::setLocalVertex(int index, float x, float y, float z)
{
    m_local_vertices[index].set(x, y, z);
}

void Hull::setLocalVertex(int index, const Vector3& vertex)
{
    m_local_vertices[index] = vertex;
}

void Hull::setCountVertices(int count)
{
    m_local_vertices.resize(count);
    m_global_vertices.resize(count);
}

void Hull::setVertices(const std::vector<Vector3>& vertices)
{
    m_local_vertices = vertices;
    m_global_vertices.resize(vertices.size());
}

void Hull::moveLocalVertices(const Vector3& v)
{
    for (auto it = m_local_vertices.begin(); it != m_local_vertices.end(); ++it) {
        *it += v;
    }
}

void Hull::scaleLocalVertices(const Vector3& scale)
{
    for (auto it = m_local_vertices.begin(); it != m_local_vertices.end(); ++it) {
        it->set(it->x * scale.x, it->y * scale.y, it->z * scale.z);
    }
}

Vector3 Hull::localVertex(int index) const
{
    return m_local_vertices[index];
}

Vector3 Hull::vertex(int index) const
{
    return m_global_vertices[index];
}

int Hull::countPolygons() const
{
    return (int)m_polygons.size();
}

Polygon& Hull::polygon(int index)
{
    return m_polygons[index];
}

const Polygon& Hull::polygon(int index) const
{
    return m_polygons[index];
}

bool Hull::formedHull(float epsinon, int maxCountVerticesOnPoligon, int maxCountPoligons)
{
    QuickHull algoritm;
    return algoritm.qHull(this, epsinon, maxCountVerticesOnPoligon, maxCountPoligons);
}

Shape* Hull::copy() const
{
    Hull* hull = new Hull();
    hull->m_local_vertices = m_local_vertices;
    hull->m_global_vertices = m_global_vertices;
    hull->m_polygons = m_polygons;
    hull->m_material = m_material;
    return hull;
}

void Hull::update()
{
    m_bounds.init();

    const RotationMatrix& rotation = m_body->rotation();
    Vector3 position = m_body->position();
    for (std::size_t i = 0; i < m_local_vertices.size(); ++i) {
        m_global_vertices[i] = rotation.vectorRotated(m_local_vertices[i]) + position;
        m_bounds.min.minAxis(m_global_vertices[i]);
        m_bounds.max.maxAxis(m_global_vertices[i]);
    }
}

Hull* Hull::creeateCube(const Vector3& scale)
{
    Hull* cube = new Hull();
    cube->m_local_vertices.resize(8);
    cube->m_global_vertices.resize(8);
    cube->m_local_vertices[0].set(scale.x, scale.y, scale.z);
    cube->m_local_vertices[1].set(scale.x, scale.y, - scale.z);
    cube->m_local_vertices[2].set(scale.x, - scale.y, scale.z);
    cube->m_local_vertices[3].set(scale.x, - scale.y, - scale.z);
    cube->m_local_vertices[4].set(- scale.x, scale.y, scale.z);
    cube->m_local_vertices[5].set(- scale.x, scale.y, - scale.z);
    cube->m_local_vertices[6].set(- scale.x, - scale.y, scale.z);
    cube->m_local_vertices[7].set(- scale.x, - scale.y, - scale.z);
    cube->formedHull();
    return cube;
}

} // namespace PE

