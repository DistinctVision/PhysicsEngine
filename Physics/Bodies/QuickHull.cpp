#include "QuickHull.h"
#include <utility>
#include "../CollisionDetected/GJK.h"

namespace PE {

bool QuickHull::qHull(Hull* hull, float epsilon, int maxCountVerticesOnPoligon, int maxCountTriangles)
{
    m_hull = hull;
    int countVertices = m_hull->countVertices();
    if (countVertices < 3)
        return false;
    m_vertices = m_hull->m_local_vertices;
    m_tempTrianglesStore.clear();
    int i, j, index;
    m_center = 0.0f;
    for (i=0; i<countVertices; i++)
        m_center += m_vertices[i];
    m_center /= (float)(countVertices);
    m_epsilon = epsilon;
    int tetras[4];
    if (!_findBasedTetras(tetras))
        return false;
    if (_formedBasedTetras(maxCountTriangles, tetras))
        return true;
    float set, max;
    int* vertices_used = new int[countVertices];
    for (i = 0; i < countVertices; ++i)
        vertices_used[i] = -1;
    for (i = 0, ++m_iteration; i < (int)m_tempTrianglesStore.size(); i++, ++m_iteration) {
        if (m_tempTrianglesStore[i].valid) {
            index = 0;
            max = dot(m_vertices[0], m_tempTrianglesStore[i].dir);
            for (j = 1; j < countVertices; ++j) {
                set = dot(m_vertices[j], m_tempTrianglesStore[i].dir);
                if (set > max) {
                    max = set;
                    index = j;
                }
            }
            if ((max - dot(m_vertices[m_tempTrianglesStore[i].vertices[0]], m_tempTrianglesStore[i].dir)) > epsilon) {
                if (!_addVertexToConvex(i, index))
                    return false;
            }
        }
    }
    m_poligons.clear();
    Polygon polygon;
    for (i = 0; i < (int)m_tempTrianglesStore.size(); ++i) {
        if (m_tempTrianglesStore[i].valid) {
            polygon.vertices.resize(0);
            polygon.normal = m_tempTrianglesStore[i].dir;
            polygon.normal.normalize();
            _trianglesToPoligons(polygon, i, 0, vertices_used);
            _trianglesToPoligons(polygon, i, 1, vertices_used);
            _trianglesToPoligons(polygon, i, 2, vertices_used);
            m_poligons.push_back(polygon);
        }
    }
    std::vector<Vector3> tempVertices;
    tempVertices.reserve(m_vertices.size());
    for (i = 0; i < countVertices; ++i) {
        if (vertices_used[i] >= 0) {
            tempVertices.push_back(m_vertices[vertices_used[i]]);
            vertices_used[i] = tempVertices.size() - 1;
        }
    }
    m_hull->m_local_vertices = std::move(tempVertices);
    for (i = 0; i < (int)m_poligons.size(); ++i) {
        for (j = 0; j < (int)m_poligons[i].vertices.size(); j++)
            m_poligons[i].vertices[j] = vertices_used[m_poligons[i].vertices[j]];
    }
    m_hull->m_polygons = std::move(m_poligons);
    m_tempTrianglesStore.clear();
    delete[] vertices_used;
    return true;
}

int QuickHull::_getIndexJoinedTempTriangle(int indexBasedTriangle, int indexJoinedTriangle) const
{
    if (m_tempTrianglesStore[indexBasedTriangle].joined[0] == indexJoinedTriangle)
        return 0;
    else if (m_tempTrianglesStore[indexBasedTriangle].joined[1] == indexJoinedTriangle)
        return 1;
    else if (m_tempTrianglesStore[indexBasedTriangle].joined[2] == indexJoinedTriangle)
        return 2;
    return -1;
}

bool QuickHull::_isNull(const Vector3& v) const
{
    return ((std::fabs(v.x) < m_epsilon) && (std::fabs(v.y) < m_epsilon) && (std::fabs(v.z) < m_epsilon));
}

void QuickHull::_addTempTriangle(int indexVertex0, int indexVertex1, int indexVertex2, const Vector3& dir)
{
    m_tempTrianglesStore.resize(m_tempTrianglesStore.size() + 1);
    TempTriangle& tempTriangle = m_tempTrianglesStore[m_tempTrianglesStore.size() - 1];
    tempTriangle.vertices[0] = indexVertex0;
    tempTriangle.vertices[1] = indexVertex1;
    tempTriangle.vertices[2] = indexVertex2;
    tempTriangle.iteration = m_iteration;
    tempTriangle.dir = dir;
    tempTriangle.valid = true;
}

void QuickHull::_addTempTriangle(int indexVertex0, int indexVertex1, int indexVertex2)
{
    m_tempTrianglesStore.resize(m_tempTrianglesStore.size() + 1);
    TempTriangle& tempTriangle = m_tempTrianglesStore[m_tempTrianglesStore.size() - 1];
    tempTriangle.vertices[0] = indexVertex0;
    tempTriangle.vertices[1] = indexVertex1;
    tempTriangle.vertices[2] = indexVertex2;
    tempTriangle.iteration = m_iteration;
    Vector3 basedVertex = m_vertices[indexVertex0];
    tempTriangle.dir = cross(m_vertices[indexVertex1] - basedVertex, m_vertices[indexVertex2] - basedVertex);
    tempTriangle.valid = true;
    tempTriangle.joined[0] = -1;
    tempTriangle.joined[1] = -1;
    tempTriangle.joined[2] = -1;
}

bool QuickHull::_findBasedTetras(int* tetras)
{
    Vector3 dir, simplex[3];
    dir.set(1.0f, 0.0f, 0.0f);
    bool flag = false;
    int i, j, n, countVertices = m_hull->countVertices();
    float min, min1, set;
    for (j = 0; ; j++) {
        tetras[0] = 0;
        min = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min) {
                tetras[0] = i;
                min = set;
            }
        }
        dir = - dir;
        tetras[1] = 0;
        min1 = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min1) {
                tetras[1] = i;
                min1 = set;
            }
        }
        if (std::fabs(min + min1) < m_epsilon) {
            if (j == 0) {
                dir.set(0.0f, 1.0f, 0.0f);
                flag = true;
            } else {
                return false;
            }
        }
        break;
    }
    Vector3 v[3];
    v[0] = m_vertices[tetras[0]] - m_center;
    v[1] = m_vertices[tetras[1]] - m_center;
    GJK::nearestPoint(dir, v[0], v[1], simplex, n);
    if (_isNull(dir)) {
        dir = cross(m_vertices[tetras[1]] -  m_vertices[tetras[0]], Vector3(0.0f, 0.0f, 1.0f));
        tetras[2] = 0;
        min = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min) {
                tetras[2] = i;
                min = set;
            }
        }
        if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon) {
            dir = - dir;
            tetras[2] = 0;
            min = dot(m_vertices[0], dir);
            for (i = 1; i < countVertices; ++i) {
                set = dot(m_vertices[i], dir);
                if (set > min) {
                    tetras[2] = i;
                    min = set;
                }
            }
            if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon) {
                dir.set(0.0f, 0.0f, 1.0f);
                tetras[2] = 0;
                min = dot(m_vertices[0], dir);
                for (i = 1; i < countVertices; ++i) {
                    set = dot(m_vertices[i], dir);
                    if (set > min) {
                        tetras[2] = i;
                        min = set;
                    }
                }
                if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon) {
                    dir = - dir;
                    tetras[2] = 0;
                    min = dot(m_vertices[0], dir);
                    for (i = 1; i < countVertices; ++i) {
                        set = dot(m_vertices[i], dir);
                        if (set > min) {
                            tetras[2] = i;
                            min = set;
                        }
                    }
                    if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon)
                        return false;
                }
            }
        }
    } else {
        dir = - dir;
        tetras[2] = 0;
        min = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min) {
                tetras[2] = i;
                min = set;
            }
        }
    }
    tetras[3] = -1;
    v[2] = m_vertices[tetras[2]] - m_center;
    if ((GJK::nearestPoint(dir, v[0], v[1], v[2], simplex, n)) || (_isNull(dir))) {
        dir = cross(v[1] - v[0], v[2] - v[0]);
        tetras[3] = 0;
        min = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min) {
                tetras[3] = i;
                min = set;
            }
        }
        if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon) {
            dir = - dir;
            tetras[3] = 0;
            min = dot(m_vertices[0], dir);
            for (i = 1; i < countVertices; ++i) {
                set = dot(m_vertices[i], dir);
                if (set > min) {
                    tetras[3] = i;
                    min = set;
                }
            }
            if (std::fabs(min - dot(m_vertices[tetras[0]], dir)) < m_epsilon) {
                tetras[3] = -1;
                return true;
            }
        }
    } else {
        dir = - dir;
        tetras[3] = 0;
        min = dot(m_vertices[0], dir);
        for (i = 1; i < countVertices; ++i) {
            set = dot(m_vertices[i], dir);
            if (set > min) {
                tetras[3] = i;
                min = set;
            }
        }
    }
    return true;
}

bool QuickHull::_formedBasedTetras(int maxCountTriangles, int* tetras)
{
    if (tetras[3] < 0) {
        int countVertices = m_hull->countVertices();
        if (countVertices > 3) {
            int* usedVertices = new int[countVertices], i, countTempVertices = 0;
            Vector3* temp = new Vector3[countVertices];
            for (i = 0; i < countVertices; ++i)
                usedVertices[i] = -1;
            usedVertices[tetras[0]] = tetras[0];
            usedVertices[tetras[1]] = tetras[1];
            usedVertices[tetras[2]] = tetras[2];
            for (i = 0; i < countVertices; ++i) {
                if (usedVertices[i] >= 0) {
                    temp[usedVertices[countTempVertices]] = m_vertices[usedVertices[i]];
                    ++countTempVertices;
                }
            }
            m_hull->setCountVertices(countTempVertices);
            m_vertices = m_hull->m_local_vertices;
            for (i = 0; i < countTempVertices; ++i)
                m_vertices[i] = temp[i];
            delete[] temp;
            delete[] usedVertices;
        }
        m_poligons.resize(2);
        m_poligons[0].normal = cross((m_vertices[1] - m_vertices[0]), (m_vertices[2] - m_vertices[0]));
        m_poligons[0].normal.normalize();
        m_poligons[0].vertices.resize(3);
        m_poligons[0].vertices[0] = 0;
        m_poligons[0].vertices[1] = 1;
        m_poligons[0].vertices[2] = 2;
        m_poligons[1].normal = - m_poligons[0].normal;
        m_poligons[1].vertices.resize(3);
        m_poligons[1].vertices[0] = 0;
        m_poligons[1].vertices[1] = 2;
        m_poligons[1].vertices[2] = 1;
        m_hull->m_polygons = m_poligons;
        return true; // completed hull (1 triangle)
    }
    m_iteration = 0;
    m_tempTrianglesStore.clear();
    Vector3 v[4];
    v[0] = m_hull->m_local_vertices[tetras[0]];
    v[1] = m_hull->m_local_vertices[tetras[1]];
    v[2] = m_hull->m_local_vertices[tetras[2]];
    v[3] = m_hull->m_local_vertices[tetras[3]];

    Vector3 vec1 = v[1] - v[0];
    Vector3 vec2 = v[2] - v[0];
    Vector3 vec3 = v[3] - v[0];
    Vector3 dir1 = cross(vec1, vec2);
    if (dot(vec3, dir1) > 0.0f) {
        _addTempTriangle(tetras[0], tetras[2], tetras[1], (-dir1));
        _addTempTriangle(tetras[0], tetras[1], tetras[3], cross(vec1, vec3));
        _addTempTriangle(tetras[0], tetras[3], tetras[2], cross(vec3, vec2));
        _addTempTriangle(tetras[1], tetras[2], tetras[3], cross((v[2] - v[1]), (v[3] - v[1])));

        m_tempTrianglesStore[0].joined[0] = 2;
        m_tempTrianglesStore[0].joined[1] = 3;
        m_tempTrianglesStore[0].joined[2] = 1;
        m_tempTrianglesStore[1].joined[0] = 0;
        m_tempTrianglesStore[1].joined[1] = 3;
        m_tempTrianglesStore[1].joined[2] = 2;
        m_tempTrianglesStore[2].joined[0] = 1;
        m_tempTrianglesStore[2].joined[1] = 3;
        m_tempTrianglesStore[2].joined[2] = 0;
        m_tempTrianglesStore[3].joined[0] = 0;
        m_tempTrianglesStore[3].joined[1] = 2;
        m_tempTrianglesStore[3].joined[2] = 1;
    } else {
        _addTempTriangle(tetras[0], tetras[1], tetras[2], dir1);
        _addTempTriangle(tetras[0], tetras[3], tetras[1], cross(vec3, vec1));
        _addTempTriangle(tetras[0], tetras[2], tetras[3], cross(vec2, vec3));
        _addTempTriangle(tetras[1], tetras[3], tetras[2], cross((v[3] - v[1]), (v[2] - v[1])));

        m_tempTrianglesStore[0].joined[0] = 1;
        m_tempTrianglesStore[0].joined[1] = 3;
        m_tempTrianglesStore[0].joined[2] = 2;
        m_tempTrianglesStore[1].joined[0] = 2;
        m_tempTrianglesStore[1].joined[1] = 3;
        m_tempTrianglesStore[1].joined[2] = 0;
        m_tempTrianglesStore[2].joined[0] = 0;
        m_tempTrianglesStore[2].joined[1] = 3;
        m_tempTrianglesStore[2].joined[2] = 1;
        m_tempTrianglesStore[3].joined[0] = 1;
        m_tempTrianglesStore[3].joined[1] = 2;
        m_tempTrianglesStore[3].joined[2] = 0;
    }
    return false;
}

void QuickHull::_solveTempTriangle(int indexTriangle, int index, int newVertex)
{
    int joinedTriangle = m_tempTrianglesStore[indexTriangle].joined[index];
    if (joinedTriangle < 0)
        return;
    if (m_tempTrianglesStore[joinedTriangle].iteration != m_iteration) {
        int indexVertex0 = m_tempTrianglesStore[joinedTriangle].vertices[0];
        if (dot(m_vertices[newVertex] - m_vertices[indexVertex0], m_tempTrianglesStore[joinedTriangle].dir) > PE_EPSf)
            m_tempTrianglesStore[joinedTriangle].valid = false;
        else
            m_tempTrianglesStore[joinedTriangle].valid = true;
        m_tempTrianglesStore[joinedTriangle].iteration = m_iteration;
    }
    int findex = _getIndexJoinedTempTriangle(joinedTriangle, indexTriangle);
    m_tempTrianglesStore[joinedTriangle].joined[findex] = -1;
    if (!m_tempTrianglesStore[joinedTriangle].valid) {
        _solveTempTriangle(joinedTriangle, (findex + 2) % 3, newVertex);
        _solveTempTriangle(joinedTriangle, (findex + 1) % 3, newVertex);
    } else {
        int newTriangle = (int)m_tempTrianglesStore.size();
        if (index == 0) {
            _addTempTriangle(m_tempTrianglesStore[indexTriangle].vertices[0],
                             m_tempTrianglesStore[indexTriangle].vertices[1],
                             newVertex);
        } else if (index == 1) {
            _addTempTriangle(m_tempTrianglesStore[indexTriangle].vertices[1],
                             m_tempTrianglesStore[indexTriangle].vertices[2],
                             newVertex);
        } else if (index == 2) {
            _addTempTriangle(m_tempTrianglesStore[indexTriangle].vertices[2],
                             m_tempTrianglesStore[indexTriangle].vertices[0],
                             newVertex);
        }
        m_tempTrianglesStore[joinedTriangle].joined[findex] = newTriangle;
        m_tempTrianglesStore[newTriangle].joined[0] = joinedTriangle;
    }
}

bool QuickHull::_addVertexToConvex(int indexTriangle, int newVertex)
{
    m_tempTrianglesStore[indexTriangle].valid = false;
    m_tempTrianglesStore[indexTriangle].iteration = m_iteration;
    int firstTriangle = (int)m_tempTrianglesStore.size(), i, j;
    _solveTempTriangle(indexTriangle, 0, newVertex);
    _solveTempTriangle(indexTriangle, 1, newVertex);
    _solveTempTriangle(indexTriangle, 2, newVertex);
    for (i = firstTriangle; i < (int)m_tempTrianglesStore.size(); ++i) {
        for (j = i + 1; j < (int)m_tempTrianglesStore.size(); ++j) {
            if (m_tempTrianglesStore[i].vertices[0] == m_tempTrianglesStore[j].vertices[0]) {
                m_tempTrianglesStore[i].joined[2] = j;
                m_tempTrianglesStore[j].joined[2] = i;
            } else if (m_tempTrianglesStore[i].vertices[0] == m_tempTrianglesStore[j].vertices[1]) {
                m_tempTrianglesStore[i].joined[2] = j;
                m_tempTrianglesStore[j].joined[1] = i;
            } else if (m_tempTrianglesStore[i].vertices[1] == m_tempTrianglesStore[j].vertices[0]) {
                m_tempTrianglesStore[i].joined[1] = j;
                m_tempTrianglesStore[j].joined[2] = i;
            } else if (m_tempTrianglesStore[i].vertices[1] == m_tempTrianglesStore[j].vertices[1]) {
                m_tempTrianglesStore[i].joined[1] = j;
                m_tempTrianglesStore[j].joined[1] = i;
            }
        }
        if ((m_tempTrianglesStore[i].joined[1] < 0) || (m_tempTrianglesStore[i].joined[2] < 0)) {
            return false;
        }
    }
    return true;
}

void QuickHull::_trianglesToPoligons(Polygon& polygon,
                                     int indexTriangle, int index, int* usedVertices)
{
    int joinedTriangle = m_tempTrianglesStore[indexTriangle].joined[index], findex;
    if (polygon.normal.equalDir(m_tempTrianglesStore[joinedTriangle].dir)) {
        if (!m_tempTrianglesStore[joinedTriangle].valid)
            return;
        m_tempTrianglesStore[joinedTriangle].valid = false;
        findex = _getIndexJoinedTempTriangle(joinedTriangle, indexTriangle);
        _trianglesToPoligons(polygon, joinedTriangle, (findex + 1) % 3, usedVertices);
        _trianglesToPoligons(polygon, joinedTriangle, (findex + 2) % 3, usedVertices);
    } else {
        findex = m_tempTrianglesStore[indexTriangle].vertices[index];
        polygon.vertices.push_back(findex);
        usedVertices[findex] = findex;
    }
}

} // namespace PE
