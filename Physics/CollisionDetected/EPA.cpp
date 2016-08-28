#include "EPA.h"
#include "GJK.h"

namespace PE {

void EPA::compute(Vector3& result, Shape* shapeA, Shape* shapeB, const Vector3* const simplex, int nsimplex)
{
    m_triangles.clear();
    m_vertices.clear();
    float sM;
    Vector3 temp;
    if (nsimplex == 3) {
        _addVertex(simplex[0]);
        _addVertex(simplex[1]);
        _addVertex(simplex[2]);
        Vector3 dir = cross(simplex[2] - simplex[0], simplex[1] - simplex[0]);
        dir.normalize();
        sM = GJK::support(temp, shapeA, shapeB, dir);
        if (sM < PE_EPSf) {
            result = dir;
            return;
        }
        _addTriangle(0, 1, 2, (-dir));
        _addVertex(temp);
        Vector3 vec1 = simplex[1] - simplex[0];
        Vector3 vec2 = simplex[2] - simplex[0];
        Vector3 vec3 = m_vertices[3] - simplex[0];

        _addTriangle(0, 3, 1 ,cross(vec3, vec1));
        _addTriangle(0, 2, 3, cross(vec2, vec3));
        _addTriangle(1, 3, 2, cross(m_vertices[3] - simplex[1], simplex[2] - simplex[1]));

        _setJoinedTriangles(m_triangles[0], 1, 3, 2);
        _setJoinedTriangles(m_triangles[1], 2, 3, 0);
        _setJoinedTriangles(m_triangles[2], 0, 3, 1);
        _setJoinedTriangles(m_triangles[3], 1, 2, 0);
    } else {
        _addVertex(simplex[0]);
        _addVertex(simplex[1]);
        _addVertex(simplex[2]);
        _addVertex(simplex[3]);

        Vector3 vec1 = simplex[1] - simplex[0];
        Vector3 vec2 = simplex[2] - simplex[0];
        Vector3 vec3 = simplex[3] - simplex[0];
        Vector3 dir1 = cross(vec1, vec2);
        if (dot(vec3, dir1) > 0.0f) {
            _addTriangle(0, 2, 1, (-dir1));
            _addTriangle(0, 1, 3, cross(vec1, vec3));
            _addTriangle(0, 3, 2, cross(vec3, vec2));
            _addTriangle(1, 2, 3, cross(m_vertices[2] - m_vertices[1], m_vertices[3] - m_vertices[1]));

            _setJoinedTriangles(m_triangles[0], 2, 3, 1);
            _setJoinedTriangles(m_triangles[1], 0, 3, 2);
            _setJoinedTriangles(m_triangles[2], 1, 3, 0);
            _setJoinedTriangles(m_triangles[3], 0, 2, 1);
        } else {
            _addTriangle(0, 1, 2, dir1);
            _addTriangle(0, 3, 1, cross(vec3, vec1));
            _addTriangle(0, 2, 3, cross(vec2, vec3));
            _addTriangle(1, 3, 2, cross(m_vertices[3] - m_vertices[1], m_vertices[2] - m_vertices[1]));

            _setJoinedTriangles(m_triangles[0], 1, 3, 2);
            _setJoinedTriangles(m_triangles[1], 2, 3, 0);
            _setJoinedTriangles(m_triangles[2], 0, 3, 1);
            _setJoinedTriangles(m_triangles[3], 1, 2, 0);
        }
    }

    int index;
    int trE;
    m_iteration =  1;
    for (;;) {
        trE = _findNearestFace();
        if (trE < 0) {
            //error("EPA - findNearestFace");
            return;
        }
        sM = GJK::support(temp, shapeA, shapeB, m_triangles[trE].dir);
        if ((sM - m_triangles[trE].dis) < PE_EPSf) {
            result = m_triangles[trE].dir;
            minDepth = m_triangles[trE].dis;
            return;
        } else {
            index = _addVertex(temp);
            if (!_addVertexToConvex(trE, index)) {
                //error("EPA");
                _error(result);
                return;
            }
        }
        ++m_iteration;
    }
}

void EPA::_error(Vector3& result)
{
    int trE = _findNearestFace();
    result = m_triangles[trE].dir;
    minDepth = m_triangles[trE].dis;
}

int EPA::_addVertex(const Vector3& pos)
{
    for (int i = 0; i < (int)m_vertices.size(); ++i)
        if (pos.equal(m_vertices[i]))
            return i;
    m_vertices.push_back(pos);
    return (int)(m_vertices.size() - 1);
}

void EPA::_checkValidTriangle(EPA::Triangle& triangle, const Vector3& newVertex)
{
    //triangle->iteration = m_iteration;
    if (dot(newVertex - m_vertices[triangle.vertex[0]], triangle.dir) > 0.0f)
        triangle.valid = false;
    else
        triangle.valid = true;
}

void EPA::_addTriangle(int ver1, int ver2, int ver3, const Vector3& dir)
{
	float l = dir.length();
    if (l > PE_EPSf) {
        Vector3 triangleNormal = dir / l;
        m_triangles.resize(m_triangles.size() + 1);
        Triangle& triangle = m_triangles[m_triangles.size() - 1];
        triangle.vertex[0] = ver1;
        triangle.vertex[1] = ver2;
        triangle.vertex[2] = ver3;
        triangle.dir = triangleNormal;
        Vector3 proj = triangleNormal * dot(m_vertices[ver1], triangleNormal);
        triangle.dis = dot(proj, triangleNormal);
        triangle.iteration = 0;
	}
}

void EPA::_addTriangle(int ver1, int ver2, int ver3)
{
    //if ((int)m_storeTriangles.size() == PE_MaxCountEPATriangles)
    //    return false;
    m_triangles.resize(m_triangles.size() + 1);
    Triangle& triangle = m_triangles[m_triangles.size() - 1];
    triangle.dir = cross(m_vertices[ver2] - m_vertices[ver1], m_vertices[ver3] - m_vertices[ver1]);
    float l = triangle.dir.length();
    //if (l > PE_EPSf) {
        triangle.dir /= l;
        triangle.vertex[2] = ver3;
        Vector3 proj = triangle.dir * (dot(m_vertices[ver1], triangle.dir));
        triangle.dis = dot(proj, triangle.dir);
        triangle.vertex[0] = ver1;
        triangle.vertex[1] = ver2;
        triangle.iteration = 0;
        triangle.joinedTriangle[1] = -1;
        triangle.joinedTriangle[2] = -1;
		//return true;
	//}
	//return false;
}

void EPA::_setJoinedTriangles(EPA::Triangle& triangle, int tr1, int tr2, int tr3)
{
	triangle.joinedTriangle[0] = tr1;
	triangle.joinedTriangle[1] = tr2;
	triangle.joinedTriangle[2] = tr3;
}

int EPA::_getIndexJoinedTriangle(const EPA::Triangle& triangle, int tr)
{
    if (tr == triangle.joinedTriangle[0])
        return 0;
    else if (tr == triangle.joinedTriangle[1])
        return 1;
    else if (tr == triangle.joinedTriangle[2])
        return 2;
	return -1;
}

void EPA::_solveTriangle(int indexTriangle, int index, int indexVertex)
{
    int joinedTriangle = m_triangles[indexTriangle].joinedTriangle[index];
    if (joinedTriangle < 0)
        return;
    m_triangles[indexTriangle].joinedTriangle[index] = -1;
    if (m_triangles[joinedTriangle].iteration != m_iteration)
        _checkValidTriangle(m_triangles[joinedTriangle], m_vertices[indexVertex]);
    int findex = _getIndexJoinedTriangle(m_triangles[joinedTriangle], indexTriangle);
    m_triangles[joinedTriangle].joinedTriangle[findex] = -1;
    if (!m_triangles[joinedTriangle].valid) {
        m_triangles[joinedTriangle].dis = PE_MAXNUMBERf + 0.1f;
        _solveTriangle(joinedTriangle, (findex + 2) % 3, indexVertex);
        _solveTriangle(joinedTriangle, (findex + 1) % 3, indexVertex);
    } else {
        if (index == 0)
            _addTriangle(m_triangles[indexTriangle].vertex[0], m_triangles[indexTriangle].vertex[1], indexVertex);
        else if (index == 1)
            _addTriangle(m_triangles[indexTriangle].vertex[1], m_triangles[indexTriangle].vertex[2], indexVertex);
        else if (index == 2)
            _addTriangle(m_triangles[indexTriangle].vertex[2], m_triangles[indexTriangle].vertex[0], indexVertex);
        else
			index = 0;
        m_triangles[joinedTriangle].joinedTriangle[findex] = (int)(m_triangles.size() - 1);
        m_triangles[m_triangles.size() - 1].joinedTriangle[0] = joinedTriangle;
	}
}

bool EPA::_addVertexToConvex(int indexTriangle, int indexVertex)
{
    m_triangles[indexTriangle].iteration = m_iteration;
    m_triangles[indexTriangle].dis = PE_MAXNUMBERf + 0.1f;
    int firstTriangle = (int)m_triangles.size(), i, j;
    _solveTriangle(indexTriangle, 0, indexVertex);
    _solveTriangle(indexTriangle, 1, indexVertex);
    _solveTriangle(indexTriangle, 2, indexVertex);
    for (i = firstTriangle; i < (int) m_triangles.size(); ++i) {
        for (j = i + 1; j < (int) m_triangles.size(); ++j) {
            if (m_triangles[i].vertex[0] == m_triangles[j].vertex[0]) {
                m_triangles[i].joinedTriangle[2] = j;
                m_triangles[j].joinedTriangle[2] = i;
            } else if (m_triangles[i].vertex[0] == m_triangles[j].vertex[1]) {
                m_triangles[i].joinedTriangle[2] = j;
                m_triangles[j].joinedTriangle[1] = i;
            } else if (m_triangles[i].vertex[1] == m_triangles[j].vertex[0]) {
                m_triangles[i].joinedTriangle[1] = j;
                m_triangles[j].joinedTriangle[2] = i;
            } else if (m_triangles[i].vertex[1] == m_triangles[j].vertex[1]) {
                m_triangles[i].joinedTriangle[1] = j;
                m_triangles[j].joinedTriangle[1] = i;
			}
		}
        if ((m_triangles[i].joinedTriangle[1] < 0) ||
                (m_triangles[i].joinedTriangle[2] < 0)) {
 			return false;
		}
	}
	return true;
}

int EPA::_findNearestFace()
{
    int result = -1;
    float minDis = PE_MAXNUMBERf;
    for (int i = 0; i < (int)m_triangles.size(); ++i) {
        if (m_triangles[i].dis < minDis) {
            minDis = m_triangles[i].dis;
            result = i;
		}
	}
	return result;
}

} // namespace PE

