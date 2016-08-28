#ifndef PE_EPA_H
#define PE_EPA_H

#include <vector>
#include "../VectorMath/Vector3.h"
#include "../Settings.h"
#include "../Bodies/Body.h"

namespace PE {

class EPA
{

public:
    float minDepth;

    void compute(Vector3& result, Shape* shapeA, Shape* shapeB, const Vector3* const simplex, int nsimplex);

private:
    struct Triangle
    {
        int vertex[3];
        bool valid;
        int iteration;
        int joinedTriangle[3];
        Vector3 dir;
        float dis;
    };

    std::vector<Triangle> m_triangles;
    std::vector<Triangle*> m_candidateTriangleEPA;

    std::vector<Vector3> m_vertices;

    int m_iteration;

	
    int _addVertex(const Vector3& pos);

    void _checkValidTriangle(Triangle &triangle, const Vector3& newVertex);

    void _addTriangle(int ver1, int ver2, int ver3, const Vector3& dir);

    void _addTriangle(int ver1, int ver2, int ver3);

    void _setJoinedTriangles(Triangle& triangle, int tr1, int tr2, int tr3);

    int _getIndexJoinedTriangle(const Triangle& triangle, int tr);

    void _solveTriangle(int indexTriangle, int index, int indexVertex);

    bool _addVertexToConvex(int indexTriangle, int indexVertex);

    int _findNearestFace();


    void _error(Vector3& result);
};

} // namespace PE

#endif // PE_EPA_H
