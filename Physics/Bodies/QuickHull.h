#ifndef PE_QUICKHULL_H
#define PE_QUICKHULL_H

#include <vector>
#include "../VectorMath/Vector3.h"
#include "Hull.h"

namespace PE {

class QuickHull
{
public:
    bool qHull(Hull* hull,
               float epsinon = PE_DefaultEpsilonConvexHull,
               int maxCountVerticesOnPoligon = PE_DefaultMaxCountVerticesOnPoligonsOnConvexHull,
               int maxCountPoligons = PE_DefaultMaxCountPoligonsOnConvexHull);

private:
    typedef struct TempTriangle
    {
        int vertices[3];
        Vector3 dir;
        int joined[3];
        int iteration;
        bool valid;
    } TempTriangle;

    Hull* m_hull;
    float m_epsilon;
    std::vector<Polygon> m_poligons;
    std::vector<Vector3> m_vertices;
    Vector3 m_center;
    std::vector<TempTriangle> m_tempTrianglesStore;
    int m_iteration;

    bool _isNull(const Vector3& v) const;

    void _addTempTriangle(int indexVertex0, int indexVertex1, int indexVertex2, const Vector3& dir);
    void _addTempTriangle(int indexVertex0, int indexVertex1, int indexVertex2);
    bool _findBasedTetras(int* tetras);
    bool _formedBasedTetras(int maxCountTriangles, int* tetra);
    void _solveTempTriangle(int indexTriangle, int index, int newVertex);
    bool _addVertexToConvex(int indexTriangle, int newVertex);
    int _getJoinedTempTriangle(int indexTriangle, int indexEdge);
    int _getIndexJoinedTempTriangle(int indexBasedTriangle, int indexJoinedTriangle) const;
    void _trianglesToPoligons(Polygon& polygon, int indexTriangle, int index, int* usedVertex);
};

} // namespace PE

#endif // PE_QUICKHULL_H
