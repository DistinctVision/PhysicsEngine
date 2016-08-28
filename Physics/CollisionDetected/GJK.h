#ifndef PE_GJK_H
#define PE_GJK_H

#include "../VectorMath/Vector2.h"
#include "../VectorMath/Vector3.h"
#include "../Bodies/Shape.h"

namespace PE {

class GJK
{
public:
    static const float EPS;

public:
    const Vector3* getSimplex() const;
    int getNSimplex() const;

    bool compute(Vector3& resultDir, Shape* shapeA, Shape* shapeB);

public:
    static float sign(float a);
    static bool isNull(const Vector2& v);
    static bool isNull(const Vector3& v);

    static bool nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2,
                             Vector3* simplex, int& n);
    static bool nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2, const Vector3& point3,
                             Vector3* simplex, int& n);
    static bool nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2,
                             const Vector3& point3, const Vector3& point4,
                             Vector3* simplex, int& n);

    static float support(Vector3& supportVertex, const Shape* shapeA, const Shape* shapeB, const Vector3& dir);
private:
    Vector3 m_simplex[4];
    Vector3 m_simplex_temp[4];
    int m_nsimplex;
    float m_sM;
    float m_temp;
    Vector3 m_supportPoint;


private:
    static void _nearestPoint_edge(Vector3& result, const Vector3& point1, const Vector3& point2,
                                   Vector3& vec12, Vector3* simplex, int& n);
    static void _nearestPoint_2edges(Vector3& result, const Vector3& point1, const Vector3& point2,
                                     const Vector3& point3, Vector3& vec12, Vector3& vec23,
                                     Vector3* simplex, int& n);
    static void _nearestPoint_triangle(Vector3& result, const Vector3& point1, const Vector3& point2, const Vector3& point3,
                                       Vector3& vec12, Vector3& vec23, Vector3& vec31, Vector3& dir,
                                       Vector3* simplex, int& n);
    static void _nearestPoint_2triangles(Vector3& result,
                                         const Vector3& point1, const Vector3& point2,
                                         const Vector3& point3, const Vector3& point4,
                                         Vector3& vec12, Vector3& vec23, Vector3& vec31, Vector3& vec24,
                                         Vector3& vec43, Vector3& dir1, Vector3& dir2,
                                         Vector3* simplex, int& n);
    static void _nearestPoint_2triangles(Vector3& result, const Vector3& point1, const Vector3& point2,
                                         const Vector3& point3, const Vector3& point4,
                                         Vector3& vec12, Vector3& vec23, Vector3& vec24, Vector3& dir1, Vector3& dir2,
                                         Vector3* simplex, int& n);
    static void _nearestPoint_3triangles(Vector3& result, const Vector3& point1, const Vector3& point2,
                                         const Vector3& point3, const Vector3& point4,
                                         Vector3& vec12, Vector3& vec23, Vector3& vec31, Vector3& vec24,
                                         Vector3& vec43, Vector3& vec14, Vector3& dir1, Vector3& dir2, Vector3& dir3,
                                         Vector3* simplex, int& n);
};

}

#endif // PE_GJK_H
