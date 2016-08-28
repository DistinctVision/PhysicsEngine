#ifndef PE_HULL_H
#define PE_HULL_H

#include <vector>
#include "Shape.h"

namespace PE {

struct Polygon
{
    std::vector<int> vertices;
    Vector3 normal;
};

class QuickHull;

class Hull:
        public Shape
{
public:
    Hull();

    void setLocalVertex(int index, float x, float y, float z);
    void setLocalVertex(int index, const Vector3& localVertex);

    void setCountVertices(int count);

    void setVertices(const std::vector<Vector3>& vertices);

    void moveLocalVertices(const Vector3& v);
    void scaleLocalVertices(const Vector3& scale);

    Vector3 localVertex(int index) const;
    Vector3 vertex(int index) const;

    int countPolygons() const;
    Polygon& polygon(int index);
    const Polygon& polygon(int index) const;

    bool formedHull(float epsinon = PE_DefaultEpsilonConvexHull,
                    int maxCountVerticesOnPoligon = PE_DefaultMaxCountVerticesOnPoligonsOnConvexHull,
                    int maxCountPoligons = PE_DefaultMaxCountPoligonsOnConvexHull);

    Shape* copy() const override;

    void update() override;

    static Hull* creeateCube(const Vector3& scale = Vector3(1.0f, 1.0f, 1.0f));

private:
    friend class QuickHull;

    std::vector<Polygon> m_polygons;
};

} // namespace PE

#endif // PE_HULL_H
