#ifndef PE_SHAPE_H
#define PE_SHAPE_H

#include <vector>
#include "../Settings.h"
#include "../VectorMath/Vector3.h"
#include "Material.h"

namespace PE {

enum class TypeShape
{
    Undefined,
    Hull,
    Sphere,
    Capsule
};

struct Bounds
{
    Vector3 min;
    Vector3 max;

    void init()
    {
        min.set(PE_MAXNUMBERf, PE_MAXNUMBERf, PE_MAXNUMBERf);
        max.set(PE_MINNUMBERf, PE_MINNUMBERf, PE_MINNUMBERf);
    }

    Vector3 getCenter() const
    {
        return (min + max) * 0.5f;
    }

    void merge(const Bounds& bounds)
    {
        min.minAxis(bounds.min);
        max.maxAxis(bounds.max);
    }

    bool collision(const Bounds& bounds) const
    {
        if (min.x <= bounds.max.x) if (max.x >= bounds.min.x)
            if (min. y<= bounds.max.y) if (max.y >= bounds.min.y)
                if (min.z <= bounds.max.z) if (max.z >= bounds.min.z)
                    return true;
        return false;
    }
};

class PhysicsWorld;
class Body;
class CollisionDetected;

class Shape
{
public:
    Shape();
    virtual ~Shape();

    TypeShape type() const;

    int countVertices() const;
    Bounds bounds() const;

    Body* body() const;
    void setBody(Body* newBody);

    Material material() const;
    void setMaterial(const Material& material);

    Bounds getLocalBounds() const;

    float support(Vector3& resultVertex, const Vector3& dir) const;
    float support_local(Vector3& resultVertex, const Vector3& dir) const;

    virtual Shape* copy() const = 0;
    virtual void update() = 0;

protected:
    friend class Body;
    friend class CollisionDetected;

    std::size_t m_index;
    TypeShape m_type;
    Material m_material;
    Body* m_body;
    Bounds m_bounds;
    std::vector<Vector3> m_local_vertices;
    std::vector<Vector3> m_global_vertices;
};

} // namespace PE

#endif // PE_SHAPE_H
