#ifndef PE_MATERIAL_H
#define PE_MATERIAL_H

#include "../VectorMath/Vector3.h"
#include "../Settings.h"

namespace PE {

class Material
{
public:
    Material();
    Material(float e, float mu);

    float e() const;
    void setE(float e);

    float mu() const;
    void setMu(float mu);

    void set(float e, float mu);

    Material mixed(const Material& m) const;

private:
    float m_e;
    float m_mu;
};

} // namespace PE

#endif // PE_MATERIAL_H
