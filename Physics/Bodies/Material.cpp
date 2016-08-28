#include "Material.h"

namespace PE {

Material::Material()
{
    set(PE_default_e, PE_default_mu);
}

Material::Material(float e, float mu)
{
    set(e, mu);
}

float Material::e() const
{
    return m_e;
}

void Material::setE(float e)
{
    m_e = e;
}

float Material::mu() const
{
    return m_mu;
}

void Material::setMu(float mu)
{
    m_mu = mu;
}

void Material::set(float e, float mu)
{
    setE(e);
    setMu(mu);
}

Material Material::mixed(const Material& m) const
{
    return Material(m_e * m.e(), (m_mu + m.mu()) * 0.5f);
}

}
