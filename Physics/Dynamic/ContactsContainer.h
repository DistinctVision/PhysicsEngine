#ifndef PE_CONTACTS_CONTAINER_H
#define PE_CONTACTS_CONTAINER_H

#include <vector>
#include "../Settings.h"
#include "../VectorMath/Vector3.h"
#include "../Bodies/Body.h"
#include "../Bodies/Material.h"
#include "ContactTypes.h"

namespace PE {

class ContactsContainer:
        public ContactTypes
{
public:
    ContactsContainer();

    void setERP(float a, float b);
    void clearCMPoints();
    int addContactManifold(Body* bodyA, Body* bodyB, const Vector3& normal, const Material& material);
    void computeBinormalOnCM_notStatB(int nCM, int nPoint);
    void solveBinormalOnCM_statB(int nCM, int nPoint);
    void addContact(int nCM, ContactPoint& contactPoint, float xdt);
    void addContact_static(int nCM, const Vector3& contactPoint, float depth, float xdt);
    void addTempContactPoint(const Vector3& pointOnBodyA, const Vector3& pointOnBodyB, float depth);
    void addTempContactPoint_static(const Vector3& pointOnBodyA, float depth);
    bool optimizeContactPoints(int nCM, const Vector3& normal, float xdt);
    void compareContacts(int nCM);
    void deleteAllContacts();

    int countUsedPrevContacts() const;
    int countNotUsedPrevContacts() const;

    void updateCollisionGroups();

    std::size_t countContactManifolds() const;
    const ContactManifold& contactManifold(std::size_t index) const;

protected:
    ContactPoint m_tempCMPoints[PE_MaxCountTempCMPoint];
    int m_countCMPoints;
    ContactPoint* m_contacts[4];
    std::vector<ContactManifold> m_prev_contactManifolds;
    std::vector<ContactManifold> m_contactManifolds;

    float m_ERP_a;
    float m_ERP_b;

    int m_countUsedPrevContacts;
    int m_countNotUsedPrevContacts;
};

} // namespace PE

#endif // PE_CONTACTS_CONTAINER_H
