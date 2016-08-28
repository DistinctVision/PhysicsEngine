#ifndef PE_CONTACT_H
#define PE_CONTACT_H

#include "../VectorMath/Vector3.h"
#include "../Settings.h"
#include "../Bodies/Body.h"

namespace PE {

class ContactTypes
{
public:
    struct ContactPoint
	{
        Vector3 pointOnBodyA;
        Vector3 pointOnBodyB;
        Vector3 point;
		float depth;
    };

    struct R_ContactPoint
	{
        Vector3 r;
        Vector3 rn;
        Vector3 rb;
    };

    struct InfoPointOnCM
	{
        Vector3 binormal;
		float depthA;
		float depthB;
		float impulse;
		float impulseFriction;
		float pseudoImpulse;
		float kNormal;
		float kBinormal;
		float kPseudo;
    };

    struct ContactManifold
	{
		bool solved;
        Body* bodyA;
        Body* bodyB;
        Vector3 normal;
        InfoPointOnCM infoPoint[PE_MaxCountContactManifoldPoints];
        R_ContactPoint pointA[PE_MaxCountContactManifoldPoints];
        R_ContactPoint pointB[PE_MaxCountContactManifoldPoints];
		bool notStatB;
		float e;
		float mu;
		int countPoints;

        Vector3 getPointA(int i) const
		{ 
            return (pointA[i].r + bodyA->position());
        }

        Vector3 getPointB(int i) const
		{ 
            return (pointB[i].r + bodyB->position());
        }

        float getPointDepth(int i) const
		{
			return infoPoint[i].depthA;
        }
    };
};

} // namespace PE

#endif // PE_CONTACT_H
