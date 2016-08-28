#ifndef PE_SETTINGS_H
#define PE_SETTINGS_H

#include "VectorMath/Vector3.h"

#define PE_BLOCK_SIZE 200

#define PE_DefaultMaxCountPoligonsOnConvexHull 2000
#define PE_DefaultMaxCountVerticesOnPoligonsOnConvexHull 200
#define PE_DefaultEpsilonConvexHull 0.01f

#define PE_MaxCountTriangleEPA 3000
#define PE_MaxCountVerticesEPA 3000

#define PE_defaultCountChecksForCollisionGroup 30

#define PE_MAIN_DEPTH 0.01f
#define PE_MaxCountContactManifoldPoints 4
#define PE_MaxCountTempCMPoint 100
#define PE_DefaultSizeContactOnBody 2
#define PE_EPS_forContact 0.1f

#define PE_BodyInertia 1

#define PE_default_mass 1.0f
#if (PE_BodyInertia == 3)
#define PE_default_inertia PE::Vector3(1.0f, 1.0f, 1.0f)
#else
#define PE_default_inertia 1.0f
#endif
#define PE_default_e 0.0f
#define PE_default_mu 1.0f

#endif
