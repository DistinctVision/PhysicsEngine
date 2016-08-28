CONFIG += c++11

SOURCES += \
    $$PWD/Physics/Bodies/Body.cpp \
    $$PWD/Physics/CollisionDetected/CollisionDetected.cpp \
    $$PWD/Physics/CollisionDetected/EPA.cpp \
    $$PWD/Physics/CollisionDetected/GJK.cpp \
    $$PWD/Physics/Bodies/Material.cpp \
    $$PWD/Physics/Bodies/Shape.cpp \
    $$PWD/Physics/Bodies/Sphere.cpp \
    $$PWD/Physics/PhysicsWorld.cpp \
    $$PWD/Physics/Bodies/Capsule.cpp \
    $$PWD/Physics/Bodies/QuickHull.cpp \
    $$PWD/Physics/Bodies/Hull.cpp \
    $$PWD/Physics/Bodies/BoundsTrees.cpp \
    $$PWD/Physics/Dynamic/Solver.cpp \
    $$PWD/Physics/Dynamic/ShockPropagationSolver.cpp \
    $$PWD/Physics/Dynamic/ContactsContainer.cpp \
    $$PWD/Physics/VectorMath/Vector2.cpp \
    $$PWD/Physics/VectorMath/Vector3.cpp
    $$PWD/Physics/VectorMath/Vector.cpp

HEADERS += \
    $$PWD/Physics/Settings.h \
    $$PWD/Physics/VectorMath/Vector2.h \
    $$PWD/Physics/VectorMath/Vector3.h \
    $$PWD/Physics/VectorMath/RotationMatrix.h \
    $$PWD/Physics/Bodies/Body.h \
    $$PWD/Physics/CollisionDetected/CollisionDetected.h \
    $$PWD/Physics/CollisionDetected/EPA.h \
    $$PWD/Physics/CollisionDetected/GJK.h \
    $$PWD/Physics/Bodies/Material.h \
    $$PWD/Physics/Bodies/Shape.h \
    $$PWD/Physics/Bodies/Sphere.h \
    $$PWD/Physics/Bodies/Capsule.h \
    $$PWD/Physics/Bodies/QuickHull.h \
    $$PWD/Physics/Bodies/Hull.h \
    $$PWD/Physics/Bodies/BoundsTrees.h \
    $$PWD/Physics/Dynamic/Solver.h \
    $$PWD/Physics/Dynamic/ShockPropagationSolver.h \
    $$PWD/Physics/Dynamic/ContactsContainer.h \
    $$PWD/Physics/PhysicsWorld.h \
    $$PWD/Physics/Physics.h
