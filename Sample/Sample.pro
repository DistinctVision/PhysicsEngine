QT += widgets multimedia

TARGET = Sample
TEMPLATE = app

include (../Physics.pri)
include (QScrollEngine.pri)

SOURCES += main.cpp \
    Scene.cpp

HEADERS += \  
    Scene.h

INCLUDEPATH += ../

DEPENDPATH += ../

RESOURCES += \
    resources.qrc



