QT += core
QT -= gui

TARGET = acasx
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += precompile_header
PRECOMPILED_HEADER = pch.h

TEMPLATE = app

SOURCES += \
    mdp.cpp \
    state_ctrl.cpp \
    mdpvi.cpp \
    state_unctrl.cpp \
    dtmc.cpp \
    dtmcvi.cpp \
    lookuptable.cpp \
    utils.cpp \
    constants.cpp \
    acasx_interface.cpp \
    lookupTableFilesGeneration.cpp

HEADERS += \
    mdp.h \
    state_ctrl.h \
    double2d.h \
    double3d.h \
    dtmc.h \
    dtmcvi.h \
    lookuptable.h \
    mdp.h \
    mdpvi.h \
    pch.h \
    state_ctrl.h \
    state_unctrl.h \
    utils.h \
    acasx_interface.h

DISTFILES += \
    README.md \
    acasx.pro \
    CMakeLists.txt \
    copyright.txt


