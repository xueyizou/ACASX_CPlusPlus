QT += core
QT -= gui

TARGET = acasx
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    mdp.cpp \
    state_ctrl.cpp \
    mdpvi.cpp \
    state_unctrl.cpp \
    dtmc.cpp \
    dtmcvi.cpp

HEADERS += \
    mdp.h \
    state_ctrl.h \
    utils.hpp \
    mdpvi.hpp \
    state_unctrl.hpp \
    dtmc.hpp \
    double2d.hpp \
    dtmcvi.hpp

CONFIG += c++11

DISTFILES += \
    README.md

