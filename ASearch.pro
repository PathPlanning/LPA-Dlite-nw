TARGET = Dlite
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11 -O2 -Wall -Wextra

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

SOURCES += \
    tinyxml2.cpp \
    xmllogger.cpp \
    isearch.cpp \
    mission.cpp \
    map.cpp \
    list.cpp \
    config.cpp \
    dsearch.cpp \
    environmentoptions.cpp

HEADERS += \
    tinyxml2.h \
    node.h \
    gl_const.h \
    xmllogger.h \
    isearch.h \
    mission.h \
    map.h \
    ilogger.h \
    list.h \
    config.h \
    searchresult.h \
    environmentoptions.h
