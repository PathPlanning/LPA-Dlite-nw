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
    mission.cpp \
    map.cpp \
    config.cpp \
    environmentoptions.cpp \
    LPA.cpp \
    search.cpp \
    isearch.cpp \
    astar.cpp

HEADERS += \
    tinyxml2.h \
    node.h \
    gl_const.h \
    xmllogger.h \
    mission.h \
    map.h \
    ilogger.h \
    config.h \
    searchresult.h \
    environmentoptions.h \
    LPA.h \
    isearch.h \
    astar.h
