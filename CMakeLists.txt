cmake_minimum_required(VERSION 2.8)
project(LPA)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g -Wall -Wextra")
set(SOURCE_FILES
        search.cpp
        config.h
        environmentoptions.cpp
        ilogger.h
        mission.cpp
        searchresult.h
        xmllogger.cpp
        environmentoptions.h
        map.cpp
        mission.h
        xmllogger.h
        config.cpp
        gl_const.h
        map.h
        node.h
        tinyxml2.cpp
        tinyxml2.h
        LPA.cpp LPA.h)
add_executable(LPA ${SOURCE_FILES})