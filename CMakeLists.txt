cmake_minimum_required(VERSION 2.8)
project(D-lite)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(SOURCE_FILES
        search.cpp
        config.h
        environmentoptions.cpp
        ilogger.h
        mission.cpp
        searchresult.h
        xmllogger.cpp
        environmentoptions.h
        dsearch.cpp
        map.cpp
        mission.h
        xmllogger.h
        config.cpp
        gl_const.h
        dsearch.h
        map.h
        node.h
        tinyxml2.cpp
        tinyxml2.h
        )
add_executable(D-lite ${SOURCE_FILES})