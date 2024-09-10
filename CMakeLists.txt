cmake_minimum_required(VERSION 3.15)

project(compressed-meshlet-supplemental VERSION 0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CXX ON)
enable_language(CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

add_executable(optimal-strips)
target_sources(optimal-strips
    PRIVATE

    include/OptimalStrips.h
    include/MILP.h

    src/demo.cpp
    src/OptimalStrips.cpp
    src/MILP.cpp
)
target_include_directories(optimal-strips
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

option(FORCE_SCIP "Force the use of SCIP instead of Gurobi" OFF)

find_package(GUROBI QUIET)
if(GUROBI_FOUND AND NOT FORCE_SCIP)
    message(STATUS "Found Gurobi at ${GUROBI_INCLUDE_DIRS}")

    target_sources(optimal-strips
        PRIVATE
        include/Gurobi.h
        src/Gurobi.cpp
    )
    target_include_directories(optimal-strips
        PUBLIC
        ${GUROBI_INCLUDE_DIRS}
    )
    target_link_libraries(optimal-strips
        PRIVATE
        ${GUROBI_LIBRARY}
        optimized ${GUROBI_CXX_LIBRARY}
        debug ${GUROBI_CXX_DEBUG_LIBRARY}
    )
    target_compile_definitions(optimal-strips PRIVATE HAS_GUROBI)
else()
    find_package(SCIP REQUIRED)
    message(STATUS "Found SCIP at ${SCIP_INCLUDE_DIRS}")

    target_sources(optimal-strips
        PRIVATE
        include/SCIP.h
        src/SCIP.cpp
    )
    target_include_directories(optimal-strips
        PUBLIC
        ${SCIP_INCLUDE_DIRS}
    )
    target_link_libraries(optimal-strips
        PRIVATE
        ${SCIP_LIBRARIES}
    )
endif()
