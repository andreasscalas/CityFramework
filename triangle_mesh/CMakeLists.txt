cmake_minimum_required(VERSION 2.8.8)

project(TriangleMesh)

if(NOT DEFINED TRIANGLE_MESH)
    set(TRIANGLE_MESH ${CMAKE_CURRENT_SOURCE_DIR})
endif()
if(NOT DEFINED TRIANGLE)
    add_definitions(-DTRIANGLE_HEADER="${TRIANGLE}shewchuck_triangle.h")
    set(TRIANGLE ${TRIANGLE_MESH}/triangle/ )
    add_subdirectory(${TRIANGLE})
endif()
set( Srcs
    ${TRIANGLE_MESH}/src/Vertex.cpp
    ${TRIANGLE_MESH}/src/Edge.cpp
    ${TRIANGLE_MESH}/src/Triangle.cpp
    ${TRIANGLE_MESH}/src/TriangleMesh.cpp
    ${TRIANGLE_MESH}/src/annotation.cpp
    ${TRIANGLE_MESH}/src/pointannotation.cpp
    ${TRIANGLE_MESH}/src/lineannotation.cpp
    ${TRIANGLE_MESH}/src/surfaceannotation.cpp
    ${TRIANGLE_MESH}/src/attribute.cpp
    ${TRIANGLE_MESH}/src/geometricattribute.cpp
    ${TRIANGLE_MESH}/src/semanticattribute.cpp
    ${TRIANGLE_MESH}/src/facet.cpp
)
set( Hdrs
    ${TRIANGLE_MESH}/include/Vertex.h
    ${TRIANGLE_MESH}/include/Point.h
    ${TRIANGLE_MESH}/include/Edge.h
    ${TRIANGLE_MESH}/include/Triangle.h
    ${TRIANGLE_MESH}/include/TriangleMesh.h
    ${TRIANGLE_MESH}/include/annotation.h
    ${TRIANGLE_MESH}/include/pointannotation.h
    ${TRIANGLE_MESH}/include/lineannotation.h
    ${TRIANGLE_MESH}/include/surfaceannotation.h
    ${TRIANGLE_MESH}/include/attribute.h
    ${TRIANGLE_MESH}/include/geometricattribute.h
    ${TRIANGLE_MESH}/include/semanticattribute.h
    ${TRIANGLE_MESH}/include/facet.h
    ${TRIANGLE_MESH}/include/quadtree.h
)

add_library(${PROJECT_NAME}-lib ${Srcs} ${Hdrs})
target_include_directories(${PROJECT_NAME}-lib PRIVATE ${TRIANGLE_MESH}/include/ ${TRIANGLE})
target_link_libraries(${PROJECT_NAME}-lib Triangle-lib)
add_executable(${PROJECT_NAME}-test ${TRIANGLE_MESH}/main.cpp ${Srcs} ${Hdrs})
target_include_directories(${PROJECT_NAME}-test PRIVATE ${TRIANGLE_MESH}/include/ ${TRIANGLE})
target_link_libraries(${PROJECT_NAME}-test Triangle-lib)
