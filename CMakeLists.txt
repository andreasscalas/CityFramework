cmake_minimum_required(VERSION 3.5)

project(CityDigitalTwin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
find_package(OpenMP)
find_library(SHAPELIB shp Shp)
find_path(SHAPELIB_H shapelib)
find_package( GDAL )
find_package(RapidJSON)
find_path(RAPIDJSON_H rapidjson-master/include/)
find_path(TINYXML_H tinyxml2 )
find_library(TINYXML_LIB libtinyxml2)
find_path(DATA_STRUCTURES_H DataStructures )
find_library(DATA_STRUCTURES_LIB libDataStructures)
find_package(PROJ)
find_package(libLAS REQUIRED)


if ( NOT GDAL_FOUND )
  message(SEND_ERROR "The GDAL library is required")
endif()

set( MAIN_FOLDER ${CMAKE_SOURCE_DIR})
set( TRIANGLE_MESH ${MAIN_FOLDER}/triangle_mesh/ )
set( TRIANGLE ${MAIN_FOLDER}/triangle/ )
set( KDTree ${MAIN_FOLDER}/KDTree/)
set( NANOFLANN ${MAIN_FOLDER}/nanoflann/include)

file(GLOB IO_srcs "${MAIN_FOLDER}/include/*.h*" "${MAIN_FOLDER}/IO/*.c*")
file(GLOB Srcs "${MAIN_FOLDER}/src/*.c*")
file(GLOB Hdrs "${MAIN_FOLDER}/include/*.h")
add_subdirectory(${TRIANGLE_MESH})
add_subdirectory(${TRIANGLE})
set(AS_LIBRARY true)
add_subdirectory(${KDTree})

#Adding Extruder executable and dependencies
list(REMOVE_ITEM Srcs
    "${MAIN_FOLDER}/src/test_in_polygon.cpp"
    "${MAIN_FOLDER}/src/visualise_city.cpp"
    "${MAIN_FOLDER}/src/test.cpp"
    "${MAIN_FOLDER}/src/compute_slopes.cpp"
    "${MAIN_FOLDER}/src/create_itinerary.cpp"
)


include_directories(Extruder
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${GDAL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${TRIANGLE}
    ${TRIANGLE_MESH}/include
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
)

add_executable(Extruder ${IO_srcs} ${Srcs} ${Hdrs} )
target_link_libraries(Extruder ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})


#Adding TestInPolygon executable and dependencies
file(GLOB TestInPolygonSrcs "${MAIN_FOLDER}/src/*.c*")
list(REMOVE_ITEM TestInPolygonSrcs
    "${MAIN_FOLDER}/src/main.cpp"
    "${MAIN_FOLDER}/src/visualise_city.cpp"
    "${MAIN_FOLDER}/src/test.cpp"
    "${MAIN_FOLDER}/src/compute_slopes.cpp"
    "${MAIN_FOLDER}/src/create_itinerary.cpp"
)
add_executable(TestInPolygon ${IO_srcs} ${TestInPolygonSrcs} ${Hdrs} )
target_link_libraries(TestInPolygon ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})
#target_include_directories(TestInPolygon PRIVATE
#    ${MAIN_FOLDER}/include/
#    ${MAIN_FOLDER}/prepair/
#    ${SHAPELIB_H}
#    ${GDAL_INCLUDE_DIR}
#    ${Boost_INCLUDE_DIR}
#    ${TRIANGLE}
#    ${TRIANGLE_MESH}/include
#    ${RAPIDJSON_H}
#    ${PROJ_INCLUDE_DIRS}
#    ${NANOFLANN}
#    ${KDTree}
#    ${DATA_STRUCTURES_H}
#)

#Adding Test executable and dependencies
file(GLOB TestSrcs "${MAIN_FOLDER}/src/*.c*")
list(REMOVE_ITEM TestSrcs
    "${MAIN_FOLDER}/src/main.cpp"
    "${MAIN_FOLDER}/src/test_in_polygon.cpp"
    "${MAIN_FOLDER}/src/visualise_city.cpp"
    "${MAIN_FOLDER}/src/compute_slopes.cpp"
    "${MAIN_FOLDER}/src/create_itinerary.cpp"
)
add_executable(Test ${IO_srcs} ${TestSrcs} ${Hdrs} )
target_link_libraries(Test ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})
#target_include_directories(Test PRIVATE
#    ${MAIN_FOLDER}/include/
#    ${MAIN_FOLDER}/prepair/
#    ${SHAPELIB_H}
#    ${GDAL_INCLUDE_DIR}
#    ${Boost_INCLUDE_DIR}
#    ${TRIANGLE}
#    ${TRIANGLE_MESH}/include
#    ${RAPIDJSON_H}
#    ${PROJ_INCLUDE_DIRS}
#    ${NANOFLANN}
#    ${KDTree}
#    ${DATA_STRUCTURES_H}
#)

#Adding ComputeSlopes executable and dependencies
file(GLOB ComputeSlopesSrcs "${MAIN_FOLDER}/src/*.c*")
list(REMOVE_ITEM ComputeSlopesSrcs
    "${MAIN_FOLDER}/src/main.cpp"
    "${MAIN_FOLDER}/src/test.cpp"
    "${MAIN_FOLDER}/src/test_in_polygon.cpp"
    "${MAIN_FOLDER}/src/visualise_city.cpp"
    "${MAIN_FOLDER}/src/create_itinerary.cpp"
)
add_executable(ComputeSlopes ${IO_srcs} ${ComputeSlopesSrcs} ${Hdrs} )
target_link_libraries(ComputeSlopes ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})

#Adding CreateItinerary executable and dependencies
file(GLOB CreateItinerarySrcs "${MAIN_FOLDER}/src/*.c*")
list(REMOVE_ITEM CreateItinerarySrcs
    "${MAIN_FOLDER}/src/main.cpp"
    "${MAIN_FOLDER}/src/test.cpp"
    "${MAIN_FOLDER}/src/test_in_polygon.cpp"
    "${MAIN_FOLDER}/src/visualise_city.cpp"
    "${MAIN_FOLDER}/src/compute_slopes.cpp"
)
add_executable(CreateItinerary ${IO_srcs} ${CreateItinerarySrcs} ${Hdrs} )
target_link_libraries(CreateItinerary ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})
#target_include_directories(ComputeSlopes PRIVATE
#    ${MAIN_FOLDER}/include/
#    ${MAIN_FOLDER}/prepair/
#    ${SHAPELIB_H}
#    ${GDAL_INCLUDE_DIR}
#    ${Boost_INCLUDE_DIR}
#    ${TRIANGLE}
#    ${TRIANGLE_MESH}/include
#    ${RAPIDJSON_H}
#    ${PROJ_INCLUDE_DIRS}
#    ${NANOFLANN}
#    ${KDTree}
#    ${DATA_STRUCTURES_H}
#)

#Adding VisualiseCity executable and dependencies

find_package(VTK COMPONENTS
    vtkCommonCore
    vtkFiltersCore
    vtkInfovisCore
    vtkInteractionStyle
    vtkGUISupportQt
    vtkIOLegacy
)
if(VTK_FOUND)

    include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
    file(GLOB VisualiseCitySrcs "${MAIN_FOLDER}/src/*.c*")
    list(REMOVE_ITEM VisualiseCitySrcs
        "${MAIN_FOLDER}/src/main.cpp"
        "${MAIN_FOLDER}/src/test_in_polygon.cpp"
        "${MAIN_FOLDER}/src/test.cpp"
        "${MAIN_FOLDER}/src/compute_slopes.cpp"
        "${MAIN_FOLDER}/src/create_itinerary.cpp"
    )
    add_executable(VisualiseCity ${IO_srcs} ${VisualiseCitySrcs} ${Hdrs} )
    target_link_libraries(VisualiseCity ${SHAPELIB} TriangleMesh-lib Triangle-lib OpenMP::OpenMP_CXX ${GDAL_LIBRARY} ${TINYXML_LIB} ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} ${VTK_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB})
#    target_include_directories(VisualiseCity PRIVATE
#        ${MAIN_FOLDER}/include/
#        ${MAIN_FOLDER}/prepair/
#        ${SHAPELIB_H}
#        ${GDAL_INCLUDE_DIR}
#        ${Boost_INCLUDE_DIR}
#        ${TRIANGLE}
#        ${TRIANGLE_MESH}/include
#        ${RAPIDJSON_H}
#        ${TINYXML_H}
#        ${PROJ_INCLUDE_DIRS}
#        ${NANOFLANN}
#        ${KDTree}
#        ${DATA_STRUCTURES_H}
#    )
endif()
