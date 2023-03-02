cmake_minimum_required(VERSION 3.5)

project(CityDigitalTwin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
find_library(SHAPELIB shp Shp)
find_path(SHAPELIB_H shapelib)
find_package(GDAL CONFIG REQUIRED)
find_package(RapidJSON)
find_path(RAPIDJSON_H rapidjson-master/include/)
find_path(DATA_STRUCTURES_H DataStructures )
find_library(DATA_STRUCTURES_LIB libDataStructures)
find_package(PROJ)
find_package(libLAS REQUIRED)
find_package(GeoJSONParser REQUIRED)
find_package(SemantisedTriangleMesh REQUIRED)
find_package(OSMManager REQUIRED)

if(NOT SemantisedTriangleMesh_FOUND)
    message(SEND_ERROR "The SemantisedTriangleMesh library is required")
endif()

if(NOT GDAL_FOUND )
  message(SEND_ERROR "The GDAL library is required")
endif()

set( MAIN_FOLDER ${CMAKE_SOURCE_DIR})
set( KDTree ${MAIN_FOLDER}/KDTree/)
add_subdirectory(${KDTree})
set( NANOFLANN ${MAIN_FOLDER}/nanoflann/include)

set(ExtruderHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(ExtruderSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/main.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(Extruder ${ExtruderSrcs} ${ExtruderHdrs} )
target_include_directories(Extruder PRIVATE
    ${MAIN_FOLDER}/include/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)

target_link_libraries(Extruder
    ${SHAPELIB}
    GDAL::GDAL
    ${libLAS_LIBRARIES}
    ${PROJ_LIBRARIES}
    KDTree
    ${DATA_STRUCTURES_LIB}
    ${GeoJSONParser_LIBRARIES}
    ${SemantisedTriangleMesh_LIBRARIES}
    ${OSMManager_LIBRARIES})

#Adding TestInPolygon executable and dependencies

set(TestInPolygonHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(TestInPolygonSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/test_in_polygon.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(TestInPolygon ${TestInPolygonSrcs} ${TestInPolygonHdrs} )
target_include_directories(TestInPolygon PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)
target_link_libraries(TestInPolygon ${SHAPELIB} ${SemantisedTriangleMesh_LIBRARIES} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})

#Adding Test executable and dependencies

set(TestHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(TestSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/test.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(Test ${TestSrcs} ${TestHdrs} )

target_include_directories(Test PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)
target_link_libraries(Test ${SHAPELIB} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})



#Adding BuildLOD2Building executable and dependencies

set(BuildLOD2BuildingHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(BuildLOD2BuildingSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/buildlod2building.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(BuildLOD2Building ${BuildLOD2BuildingSrcs} ${BuildLOD2BuildingHdrs} )

target_include_directories(BuildLOD2Building PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)

target_link_libraries(BuildLOD2Building ${SHAPELIB} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})


#Adding ComputeSlopes executable and dependencies
set(ComputeSlopesHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(ComputeSlopesSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/compute_slopes.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)
#set(ComputeSlopesSrcs
#    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
#    ${MAIN_FOLDER}/src/compute_slopes.cpp
#    ${MAIN_FOLDER}/src/geotiff.cpp
#    ${MAIN_FOLDER}/src/utilities.cpp)
#set(ComputeSlopesHdrs
#    ${MAIN_FOLDER}/include/annotationfilemanager.h
#    ${MAIN_FOLDER}/include/geotiff.h
#    ${MAIN_FOLDER}/include/utilities.h)

add_executable(ComputeSlopes ${ComputeSlopesSrcs} ${ComputeSlopesHdrs} )
target_include_directories(ComputeSlopes PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)
target_link_libraries(ComputeSlopes ${SHAPELIB} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})
#target_link_libraries(ComputeSlopes GDAL::GDAL ${TINYXML_LIB} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB}  ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})

#Adding CreateItinerary executable and dependencies
set(CreateItineraryHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(CreateItinerarySrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/create_itinerary.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(CreateItinerary ${CreateItinerarySrcs} ${CreateItineraryHdrs} )
target_include_directories(CreateItinerary PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)
target_link_libraries(CreateItinerary ${SHAPELIB} ${SemantisedTriangleMesh_LIBRARIES} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})

set(TerrainTriangulationHdrs
    ${MAIN_FOLDER}/include/annotationfilemanager.h
    ${MAIN_FOLDER}/include/building.h
    ${MAIN_FOLDER}/include/buildingsgroup.h
    ${MAIN_FOLDER}/include/citygmlcore.h
    ${MAIN_FOLDER}/include/coordsconverter.h
    ${MAIN_FOLDER}/include/geotiff.h
    ${MAIN_FOLDER}/include/utilities.h
)

set(TerrainTriangulationSrcs
    ${MAIN_FOLDER}/src/annotationfilemanager.cpp
    ${MAIN_FOLDER}/src/building.cpp
    ${MAIN_FOLDER}/src/buildingsgroup.cpp
    ${MAIN_FOLDER}/src/citygmlcore.cpp
    ${MAIN_FOLDER}/src/coordsconverter.cpp
    ${MAIN_FOLDER}/src/geotiff.cpp
    ${MAIN_FOLDER}/src/triangulateTerrain.cpp
    ${MAIN_FOLDER}/src/utilities.cpp
)

add_executable(TerrainTriangulation ${TerrainTriangulationSrcs} ${TerrainTriangulationHdrs} )
target_include_directories(TerrainTriangulation PRIVATE
    ${MAIN_FOLDER}/include/
    ${MAIN_FOLDER}/prepair/
    ${SHAPELIB_H}
    ${Boost_INCLUDE_DIR}
    ${RAPIDJSON_H}
    ${PROJ_INCLUDE_DIRS}
    ${NANOFLANN}
    ${KDTree}
    ${DATA_STRUCTURES_H}
    ${GeoJSONParser_INCLUDE_DIRS}
    ${SemantisedTriangleMesh_INCLUDE_DIRS}
    ${OSMManager_INCLUDE_DIRS}
)
target_link_libraries(TerrainTriangulation ${SHAPELIB} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})

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

    set(VisualiseCityHdrs
        ${MAIN_FOLDER}/include/annotationfilemanager.h
        ${MAIN_FOLDER}/include/building.h
        ${MAIN_FOLDER}/include/buildingsgroup.h
        ${MAIN_FOLDER}/include/citygmlcore.h
        ${MAIN_FOLDER}/include/coordsconverter.h
        ${MAIN_FOLDER}/include/geotiff.h
        ${MAIN_FOLDER}/include/utilities.h
    )

    set(VisualiseCitySrcs
        ${MAIN_FOLDER}/src/annotationfilemanager.cpp
        ${MAIN_FOLDER}/src/building.cpp
        ${MAIN_FOLDER}/src/buildingsgroup.cpp
        ${MAIN_FOLDER}/src/citygmlcore.cpp
        ${MAIN_FOLDER}/src/coordsconverter.cpp
        ${MAIN_FOLDER}/src/geotiff.cpp
        ${MAIN_FOLDER}/src/visualise_city.cpp
        ${MAIN_FOLDER}/src/utilities.cpp
    )


    add_executable(VisualiseCity ${VisualiseCitySrcs} ${VisualiseCityHdrs} )
    target_include_directories(VisualiseCity PRIVATE
        ${MAIN_FOLDER}/include/
        ${MAIN_FOLDER}/prepair/
        ${SHAPELIB_H}
        ${Boost_INCLUDE_DIR}
        ${RAPIDJSON_H}
        ${PROJ_INCLUDE_DIRS}
        ${NANOFLANN}
        ${KDTree}
        ${DATA_STRUCTURES_H}
        ${GeoJSONParser_INCLUDE_DIRS}
        ${SemantisedTriangleMesh_INCLUDE_DIRS}
        ${OSMManager_INCLUDE_DIRS}
    )
    target_link_libraries(VisualiseCity ${SHAPELIB} ${SemantisedTriangleMesh_LIBRARIES} GDAL::GDAL ${libLAS_LIBRARIES} ${PROJ_LIBRARIES} ${VTK_LIBRARIES} KDTree ${DATA_STRUCTURES_LIB} ${GeoJSONParser_LIBRARIES} ${SemantisedTriangleMesh_LIBRARIES} ${OSMManager_LIBRARIES})
endif()
