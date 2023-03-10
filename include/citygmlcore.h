#ifndef CITYGMLCORE_H
#define CITYGMLCORE_H

#include "building.h"

#include "TriangleMesh.hpp"
#include "node.h"
#include "way.h"
#include "relation.h"

#include <vector>
#include <string>

#include <geotiff.h>
#include <root.h>

typedef unsigned int uint;
class CityGMLCore
{
public:
    CityGMLCore();
    CityGMLCore(std::string, std::string, std::string, std::string lidarFilename, std::string);

    int buildLevel(uint level = 0);

    int readLiDAR(std::string filename,
                  std::vector<std::string> allowedClasses,
                  bool checkInsideBounds,
                  std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > bounds,
                  std::vector<std::pair<std::shared_ptr<SemantisedTriangleMesh::Point>, std::string> > &classifiedPoints);

    /**
     * @brief fixWay removes duplicated nodes, spikes and other errors from a polyline defined by an OpenStreetMap::Way.
     * @param way the way to be fixed.
     */
    void fixWay(OpenStreetMap::Way* way);

    /**
     * @brief removeOpenStreetMap::WayNodesFromBuildings removes OpenStreetMap::Nodes from a list of OpenStreetMap::Ways if they are contained in a building. This assumes
     * that inner and outer boundaries of buildings have been previously paired.
     * @param ways reference to the list containing the OpenStreetMap::Ways to be modified.
     * @param polygons reference to the list containing the boundaries of buildings.
     */
    void removeOSMWayNodesFromBuildings(std::vector<std::string> &ways, std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > polygons);


    /**
     * @brief removeOSMWayNodesFromBuildings removes OpenStreetMap::Nodes from a list of OpenStreetMap::Ways if they are contained in a building.
     * @param ways reference to the list containing the OpenStreetMap::Ways to be modified.
     */
    void removeOSMWayNodesFromBuildings(std::vector<std::string> &ways);

    /**
     * @brief removeLinePointsInsideBuildings removes points from a list of streets points if they are inside a building.
     * @param streets the streets
     */
    void removeLinePointsInsideBuildings(std::vector<VertexList> &streets);

    std::vector<std::pair<uint, uint> > removeSegmentsIntersectingBuildings(std::vector<VertexList> &lines);

    bool arePolygonsAdjacent(VertexList, VertexList) const;

    VertexList createDTMVertices(double scale_factor = 1.0, SemantisedTriangleMesh::Point origin = SemantisedTriangleMesh::Point(0,0,0));

    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > extractBuildingsHeights(bool is_dtm_tiff, double scale_factor, SemantisedTriangleMesh::Point origin);
    std::vector<std::vector<double> > extractBuildingsHeightsFromLidar(/*bool isDTM, */double scale_factor, SemantisedTriangleMesh::Point origin);

    void associateElevations(std::vector<std::vector<VertexList> > triangulationHoles, std::vector<VertexList> streetsArcsPoints);

    //Getters and setters
    std::string getOsmFilename() const;
    void setOsmFilename(const std::string &value);
    std::string getDtmFilename() const;
    void setDtmFilename(const std::string &value);
    std::string getDsmFilename() const;
    void setDsmFilename(const std::string &value);

    void setLevel(uint level, std::string meshFileName, std::string annotationFileName);

    VertexList createLiDARDTMVertices();
    void removeAlreadyExistingPoints(std::vector<std::vector<VertexList> > boundaries, std::vector<VertexList > arcs);
    bool isPointOutsideBounds(const std::shared_ptr<SemantisedTriangleMesh::Point> p);

private:
    std::string osmFilename, dtmFilename, dsmFilename;
    //Lists of entities loaded from the OSM file
    OpenStreetMap::Root osm;

    std::vector<std::string> buildingsArcs;
    std::vector<std::string> streetsArcs;
    std::vector<std::shared_ptr<Building> > buildings;
    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > >  bounds;
    //std::vector<std::vector<std::pair<uint, uint> > > pixelToBuildingAssociation;

    std::map<OpenStreetMap::Point*, std::shared_ptr<SemantisedTriangleMesh::Vertex> > osmPointToMeshPointMap;
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > lidarDTMPoints, lidarDSMPoints;

    std::shared_ptr<SemantisedTriangleMesh::TriangleMesh> meshes[5];

    std::shared_ptr<GeoTiff> dtm, dsm;

    bool osmIsLoaded, dtmIsLoaded, dsmIsLoaded;


    int buildLevel0();
    int buildLevel1();

    void setLevel0(std::string meshFileName, std::string annotationFileName);
    void setLevel1(std::string meshFileName, std::string annotationFileName);

};

#endif // CITYGMLCORE_H
