#ifndef CITYGMLCORE_H
#define CITYGMLCORE_H


#include "building.h"

#include "TriangleMesh.h"
#include "osmnode.h"
#include "osmrelation.h"
#include "osmway.h"

#include <vector>
#include <string>

#include <geotiff.h>

typedef unsigned int uint;
class CityGMLCore
{
public:
    CityGMLCore();
    CityGMLCore(std::string, std::string, std::string, std::string lidarFilename, std::string);

    int buildLevel(uint level = 0);

    /**
     * @brief loadOSM method for erading and parsing an OSM formatted XML file (.osm)
     * @param filename the filename (path + actual name) of the file
     * @param nodes reference to the list (map) that will contain the loaded OSMNodes
     * @param ways reference to the list (map) that will contain the loaded OSMWays
     * @param relations reference to the list (map) that will contain the loaded OSMRelations
     * @return error code, equals 0 if no error is detected.
     */
    int loadOSM(std::string filename, std::map<std::string, std::shared_ptr<OSMNode> > &nodes,
                std::map<std::string, std::shared_ptr<OSMWay> > &ways, std::map<std::string, std::shared_ptr<OSMRelation> > &relations);

    int readLiDAR(std::string filename);

    /**
     * @brief fixWay removes duplicated nodes, spikes and other errors from a polyline defined by an OSMWay.
     * @param way the way to be fixed.
     */
    void fixWay(std::shared_ptr<OSMWay > way);

    /**
     * @brief removeOSMWayNodesFromBuildings removes OSMNodes from a list of OSMWays if they are contained in a building. This assumes
     * that inner and outer boundaries of buildings have been previously paired.
     * @param ways reference to the list containing the OSMWays to be modified.
     * @param polygons reference to the list containing the boundaries of buildings.
     */
    void removeOSMWayNodesFromBuildings(std::vector<std::string> &ways, std::vector<std::vector<std::vector<std::shared_ptr<Point> > > > polygons);


    /**
     * @brief removeOSMWayNodesFromBuildings removes OSMNodes from a list of OSMWays if they are contained in a building.
     * @param ways reference to the list containing the OSMWays to be modified.
     */
    void removeOSMWayNodesFromBuildings(std::vector<std::string> &ways);

    /**
     * @brief removeLinePointsInsideBuildings removes points from a list of streets points if they are inside a building.
     * @param streets the streets
     */
    void removeLinePointsInsideBuildings(std::vector<VertexList> &streets);

    std::vector<std::pair<uint, uint> > removeSegmentsIntersectingBuildings(std::vector<VertexList> &lines);

    bool arePolygonsAdjacent(VertexList, VertexList) const;

    VertexList createDTMVertices(double scale_factor = 1.0, Point origin = Point(0,0,0));

    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > extractBuildingsHeights(bool is_dtm_tiff, double scale_factor, Point origin);
    std::vector<std::vector<double> > extractBuildingsHeightsFromLidar(/*bool isDTM, */double scale_factor, Point origin);

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
    bool isPointOutsideBounds(const std::shared_ptr<Point> p);

private:
    std::string osmFilename, dtmFilename, dsmFilename;
    //Lists of entities loaded from the OSM file
    std::map<std::string, std::shared_ptr<OSMNode> > nodes;
    std::map<std::string, std::shared_ptr<OSMWay> > arcs;
    std::map<std::string, std::shared_ptr<OSMRelation> > relations;

    std::vector<std::string> buildingsArcs;
    std::vector<std::string> streetsArcs;
    std::vector<std::shared_ptr<Building> > buildings;
    std::vector<std::vector<std::shared_ptr<Point> > >  bounds;
    //std::vector<std::vector<std::pair<uint, uint> > > pixelToBuildingAssociation;

    std::shared_ptr<TriangleMesh> meshes[5];

    std::shared_ptr<GeoTiff> dtm, dsm;
    std::vector<std::shared_ptr<Point> > lidarDTMPoints, lidarDSMPoints;

    bool osmIsLoaded, dtmIsLoaded, dsmIsLoaded;


    int buildLevel0();
    int buildLevel1();

    void setLevel0(std::string meshFileName, std::string annotationFileName);
    void setLevel1(std::string meshFileName, std::string annotationFileName);
};

#endif // CITYGMLCORE_H
