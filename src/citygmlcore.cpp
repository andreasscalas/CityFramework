#include "citygmlcore.h"

#include "trianglehelper.hpp"
#include "utilities.h"
#include "surfaceannotation.hpp"
#include "lineannotation.hpp"
#include "pointannotation.hpp"
#include "semanticattribute.hpp"

#include <OSMManager-1.0/coordinatesconverter.h>

#include <cmath>
#include <cctype>
#include <algorithm>
#include <tuple>
#include <chrono>
#include <liblas/liblas.hpp>
#include <shapefil.h>
#include <KDTree.hpp>
#include <semanticsfilemanager.hpp>


using namespace std::chrono;
using namespace SemantisedTriangleMesh;
using namespace OpenStreetMap;
#define MATERA_EPSG_CODE 32633

VertexList pointToVertexList(PointList points)
{
    VertexList vertices;
    for(const auto& p : points)
        vertices.push_back(std::static_pointer_cast<Vertex>(p));
    return vertices;
}

PointList vertexToPointList(VertexList vertices)
{
    PointList points;
    for(const auto &v : vertices)
        points.push_back(std::static_pointer_cast<SemantisedTriangleMesh::Point>(v));
    return points;
}


CityGMLCore::CityGMLCore()
{
    buildingsArcs.clear();
    streetsArcs.clear();
    buildings.clear();
    bounds.clear();
    osmPointToMeshPointMap.clear();
    lidarDTMPoints.clear();
    lidarDSMPoints.clear();
    osmIsLoaded = false;
    osmFilename = "";
}

CityGMLCore::CityGMLCore(const std::string& osmFilename, const std::string& lidarFilename, const std::string& boundsFile)
{
    this->osmFilename = osmFilename;

    int retValue = osm.load(osmFilename);
    if(retValue == 0)
    {
        for(const auto &node : osm.getNodes())
        {
            auto p = node.second->getCoordinates();
            OpenStreetMap::CoordsConverter converter(MATERA_EPSG_CODE);
            double x, y;
            converter.convertToUTM(p->x, p->y, x,y);
            p->x = x;
            p->y = y;
        }

        osmIsLoaded = true;
    } else
        std::cerr << "Error loading OSM file" << std::endl;

    std::cout << "Loading DTM" << std::endl;

    Utilities::load_shapefile_shp(boundsFile, bounds);
    auto it = osm.getNodes().begin();
    while(it != osm.getNodes().end())
    {
        auto osmPoint = it->second->getCoordinates();
        auto p = std::make_shared<SemantisedTriangleMesh::Point>(osmPoint->x, osmPoint->y, 0);
        if(isPointOutsideBounds(p))
            it = osm.removeNode(it->first);
        else
            it++;
    }

    std::vector<std::string> allowedClasses = {"ground", "building"};
    std::vector<std::pair<std::shared_ptr<SemantisedTriangleMesh::Point>, std::string> > classifiedPoints;
    int retValue_ = readLiDAR(lidarFilename, allowedClasses, true, bounds, classifiedPoints);
    if(retValue_ == 0)
    {
        for(const auto &p : classifiedPoints)
            if(p.second.compare("ground") == 0)
                lidarDTMPoints.push_back(p.first);
            else
                lidarDSMPoints.push_back(p.first);
        std::cout << "LiDAR data loading ended!" << std::endl << std::flush;
    } else
        std::cout << "Error loading LiDAR file" << std::endl;

}

void CityGMLCore::fixWay(OpenStreetMap::Way *way)
{
    way->fixRepeatedNodes();
}

void CityGMLCore::removeOSMWayNodesFromBuildings(std::vector<std::string> &ways, std::vector<std::vector<PointList > > polygons)
{
    pointVec points_vector;
    std::map<uint, uint > pointPolygonLink;
    uint id = 0, i = 0;
    double meanDiagonal = 0;
    for(const auto &buildingPolygons : polygons)
    {
        for(const auto &p : buildingPolygons.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              0};
            points_vector.push_back(point);
            pointPolygonLink.insert(std::make_pair(id++, i));
        }
        PointList bb = Utilities::bbExtraction(polygons[i].at(0));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    meanDiagonal /= static_cast<double>(polygons.size());

    KDTree tree(points_vector);
    uint counter = 0;


    #pragma omp parallel for num_threads(31)
    for(auto lit = ways.begin(); lit != ways.end(); lit++)
    {
        auto nodes = osm.getWays().at(*lit)->getNodes();
        for(auto nit = nodes.begin(); nit != nodes.end(); nit++){
            PointList intersections;
            auto v = (*nit)->getCoordinates();
            std::vector<std::pair<size_t, double> > neighbors_distances;
            std::vector<uint> neighboringPolygons;
            point_t query_point = {v->x, v->y, 0};
            neighbors_distances.clear();
            std::vector<size_t> neighbors = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            for(const auto &n : neighbors){
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(n));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(n));
            }

            for(const auto& np : neighboringPolygons)
            {
                auto v = std::make_shared<SemantisedTriangleMesh::Point>();
                bool insideOuter = Utilities::isPointInsidePolygon(v, polygons.at(np).at(0));
                bool outsideInners = true;

                for(const auto& polygon : polygons.at(np))
                    outsideInners = outsideInners && !Utilities::isPointInsidePolygon(v, polygon);

                if(insideOuter && outsideInners)
                {
                    bool onBoundary = false;
                    for(auto pit = polygons.at(np).begin(); pit != polygons.at(np).end(); pit++)
                    {
                        for(uint m = 1; m < pit->size(); m++)
                        {
                            auto p1 = pit->at(m - 1);
                            auto p2 = pit->at(m);
                            if(Utilities::isPointInSegment(p1.get(), p2.get(), v.get()))
                            {
                                onBoundary = true;
                                break;
                            }
                        }
                    }

                    if(!onBoundary)
                    {
                        osm.getWays().at(*lit)->removeNode((*nit)->getId());
                        nodes.erase(nit);
                        v.reset();
                        break;
                    }
                }
            }
        }

        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / ways.size() << "%\r" << std::flush;
        }
    }
    std::cout << std::endl;
}

void CityGMLCore::removeLinePointsInsideBuildings(std::vector<VertexList> &streets)
{
    pointVec points_vector;
    std::map<uint, uint > pointBuildingLink;
    uint id = 0, i = 0;
    double meanDiagonal = 0;
    for(const auto &building : buildings)
    {
        const auto &boundaries = building->getOutlines();
        for(const auto &p : boundaries.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              0};
            points_vector.push_back(point);
            pointBuildingLink.insert(std::make_pair(id++, i));
        }
        auto bb = Utilities::bbExtraction(vertexToPointList(building->getOutlines().at(0)));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    meanDiagonal /= static_cast<double>(buildings.size());

    KDTree tree(points_vector);
    uint counter = 0;

    #pragma omp parallel for num_threads(31)
    for(auto iit = streets.begin(); iit != streets.end(); iit++)
    {
        for(size_t j = 0; j < iit->size(); j++){
            PointList intersections;
            auto v = iit->at(j);
            auto query_point = {v->getX(), v->getY(), 0.0};
            auto neighbors_indices = tree.neighborhood_indices(query_point, meanDiagonal / 2);
            std::vector<uint> neighboringPolygons;
            for(auto it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointBuildingLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointBuildingLink.at(*it));
            }
            for(uint k = 0; k < neighboringPolygons.size(); k++)
            {
                auto building_boundaries = buildings.at(neighboringPolygons.at(k))->getOutlines();
                if(Utilities::isPointInsidePolygon(v, vertexToPointList(building_boundaries.at(0))))
                {
                    bool inside = false;
                    for(uint l = 1; l < building_boundaries.size(); l++)
                    {
                        if(Utilities::isPointInsidePolygon(v, vertexToPointList(building_boundaries.at(l))))
                        {
                            inside = true;
                            break;
                        }
                    }
                    bool onBoundary = Utilities::checkOnBoundary(v, building_boundaries);

                    if(!inside && !onBoundary)
                    {
                        iit->erase(iit->begin() + j);
                        v.reset();
                        j--;
                        break;
                    }
                }
            }
        }

        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / streets.size() << "%\r" << std::flush;
        }
    }
    std::cout << std::endl;
}

std::vector<std::pair<uint, uint> > CityGMLCore::removeSegmentsIntersectingBuildings(std::vector<VertexList> &lines)
{
    std::vector<std::pair<uint, uint> > keptClippings;
    pointVec points_vector;
    std::map<uint, uint > pointBuildingLink;
    uint id = 0, i = 0, counter = 0;
    double meanDiagonal = 0;
    for(auto building : buildings)
    {
        auto boundaries = building->getOutlines();
        for(auto p : boundaries.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              0.0};
            points_vector.push_back(point);
            pointBuildingLink.insert(std::make_pair(id++, i));
        }
        auto bb = Utilities::bbExtraction(vertexToPointList(boundaries.at(0)));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    meanDiagonal /= static_cast<double>(buildings.size());

    KDTree tree(points_vector);

    SemantisedTriangleMesh::Point query1(0.342792852414897,0.0403798386114185,0);
    SemantisedTriangleMesh::Point query2(0.342341167004858,0.0394952027521175,0);
    for(uint i = 0; i < lines.size(); i++)
    {
        size_t deviation = 0;

        for(uint j = 1; j < lines[i].size(); j++){
            VertexList intersections;
            std::vector<uint> neighboringPolygons;
            auto v = lines.at(i).at(j);
            auto query_point = {v->getX(), v->getY(), 0.0};
            auto neighbors_distances = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            for(std::vector<size_t>::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointBuildingLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointBuildingLink.at(*it));
            }

            for(uint k = 0; k < neighboringPolygons.size(); k++)
            {
                auto building = buildings.at(neighboringPolygons.at(k));
                auto polygons = building->getOutlines();

                std::vector<std::pair<uint, uint> > modifiedBoundaries;
                for(uint l = 0; l < polygons.size(); l++)
                {
                    for(uint m = 0; m < polygons.at(l).size(); m++)
                    {
                        std::shared_ptr<Vertex> pout = std::make_shared<Vertex>();

                        if(Utilities::segmentsIntersection(lines.at(i).at(j - 1), lines.at(i).at(j), polygons.at(l).at(m), polygons.at(l).at((m + 1) % polygons.at(l).size()), *pout))
                        {
                            auto it = std::find_if(intersections.begin(), intersections.end(), [pout](std::shared_ptr<SemantisedTriangleMesh::Point> p) { return *p == *pout; });
                            if(it == intersections.end())
                            {
                                if(*pout != *polygons.at(l).at(m) && *pout != *polygons.at(l).at(m + 1))
                                    polygons.at(l).insert(polygons.at(l).begin() + m + 1, pout);
                                else if (*pout == *polygons.at(l).at(m))
                                {
                                    pout->setInfo(polygons.at(l).at(m)->getInfo());
                                    *polygons.at(l).at(m) = *pout;
                                }else
                                {
                                    pout->setInfo(polygons.at(l).at(m + 1)->getInfo());
                                    *polygons.at(l).at(m + 1) = *pout;
                                }
                                modifiedBoundaries.push_back(std::make_pair(l, m));
                                m++;
                                intersections.push_back(pout);
                            }
                        }
                    }

                }
                for(auto p : modifiedBoundaries)
                {
                    buildings.at(neighboringPolygons.at(k))->setOutlines(polygons); //Boh, ottimizzare?
                }
            }
            if(intersections.size() > 1)
            {

                std::sort(intersections.begin(), intersections.end(),
                          [lines, i, j](std::shared_ptr<SemantisedTriangleMesh::Point> p1, std::shared_ptr<SemantisedTriangleMesh::Point> p2)
                {
                    return ((*p1) - (*lines.at(i).at(j - 1))).norm() < ((*p2) - (*lines.at(i).at(j - 1))).norm();
                });

                if(intersections.size() % 2 != 0)
                {
                    //Da gestire caso in cui segmento traversa uno "spuntone" di poligono e poi "tocca" una punta (tecnicamente sono 3 punti).
                    //Al momento si ignora il caso, eliminando l'ultima intersezione e considerandolo come un caso con numero pari di intersezioni
                    intersections.pop_back();
                }

                VertexList remaining(lines.at(i).begin() + j, lines.at(i).end());
                remaining.insert(remaining.begin(), intersections.back());
                lines.at(i).erase(lines.at(i).begin() + j, lines.at(i).end());
                lines.at(i).push_back(intersections.at(0));
                intersections.erase(intersections.begin());
                std::vector<VertexList> newLines;
                for(uint l = 0; l < intersections.size() / 2; l++)
                {
                    VertexList exteriorSegment = {intersections.at(l * 2), intersections.at(l * 2 + 1)};
                    newLines.push_back(exteriorSegment);
                }
                newLines.push_back(remaining);
                lines.insert(lines.begin() + i + 1, newLines.begin(), newLines.end());
                keptClippings.push_back(std::make_pair(i, i + newLines.size()));
                i += newLines.size();

            }

        }

        counter++;
        std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
    }

    std::cout << std::endl;
    return keptClippings;
}

bool CityGMLCore::arePolygonsAdjacent(VertexList p1, VertexList p2) const
{

    //std::reverse(p2.begin(), p2.end());
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > poly1(p1.begin(), p1.end());
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > poly2(p2.begin(), p2.end());
    if(Utilities::isPolygonClockwise(poly1))
        std::reverse(poly1.begin(), poly1.end());
    if(!Utilities::isPolygonClockwise(poly2))
        std::reverse(poly2.begin(), poly2.end());
    for(uint i = 0; i < poly1.size(); i++)
        for(uint j = 0; j < poly2.size(); j++)
            if(*poly1[i] == *poly2[j] && *poly1[(i + 1) % poly1.size()] == *poly2[(j + 1) % poly2.size()])
            {
                return true;
            }
    return false;
}

VertexList CityGMLCore::createLiDARDTMVertices()
{
    VertexList dtmVertices;
    for(uint i = 0; i < lidarDTMPoints.size(); i++)
        dtmVertices.push_back(std::make_shared<Vertex>(lidarDTMPoints[i]->getX(), lidarDTMPoints[i]->getY(), lidarDTMPoints[i]->getZ()));

    return dtmVertices;

}

/**
 * @brief CityGMLCore::removeAlreadyExistingPoints method for removing LiDAR points that are already in the boundaries' list or in the arcs' list
 * @param boundaries the buildings' boundaries
 * @param arcs the streets' arcs
 */
void CityGMLCore::removeAlreadyExistingPoints(std::vector<std::vector<VertexList> > boundaries, std::vector<VertexList> arcs)
{
    // Create a vector to store 2D points from the lidarDTMPoints
    pointVec points_vector;
    double epsilon = SemantisedTriangleMesh::Point::EPSILON;

    // Populate the points_vector with the lidarDTMPoints' x and y coordinates, setting the z coordinate to 0
    for (uint i = 0; i < lidarDTMPoints.size(); i++) {
        auto p = lidarDTMPoints.at(i);
        point_t point = { p->getX(), p->getY(), 0 };
        points_vector.push_back(point);
    }

    // Create a KDTree with the points_vector
    KDTree tree(points_vector);

    // A value used to mark points as used
    int used = 9009;

    // Parallel loop using OpenMP with 31 threads (num_threads(31))
    #pragma omp parallel for num_threads(31)
    for (uint i = 0; i < boundaries.size(); i++) {
        for (uint j = 0; j < boundaries.at(i).size(); j++) {
            for (uint k = 0; k < boundaries.at(i).at(j).size(); k++) {
                auto p = boundaries.at(i).at(j).at(k);

                // Create a 2D point from the boundary point coordinates
                point_t point = { p->getX(), p->getY(), 0.0 };

                // Find the neighbors of the boundary point within the specified epsilon distance in the KDTree
                auto neighbors = tree.neighborhood_indices(point, epsilon);

                // Iterate over the neighbors found and mark their corresponding lidarDTMPoints as used
                for (uint i = 0; i < neighbors.size(); i++) {
                    // Use a critical section to safely access and modify the 'info' field of lidarDTMPoints
                    #pragma omp critical
                    {
                        lidarDTMPoints[neighbors[i]]->setInfo(static_cast<void*>(&used));
                    }
                }
            }
        }
    }

    // Now, remove points that are marked as used from the lidarDTMPoints vector
    for (uint i = 0; i < lidarDTMPoints.size(); i++) {
        if (lidarDTMPoints[i]->getInfo() != nullptr && *static_cast<int*>(lidarDTMPoints[i]->getInfo()) == used) {
            lidarDTMPoints.erase(lidarDTMPoints.begin() + i);
        }
    }

}

// This function removes OSM way nodes that lie inside buildings from the OSM data.
void CityGMLCore::removeOSMWayNodesFromBuildings(std::vector<std::string> &ways) {
    // Create a vector to store 2D points from the buildings' outlines
    pointVec points_vector;

    // Create a map to link point indices with building indices
    std::map<uint, uint> pointBuildingLink;

    // Initialize variables for point and building IDs, and mean diagonal measurement
    uint id = 0, i = 0;
    double meanDiagonal = 0;

    // Loop through each building in the 'buildings' vector
    for (auto &building : buildings) {
        // Get the outlines of the building
        auto boundaries = building->getOutlines();

        // Process each point in the building's outer boundary
        for (auto &p : boundaries.at(0)) {
            // Create a 2D point from the point's x and y coordinates and set the z coordinate to 0
            point_t point = { p->getX(), p->getY(), 0 };
            points_vector.push_back(point);

            // Link the point ID to the building ID in the pointBuildingLink map
            pointBuildingLink.insert(std::make_pair(id++, i));
        }

        // Calculate the diagonal measure of the building's bounding box
        auto bb = Utilities::bbExtraction(vertexToPointList(building->getOutlines().at(0)));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    // Calculate the mean diagonal measure
    meanDiagonal /= static_cast<double>(buildings.size());

    // Create a KDTree with the points_vector for efficient neighborhood searches
    KDTree tree(points_vector);
    uint counter = 0;

    for (const auto &w : ways) {
        // Get the OSM way for the current ID
        auto way = osm.getWay(w);
        auto nodes = way->getNodes();

        // Process each node of the way
        for (auto &node : nodes) {
            PointList intersections;
            auto v = node->getCoordinates();

            // Find the neighbors (buildings) of the current node within a specified radius (meanDiagonal / 2)
            std::vector<std::pair<size_t, double>> neighbors_distances;
            std::vector<uint> neighboringPolygons;
            point_t query_point = { v->x, v->y, 0.0 };
            neighbors_distances.clear();
            auto neighbors = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            // Find neighboring polygons (buildings) based on their points' indices
            for (auto &building : neighbors) {
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointBuildingLink.at(building));
                if (pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointBuildingLink.at(building));
            }

            // Check if the node lies inside any neighboring building's outer boundary and outside all inner boundaries
            for (auto npit = neighboringPolygons.begin(); npit != neighboringPolygons.end(); npit++) {
                auto building_boundaries = buildings.at(*npit)->getOutlines();
                auto point = std::make_shared<SemantisedTriangleMesh::Point>(v->x, v->y, 0);
                bool insideOuter = Utilities::isPointInsidePolygon(point, vertexToPointList(building_boundaries.at(0)));
                bool outsideInners = true;

                // Check if the point lies inside the outer boundary and outside all inner boundaries of the building
                for (auto pit = building_boundaries.begin(); pit != building_boundaries.end(); pit++)
                    outsideInners = outsideInners && !Utilities::isPointInsidePolygon(point, vertexToPointList(*pit));

                // If the point is inside the outer boundary and outside all inner boundaries, check if it is on the boundary
                if (insideOuter && outsideInners) {
                    bool onBoundary = false;
                    for (auto &boundary : building_boundaries) {
                        for (uint m = 1; m < boundary.size(); m++) {
                            auto p1 = boundary.at(m - 1);
                            auto p2 = boundary.at(m);
                            double p1z = p1->getZ();
                            p1->setZ(0);
                            double p2z = p2->getZ();
                            p2->setZ(0);

                            // Check if the point lies on any boundary segment of the building
                            if (Utilities::isPointInSegment(p1.get(), p2.get(), point.get())) {
                                p1->setZ(p1z);
                                p2->setZ(p2z);
                                onBoundary = true;
                                break;
                            }
                            p1->setZ(p1z);
                            p2->setZ(p2z);
                        }
                    }

                    // If the point is not on the boundary, remove the node from the OSM data
                    if (!onBoundary) {
                        osm.removeNode(node->getId());
                        break;
                    }
                }
            }
        }
        counter++;
        std::cout << counter * 100 / ways.size() << "%\r";
    }
    std::cout << std::endl << std::flush;
}

// Function to extract building heights from LiDAR data based on building outlines.
std::vector<std::vector<double>> CityGMLCore::extractBuildingsHeightsFromLidar(double scale_factor, SemantisedTriangleMesh::Point origin)
{
    std::vector<std::vector<double>> heights; // Vector to store heights of each building
    unsigned int counter = 0; // Counter for tracking progress

    // Initialize an empty vector for each building
    for (auto &building : buildings)
    {
        std::vector<double> building_heights;
        heights.push_back(building_heights);
    }

    // Convert LiDAR points to a 2D point vector
    pointVec points_vector;
    for (auto p : lidarDSMPoints)
    {
        std::vector<double> point = {p->getX(), p->getY(), 0.0};
        points_vector.push_back(point);
    }

    // Build a KDTree from the LiDAR points for efficient nearest neighbor searches
    KDTree tree(points_vector);

    counter = 0;

    std::cout << "0%" << std::endl;

    // Loop through each building
    #pragma omp parallel for num_threads(31)
    for (uint i = 0; i < buildings.size(); i++)
    {
        const auto &boundaries = buildings.at(i)->getOutlines();
        std::vector<VertexList> boundaries2D;

        // Convert building boundaries to 2D points
        for (const auto &boundary : boundaries)
        {
            VertexList boundary2D;
            for (const auto &v : boundary)
                boundary2D.push_back(std::make_shared<Vertex>(v->getX(), v->getY(), 0.0));

            if(*boundary2D.back() != *boundary2D[0])
                boundary2D.push_back(boundary2D.at(0));
            boundaries2D.push_back(boundary2D);
        }

        // Calculate the middle point and sphere radius for nearest neighbor search
        const std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> &bb = Utilities::bbExtraction(boundaries2D[0]);
        double sphere_radius = ((*bb[0]) - (*bb[2])).norm() / 2;
        SemantisedTriangleMesh::Point middle(0, 0, 0);
        std::for_each(boundaries2D[0].begin(), boundaries2D[0].end(), [&middle](std::shared_ptr<Vertex> v){
            middle += *v;
        });
        middle /= static_cast<double>(boundaries2D[0].size());

        std::vector<size_t> neighbors_indices;
        std::vector<std::pair<unsigned int, unsigned int>> neighbors;
        point_t query_point = {middle.getX(), middle.getY(), 0.0};

        // Find neighbors within the sphere around the middle point using KDTree
        neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);

        uint counter = 0; // Counter for tracking progress within each building

        // Loop through neighbors and check if they are inside the building boundaries
        for (auto index : neighbors_indices)
        {
            auto v = lidarDSMPoints.at(index);
            auto p = std::make_shared<SemantisedTriangleMesh::Point>(v->getX(), v->getY(), 0);

            bool insideOuter = false, outsideInner = true;

            // Check if the LiDAR point is inside the outer boundary of the building
            if (Utilities::isPointInsidePolygon(p, boundaries2D[0]))
            {
                insideOuter = true;

                // Check if the LiDAR point is outside any inner boundaries (holes) of the building
                for (uint l = 1; l < boundaries2D.size(); l++)
                {
                    if (Utilities::isPointInsidePolygon(p, boundaries2D[l]))
                    {
                        outsideInner = false;
                        break;
                    }
                }
            }

            // If the LiDAR point is inside both outer and inner boundaries, add its height to the building's heights vector
            if (insideOuter && outsideInner)
                heights[i].push_back(lidarDSMPoints.at(index)->getZ());
        }

        // If no height was found for the building, add the default height from its outline
        if (heights[i].size() == 0)
            heights[i].push_back(buildings[i]->getOutlines()[0][0]->getZ());

        // Update the progress counter and print progress
        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / buildings.size() << "%\r" << std::flush;
        }
    }

    return heights; // Return the vector of building heights
}

// Function to associate elevations from LiDAR data to sets of points.
void CityGMLCore::associateElevations(const std::vector<std::vector<VertexList>> &triangulationHoles, const std::vector<VertexList> &streetsArcsPoints)
{
    unsigned int counter = 0; // Counter for tracking progress

    // Convert LiDAR DTM points to a 2D point vector
    pointVec points_vector;
    for (const auto &p : lidarDTMPoints)
    {
        std::vector<double> point = {p->getX(), p->getY(), 0.0};
        points_vector.push_back(point);
    }

    // Build a KDTree from the LiDAR DTM points for efficient nearest neighbor searches
    KDTree tree(points_vector);

    // Loop through each set of points in triangulationHoles and associate elevations
    #pragma omp parallel for num_threads(31)
    for (uint i = 0; i < triangulationHoles.size(); i++)
    {
        auto boundaries = triangulationHoles[i];
        for (const auto &boundary : boundaries)
            for (const auto &p : boundary)
            {
                point_t point = {p->getX(), p->getY(), 0.0};

                // Find the nearest LiDAR DTM point using the KDTree
                size_t ret_index = tree.nearest_index(point);
                // Set the elevation of the current point to the elevation of the nearest LiDAR DTM point
                p->setZ(lidarDTMPoints[ret_index]->getZ());
            }

        // Update the progress counter and print progress
        #pragma omp critical
        {
            std::cout << counter++ * 100 / triangulationHoles.size() << "%\r" << std::flush;
        }
    }

    // Loop through each set of points in streetsArcsPoints and associate elevations
    #pragma omp parallel for num_threads(31)
    for (uint i = 0; i < streetsArcsPoints.size(); i++)
    {
        auto polyline = streetsArcsPoints[i];
        for (const auto &p : polyline)
        {
            point_t point = {p->getX(), p->getY(), 0.0};

            // Find the nearest LiDAR DTM point using the KDTree
            size_t ret_index = tree.nearest_index(point);
            // Set the elevation of the current point to the elevation of the nearest LiDAR DTM point
            p->setZ(lidarDTMPoints[ret_index]->getZ());
        }

        // Update the progress counter and print progress
        #pragma omp critical
        {
            std::cout << counter++ * 100 / streetsArcsPoints.size() << "%\r" << std::flush;
        }
    }
}

std::string CityGMLCore::getOsmFilename() const
{
    return osmFilename;
}

void CityGMLCore::setOsmFilename(const std::string &value)
{
    osmFilename = value;
}


void CityGMLCore::setLevel(uint level, std::string meshFileName, std::string annotationFileName)
{
    switch (level) {
        case 0:
            setLevel0(meshFileName, annotationFileName);
        case 1:
            setLevel1(meshFileName, annotationFileName);
        default:
            ;
    }
}

bool CityGMLCore::isPointOutsideBounds(const std::shared_ptr<SemantisedTriangleMesh::Point> p)
{
    bool outside = true, inside = false;
    outside = !Utilities::isPointInsidePolygon(p, bounds[0]);
    for(uint i = 1; i < bounds.size(); i++)
    {
        if(Utilities::isPointInsidePolygon(p, bounds[i]))
        {
            return true;
        }
    }
    return outside;
}

int CityGMLCore::buildLevel(const uint level)
{

    switch (level) {
        case 0:
            return buildLevel0();
        case 1:
            return buildLevel1();
    }
    return -1;
}

// Function to read LiDAR data from a file and extract classified points within specified bounds.
int CityGMLCore::readLiDAR(const std::string &filename,
                           const std::vector<std::string> &allowedClasses,
                           const bool checkInsideBounds,
                           const std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>>> &bounds,
                           std::vector<std::pair<std::shared_ptr<SemantisedTriangleMesh::Point>, std::string>>& classifiedPoints)
{
    // Open the LiDAR file for reading
    std::ifstream ifs;
    ifs.open(filename, std::ios::in | std::ios::binary);

    if (ifs.is_open() && bounds.size() > 0)
    {
        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const& header = reader.GetHeader();

        std::cout << "Compressed: " << header.Compressed() << std::endl;
        std::cout << "Signature: " << header.GetFileSignature() << std::endl;
        std::cout << "Points count: " << header.GetPointRecordsCount() << std::endl;

        auto bb = Utilities::bbExtraction(bounds[0]); // Extract bounding box for external boundary (first one)

        // Loop through each point in the LiDAR data
        while (reader.ReadNextPoint())
        {
            liblas::Point const& p = reader.GetPoint();
            auto p_ = std::make_shared<SemantisedTriangleMesh::Point>(p.GetX(), p.GetY(), p.GetZ());
            std::string s = p.GetClassification().GetClassName();
            auto p_2d = std::make_shared<SemantisedTriangleMesh::Point>(*p_);

            p_2d->setZ(0); // Convert the 3D point to a 2D point (ignoring the Z coordinate)

            // Check if the point is within the bounds (if enabled)
            if (!checkInsideBounds || Utilities::isPointInsidePolygon(p_2d, bb))
            {
                bool insideBounds = false;

                // If bounds checking is enabled, check if the point is inside the outer boundary
                if (checkInsideBounds && Utilities::isPointInsidePolygon(p_2d, bounds[0]))
                {
                    insideBounds = true;

                    // Check if the point is inside any inner boundaries (holes)
                    for (uint i = 1; i < bounds.size(); i++)
                    {
                        auto bound = bounds.at(i);
                        if (Utilities::isPointInsidePolygon(p_2d, bound))
                        {
                            insideBounds = false;
                            break;
                        }
                    }

                    if (!insideBounds)
                        break; // Break the loop if the point is not inside any inner boundaries
                }

                // If bounds checking is disabled or the point is inside the bounds
                if (!checkInsideBounds || insideBounds)
                {
                    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });

                    // Check if the point's classification is in the list of allowed classes
                    auto sit = std::find_if(allowedClasses.begin(), allowedClasses.end(), [s](std::string classString) {
                        return classString.compare(s) == 0;
                    });

                    if (sit != allowedClasses.end())
                        classifiedPoints.push_back(std::make_pair(p_, s)); // Store the classified point
                }
            }
        }

        ifs.close();

        return 0; // Return success
    }

    return -1; // Return error if the file could not be opened
}

/*****************************************************************************/

// Function to calculate the minimum and maximum X and Y coordinates from lidarDTMPoints and bounds
void CityGMLCore::calculateMinMax(SemantisedTriangleMesh::Point &min, SemantisedTriangleMesh::Point &max)
{
    // Initialize the min and max points with extreme values
    min = SemantisedTriangleMesh::Point(std::numeric_limits<double>::max(),
                                        std::numeric_limits<double>::max(),
                                        0);
    max = SemantisedTriangleMesh::Point(-std::numeric_limits<double>::max(),
                                        -std::numeric_limits<double>::max(),
                                        0);

    // Loop through lidarDTMPoints and update min and max points
    for (const auto& point : lidarDTMPoints) {
        if (point->getX() < min.getX())
            min.setX(point->getX());
        if (point->getY() < min.getY())
            min.setY(point->getY());
        if (point->getX() > max.getX())
            max.setX(point->getX());
        if (point->getY() > max.getY())
            max.setY(point->getY());
    }

    // Loop through bounds and update min and max points
    for (const auto& pointList : bounds) {
        for (const auto& point : pointList) {
            if (point->getX() < min.getX())
                min.setX(point->getX());
            if (point->getY() < min.getY())
                min.setY(point->getY());
            if (point->getX() > max.getX())
                max.setX(point->getX());
            if (point->getY() > max.getY())
                max.setY(point->getY());
        }
    }
}

// Function to normalize lidarDTMPoints, lidarDSMPoints, and bounds using the origin and city size
void CityGMLCore::normalizePoints(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>>& points,
                     const std::shared_ptr<SemantisedTriangleMesh::Point>& origin,
                     double city_size) {
    // Loop through the points and normalize them by subtracting the origin and dividing by city_size
    for (auto& point : points) {
        auto opoint = std::make_shared<SemantisedTriangleMesh::Point>(*point);
        (*point) -= (*origin);
        (*point) /= city_size;
    }
}

// Function to filter OpenStreetMap nodes outside the bounds
void CityGMLCore::filterNodesOutsideBounds() {
    for (auto nit = osm.getNodes().begin(); nit != osm.getNodes().end();) {
        auto p = nit->second->getCoordinates();
        auto point = std::make_shared<SemantisedTriangleMesh::Point>(p->x, p->y, 0);
        // Check if the point is outside the bounds and remove it if true
        if (isPointOutsideBounds(point))
            nit = osm.removeNode(nit->second->getId());
        else
            nit++;
    }
}

// Function to process OpenStreetMap ways and identify buildings and streets
void CityGMLCore::processOpenStreetMapWays() {
    for (const auto& way : osm.getWays()) {
        if (way.second->checkTag(std::make_pair("building", "")))
            buildingsArcs.push_back(way.first);

        if (way.second->checkTag(std::make_pair("highway", "")) &&
            (!way.second->checkTag(std::make_pair("area", "")) ||
             way.second->checkTag(std::make_pair("area", "no")))) {
            fixWay(way.second);
            if (way.second->getNodes().size() > 1)
                streetsArcs.push_back(way.first);
        }
    }
}

void CityGMLCore::processRelations(std::set<std::string> &usedWays)
{
    // Loop through each relation in the OSM data
    for (auto rit = osm.getRelations().begin(); rit != osm.getRelations().end();) {
        auto tags = rit->second->getTags();

        // Lambda function to check if the relation represents a multipolygon
        auto checkMultipolygon = [](std::pair<std::string, std::string> p) {
            return ((p.first.compare("type") == 0) && p.second.compare("multipolygon") == 0);
        };

        // Lambda function to check if the relation represents a building
        auto checkBuilding = [](std::pair<std::string, std::string> p) {
            return (p.first.compare("building")) == 0;
        };

        // We only consider relations defining buildings as multipolygons
        if (std::find_if(tags.begin(), tags.end(), checkMultipolygon) != tags.end() &&
            std::find_if(tags.begin(), tags.end(), checkBuilding) != tags.end()) {

            // Get the ways representing the building boundaries
            auto buildingBoundaries = rit->second->getWays();
            auto building = std::make_shared<Building>();
            std::vector<VertexList> buildingBoundariesPoints;
            bool hasOuter = false;
            bool invalidRelation = false;

            // Loop through each boundary way of the building
            for (auto bbit = buildingBoundaries.begin(); bbit != buildingBoundaries.end(); bbit++) {

                if (bbit->first == nullptr)
                    continue;

                // Check if the way has already been used, if not, add it to the usedWays set
                auto it = std::find(usedWays.begin(), usedWays.end(), bbit->first->getId());
                if (it == usedWays.end()) {
                    usedWays.insert(bbit->first->getId());

                    // Get the nodes (vertices) of the boundary
                    auto boundaryNodes = bbit->first->getNodes();
                    VertexList boundaryPoints;

                    // The isPolygonClockwise function requires the polygon to be open (the last segment is implicit)
                    if (boundaryNodes.at(0)->getId() == boundaryNodes.back()->getId())
                        boundaryNodes.pop_back();

                    // Loop through each node (vertex) of the boundary way
                    for (uint j = 0; j < boundaryNodes.size(); j++) {
                        auto p = boundaryNodes.at(j)->getCoordinates();

                        // Check if the vertex already exists in the osmPointToMeshPointMap, if not, add it
                        if (osmPointToMeshPointMap.find(p) == osmPointToMeshPointMap.end()) {
                            auto newVertex = std::make_shared<Vertex>(p->x, p->y, 0);
                            newVertex->setInfo(boundaryNodes.at(j));
                            osmPointToMeshPointMap.insert(std::make_pair(p, newVertex));
                            boundaryPoints.push_back(newVertex);
                        } else
                            boundaryPoints.push_back(osmPointToMeshPointMap.at(p));
                    }

                    // Check if the polygon representing the boundary is clockwise or anti-clockwise
                    bool isClockwise = Utilities::isPolygonClockwise(vertexToPointList(boundaryPoints));

                    // Re-close the polygon by adding the first vertex at the end
                    boundaryPoints.push_back(boundaryPoints.at(0));

                    // Next part is performed only if we are in the outmost boundary of the multipolygon
                    if (bbit->second.compare("outer") == 0) {
                        if (!hasOuter) {
                            hasOuter = true;
                            // Exterior boundaries are in anti-clockwise order by convention.
                            if (isClockwise)
                                std::reverse(boundaryPoints.begin(), boundaryPoints.end());
                            // The first boundary of the list is the outmost one.
                            buildingBoundariesPoints.insert(buildingBoundariesPoints.begin(), boundaryPoints);
                            building->addOuterBoundary(boundaryPoints);
                        } else {
                            // TO DO: The case of multiple outer boundaries needs to be discussed and handled accordingly.
                            // For now, surplus outer boundaries are eliminated by removing the way and erasing it from usedWays.
                            buildingsArcs.push_back(bbit->first->getId());
                            rit->second->removeWay(bbit->first);
                            usedWays.erase(bbit->first->getId());
                        }
                    } else {
                        // Interior boundaries are in clockwise order by convention.
                        if (!isClockwise)
                            std::reverse(boundaryPoints.begin(), boundaryPoints.end());
                        buildingBoundariesPoints.push_back(boundaryPoints);
                        building->addInnerBoundary(boundaryPoints);
                    }
                } else {
                    // The way has been used in another relation, marking the current relation as invalid.
                    invalidRelation = true;
                    break;
                }
            }

            // If the relation is valid (no duplicate ways used), add the building to the buildings vector and move to the next relation.
            if (!invalidRelation) {
                building->setId(std::to_string(buildings.size()));
                buildings.push_back(building);
                rit++;
            } else {
                // If the relation is invalid, remove it from the OSM data and move to the next relation.
                rit = osm.removeRelation(rit->second->getId());
            }
        } else
            rit++; // Move to the next relation if it does not represent a building multipolygon.
    }

}

void CityGMLCore::processBuildingWays(std::set<std::string> &usedWays)
{
    // Loop through each building way ID in the buildingsArcs vector
    for (auto ait = buildingsArcs.begin(); ait != buildingsArcs.end(); ait++) {
        // Check if the way has not been used yet
        if (std::find(usedWays.begin(), usedWays.end(), *ait) == usedWays.end()) {
            std::vector<VertexList> buildingBoundariesPoints;
            VertexList boundaryPoints;

            // Get the nodes (vertices) of the way
            auto nodes = osm.getWays().at(*ait)->getNodes();

            // The isPolygonClockwise function requires the polygon to be open (the last segment is implicit)
            if (nodes.at(0)->getId() == nodes.back()->getId())
                nodes.pop_back();

            // Loop through each node (vertex) of the way
            for (uint j = 0; j < nodes.size(); j++) {
                auto p = nodes.at(j)->getCoordinates();

                // Check if the vertex already exists in the osmPointToMeshPointMap, if not, add it
                if (osmPointToMeshPointMap.find(p) == osmPointToMeshPointMap.end()) {
                    auto newVertex = std::make_shared<Vertex>(p->x, p->y, 0);
                    newVertex->setInfo(nodes.at(j));
                    osmPointToMeshPointMap.insert(std::make_pair(p, newVertex));
                    boundaryPoints.push_back(newVertex);
                } else
                    boundaryPoints.push_back(osmPointToMeshPointMap.at(p));
            }

            // Check if the polygon representing the boundary is clockwise or anti-clockwise
            bool isClockwise = Utilities::isPolygonClockwise(vertexToPointList(boundaryPoints));

            // Re-close the polygon by adding the first vertex at the end
            boundaryPoints.push_back(boundaryPoints.at(0));

            // Exterior boundaries are in anti-clockwise order by convention.
            if (isClockwise)
                std::reverse(boundaryPoints.begin(), boundaryPoints.end());

            // Create a new Building object, add the outer boundary, and assign an ID
            buildings.push_back(std::make_shared<Building>());
            buildings.back()->addOuterBoundary(boundaryPoints);
            buildings.back()->setId(std::to_string(buildings.size()));
        }
    }
}

void CityGMLCore::preProcessBuildings()
{
    // Remove buildings with vertices outside the bounds
    for (auto iit = buildings.begin(); iit != buildings.end();) {
        std::vector<VertexList> building_boundaries = (*iit)->getOutlines();
        bool erased = false;

        // Loop through each boundary of the building
        for (auto jit = building_boundaries.begin(); jit != building_boundaries.end(); jit++) {
            for (auto kit = jit->begin(); kit != jit->end(); kit++) {
                // Check if any vertex of the boundary is outside the bounds
                if (isPointOutsideBounds(*kit)) {
                    // Reset all vertices and clear the boundary if any vertex is outside the bounds
                    for (auto kit = jit->begin(); kit != jit->end(); kit++)
                        kit->reset();
                    jit->clear();
                    iit = buildings.erase(iit); // Erase the building from the buildings vector
                    erased = true;
                    break;
                }
            }
            if (erased)
                break;
        }
        if (!erased)
            iit++; // Move to the next building if the current one is not erased
    }

    // Remove buildings contained within other buildings (buildings inside other buildings cannot be handled)
    for (auto it = buildings.begin(); it != buildings.end();) {
        auto building1 = *it;

        // Find if there is any other building that contains the current building
        auto containedIt = std::find_if(buildings.begin(), buildings.end(), [building1](std::shared_ptr<Building> building2) {
            if (building1->getId().compare(building2->getId()) == 0)
                return false;

            bool contained = false;
            auto boundaries1 = building1->getOutlines();
            auto boundaries2 = building2->getOutlines();

            // Convert the boundaries into vector of shared pointers to facilitate the comparison
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary1(boundaries1[0].begin(), boundaries1[0].end());
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary2(boundaries2[0].begin(), boundaries2[0].end());

            // Check if building1 is completely inside building2 (building2 contains building1)
            if (Utilities::isPolygonInsidePolygon(boundary1, boundary2)) {
                contained = true;
                for (uint j = 1; j < boundaries2.size(); j++) {
                    // Check if any other boundary of building2 contains building1
                    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary3(boundaries2[j].begin(), boundaries2[j].end());
                    if (Utilities::isPolygonInsidePolygon(boundary1, boundary3)) {
                        contained = false;
                        break;
                    }
                }
                if (!contained)
                    return false;
            }
            return contained;
        });

        // If building1 is contained inside another building, erase it from the buildings vector
        if (containedIt != buildings.end())
            it = buildings.erase(it);
        else
            it++;
    }

}

void CityGMLCore::groupBuildings(std::vector<std::shared_ptr<BuildingsGroup> >& groups)
{
    uint reachedGroupId = 0;
    for(const auto& b1 : buildings)
    {
        b1->addFlag(Building::FlagType::USED);
        auto group = std::make_shared<BuildingsGroup>();
        std::queue<std::shared_ptr<Building> > queue;
        queue.push(b1);
        while(!queue.empty())
        {
            auto building = queue.front();
            if((*building) != (*b1))
            {
                b1->addAdjacentBuilding(building);
                building->addAdjacentBuilding(b1);
            }
            queue.pop();
            group->addBuilding(building);
            for(const auto& b2 : buildings)
            {
                if((*b1) == (*b2))
                    continue;
                if(b2->searchFlag(Building::FlagType::USED) == -1 &&
                   arePolygonsAdjacent(building->getOutlines().at(0), b2->getOutlines().at(0)))
                {
                    b2->addFlag(Building::FlagType::USED);
                    queue.push(b2);
                }
            }
        }
        if(group->getBuildings().size() > 1)
        {
            group->setId(std::to_string(reachedGroupId++));
            groups.push_back(group);
        }

    }
    for(auto b : buildings)
        b->removeFlag(Building::FlagType::USED);
}

void CityGMLCore::generateVertices(std::vector<VertexList> &arcsPoints)
{
    // Remove streets with less than 2 nodes (vertices)
    for (auto ait = streetsArcs.begin(); ait != streetsArcs.end();) {
        if (osm.getWays().at(*ait)->getNodes().size() < 2) {
            // Erase the street from the streetsArcs vector if it has less than 2 nodes (vertices)
            ait = streetsArcs.erase(ait);
        } else {
            VertexList arc_points;
            auto arc_nodes = osm.getWays().at(*ait)->getNodes();

            // Convert the nodes into vertices and add them to the arc_points vector
            for (auto node : arc_nodes) {
                auto p = node->getCoordinates();
                if (osmPointToMeshPointMap.find(p) == osmPointToMeshPointMap.end()) {
                    auto newVertex = std::make_shared<Vertex>(p->x, p->y, 0);
                    newVertex->setInfo(node);
                    osmPointToMeshPointMap.insert(std::make_pair(p, newVertex));
                    arc_points.push_back(newVertex);
                } else
                    arc_points.push_back(osmPointToMeshPointMap.at(p));
            }

            // Add the vertices of the street to the arcsPoints vector
            arcsPoints.push_back(arc_points);
            ait++; // Move to the next street arc
        }
    }

}

void CityGMLCore::processBuildingsGroups(std::vector<std::shared_ptr<BuildingsGroup> >& groups, std::vector<std::vector<VertexList> >& triangulationHoles,
                            std::set<std::string>& usedBuildings)
{
    // Loop through each group in the groups vector
    for (auto group : groups) {
        // Compute the adjacency graph for the current group of buildings
        group->computeAdjacencyGraph();

        // Extract the overall base polygon of the group
        auto groupBoundaries = group->extractOverallBasePolygon();

        // Counter variable to keep track of building indices within the group
        uint counter = 0;


        // Get the list of buildings to be triangulated within the current group
        auto buildingsToBeTriangulated = group->getBuildings();

        // Vector to store inner vertices of the group (vertices that are not part of building outlines)
        std::vector<std::shared_ptr<Vertex>> groupInnerVertices;

        // Loop through each building in the group
        for (const auto &building : group->getBuildings()) {
            // Check if the building's ID is already in the usedBuildings set
            auto it = std::find(usedBuildings.begin(), usedBuildings.end(), building->getId());

            // Check for consistency: a building should only belong to one group
            if (it != usedBuildings.end()) {
                std::cerr << "Impossible, all buildings that are part of a group must be in that group. There couldn't be more groups containing a building.";
                exit(12);
            }

            // Add the building's ID to the usedBuildings set
            usedBuildings.insert(building->getId());
        }

        // Add the group boundaries to the triangulationHoles vector for later triangulation
        triangulationHoles.push_back(groupBoundaries);
    }

    // Loop through each building in the buildings vector
    for (auto it = buildings.begin(); it != buildings.end();) {
        auto building = *it;

        // Check if the building is not already used in a group
        auto usedIt = std::find_if(usedBuildings.begin(), usedBuildings.end(), [building](std::string s) { return building->getId().compare(s) == 0; });
        if (usedIt == usedBuildings.end()) {
            // Check if the building is not completely contained inside any existing triangulation hole
            auto containedIt = std::find_if(triangulationHoles.begin() + 1, triangulationHoles.end(), [building](std::vector<VertexList> list) {
                bool contained = false;
                auto boundaries = building->getOutlines();

                // Convert building and triangulation hole boundaries into vectors of shared pointers
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary1(boundaries[0].begin(), boundaries[0].end());
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary2(list[0].begin(), list[0].end());

                // Check if building is completely inside the current triangulation hole (list[0])
                if (Utilities::isPolygonInsidePolygon(boundary1, boundary2)) {
                    contained = true;
                    for (uint j = 1; j < list.size(); j++) {
                        // Check if building is inside any other hole (list[j])
                        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point>> boundary3(list[j].begin(), list[j].end());
                        if (Utilities::isPolygonInsidePolygon(boundary1, boundary3)) {
                            contained = false;
                            break;
                        }
                    }
                    if (!contained)
                        return false;
                }

                return contained;
            });

            if (containedIt == triangulationHoles.end()) {
                // If the building is not contained inside any hole, add its outlines to the triangulationHoles vector
                triangulationHoles.push_back(building->getOutlines());
                it++; // Move to the next building
            } else {
                // If the building is completely contained inside a hole, remove it from the buildings vector
                it = buildings.erase(it);
            }
        } else {
            it++; // Move to the next building if it is already part of a group
        }
    }


}

void CityGMLCore::reInsertStreetsClippings(std::vector<std::pair<uint, uint> > keptClippings, std::vector<VertexList >& arcsPoints)
{
    // Loop through the ranges in keptClippings vector
    for (uint i = 0; i < keptClippings.size(); i++)
    {
        // Calculate the number of new lines to insert
        uint numberOfNewLines = keptClippings.at(i).second - keptClippings.at(i).first;

        // Loop through the new lines to insert
        for (uint j = 0; j < numberOfNewLines; j++)
        {
            // Get the previous line and clipping to merge
            VertexList prevLine = arcsPoints.at(keptClippings.at(i).first);
            VertexList clipping = arcsPoints.at(keptClippings.at(i).first + j + 1);

            // Check if the first two vertices in the clipping are identical, remove one if they are
            if (clipping.at(0)->getId().compare(clipping.at(1)->getId()) == 0)
                clipping.erase(clipping.begin());

            // Check if the last two vertices in the previous line are identical, remove one if they are
            if (prevLine.at(prevLine.size() - 2)->getId().compare(prevLine.at(prevLine.size() - 1)->getId()) == 0)
                arcsPoints.at(keptClippings.at(i).first).pop_back();

            // Append the clipping to the previous line
            arcsPoints.at(keptClippings.at(i).first).insert(arcsPoints.at(keptClippings.at(i).first).end(), clipping.begin(), clipping.end());
        }

        // Erase the lines within the range [first, second] (excluding the first line)
        arcsPoints.erase(arcsPoints.begin() + keptClippings.at(i).first + 1, arcsPoints.begin() + keptClippings.at(i).second + 1);

        // Adjust the indices of the remaining ranges in keptClippings vector
        for (uint j = i + 1; j < keptClippings.size(); j++)
        {
            keptClippings.at(j).first -= numberOfNewLines;
            keptClippings.at(j).second -= numberOfNewLines;
        }
    }
}
/*****************************************************************************/

int CityGMLCore::buildLevel0()
{
    if(!osmIsLoaded)
        return -1;

    if(meshes[0] != nullptr)
        meshes[0].reset();

    meshes[0] = std::make_shared<TriangleMesh>();

    // Calculate the minimum and maximum coordinates from lidarDTMPoints and bounds
    SemantisedTriangleMesh::Point min(std::numeric_limits<double>::max(),
              std::numeric_limits<double>::max(),
              0);
    SemantisedTriangleMesh::Point max(-std::numeric_limits<double>::max(),
              -std::numeric_limits<double>::max(),
              0);

    calculateMinMax(min, max);
    auto origin = std::make_shared<SemantisedTriangleMesh::Point>((min + max) / 2);
    double city_size = (max - min).norm();

    origin->setZ(0);

    for(const auto &node : osm.getNodes()){
        auto pos = node.second->getCoordinates();
        OpenStreetMap::Point origin2D(origin->getX(), origin->getY());
        *pos -= origin2D;
        *pos /= city_size;
    }

    //Points normalization for reducing precision errors
    normalizePoints(lidarDTMPoints, origin, city_size);
    normalizePoints(lidarDSMPoints, origin, city_size);
    for (auto& bound : bounds)
        normalizePoints(bound, origin, city_size);

    std::vector<PointList > boundaries;

    std::cout << "Cleaning polygons/lines" << std::endl;

    filterNodesOutsideBounds();
    processOpenStreetMapWays();

    this->buildings.clear();
    std::set<std::string> usedWays;
    std::cout.precision(15);
    std::cout << city_size << std::endl;
    origin->print(std::cout, BracketsType::NONE, " ");
    std::cout << std::endl;

    //Buildings are partly defined as relations and partly as ways
    processRelations(usedWays);
    //Remaining buildings are defined by simple OSMWays
    processBuildingWays(usedWays);
    //Removing buildings with points outside the bounds
    preProcessBuildings();

    std::cout << "Removing nodes into buildings"<< std::endl;
    removeOSMWayNodesFromBuildings(streetsArcs);

    //Fixing building boundaries after managing collision between buildings and streets
    for(auto bit = buildings.begin(); bit != buildings.end();)
    {
        (*bit)->fixBoundaries();
        if((*bit)->getOutlines().size() == 0)
            bit = buildings.erase(bit);
        else
            bit++;
    }

    std::vector<VertexList > external_bb = {{
        std::make_shared<Vertex>(min.getX(), min.getY(), 0.0),
        std::make_shared<Vertex>(max.getX(), min.getY(), 0.0),
        std::make_shared<Vertex>(max.getX(), max.getY(), 0.0),
        std::make_shared<Vertex>(min.getX(), max.getY(), 0.0)
    }};
    *external_bb[0][0] = ((*external_bb[0][0]) - (*origin)) / city_size;
    *external_bb[0][1] = ((*external_bb[0][1]) - (*origin)) / city_size;
    *external_bb[0][2] = ((*external_bb[0][2]) - (*origin)) / city_size;
    *external_bb[0][3] = ((*external_bb[0][3]) - (*origin)) / city_size;

    std::vector<std::vector<VertexList> > triangulationHoles = {external_bb};

    std::vector<std::shared_ptr<BuildingsGroup> > groups;
    std::cout << "Creating buildings' groups" << std::endl;
    groupBuildings(groups);
    std::cout << "Ended!" << std::endl;


    std::vector<VertexList > arcsPoints, constraints; //Constraints are all the points that must be inserted in the triangulation

    //Vertices generation from arcs
    generateVertices(arcsPoints);

    //Remove OSMNodes that are not part of the scene
    for(auto nit = osm.getNodes().begin(); nit != osm.getNodes().end();)
    {
        auto n = *nit;
        if(osmPointToMeshPointMap.find(nit->second->getCoordinates()) == osmPointToMeshPointMap.end())
        {
            nit = osm.removeNode(nit->second->getId());
        } else
            nit++;
    }


    //Checking if an arc is a singlet (error)
    for(uint i = 0; i < arcsPoints.size(); i++)
        if(arcsPoints.at(i).size() == 1)
        {
            std::cout <<"Found singlet" << std::endl;
            exit(1312);
        }


    std::cout << "Removing inserted critical vertices:" << std::endl;
    removeLinePointsInsideBuildings(arcsPoints);

    std::cout << "Ended!" << std::endl;

    for(uint i = 0; i < arcsPoints.size(); i++)
        if(arcsPoints.at(i).size() < 2)
        {
            arcsPoints.erase(arcsPoints.begin() + i);
            streetsArcs.erase(streetsArcs.begin() + i);
            i--;
        }

    std::cout << "Removing segments intersecting buildings:" << std::endl;
    //Removing arcs' clippings
    auto keptClippings = removeSegmentsIntersectingBuildings(arcsPoints);  //VA SISTEMATO, DEVE USARE I GRUPPI DI EDIFICI
    std::cout << "Ended!" << std::endl;
    std::cout << "Extracting overall polygons" << std::endl;

    // Set to keep track of used buildings (by their IDs) to ensure buildings are not part of multiple groups
    std::set<std::string> usedBuildings;

    processBuildingsGroups(groups, triangulationHoles, usedBuildings);
    std::cout << "Ended!" << std::endl;

    removeAlreadyExistingPoints(triangulationHoles, arcsPoints);

    //Each street arc must be in the final triangulation
    constraints.insert(constraints.end(), arcsPoints.begin(), arcsPoints.end());
    //Each LiDAR point is should be inserted in the final triangulation

    VertexList constraintVertices = createLiDARDTMVertices();

    meshes[0]->triangulate(triangulationHoles, arcsPoints, constraintVertices);

    //Removed clippings needs to be removed
    reInsertStreetsClippings(keptClippings, arcsPoints);

    std::cout << "Ended" << std::endl;

    uint buildingPos = 0;
    std::cout << "Starting buildings' triangulation" << std::endl << "0%";


    for(const auto& building : buildings)
    {
        for(auto &boundary : building->getOutlines())
        {
            for(uint i = 1; i < boundary.size(); i++)
            {
                auto v1 = boundary.at(i - 1);
                auto v2 = boundary.at(i);

                auto closeVertices = meshes[0]->getVerticesCloseToLine(*v1, *v2, 1e-5);
                auto it = closeVertices.begin();
                while(it != closeVertices.end())
                    if(!Utilities::isPointInSegment(v1.get(), v2.get(), it->get()))
                        it = closeVertices.erase(it);
                    else it++;

                if(closeVertices.size() > 2)
                {
                    std::sort(closeVertices.begin(), closeVertices.end(), [boundary](std::shared_ptr<Vertex> v1,
                                                                                     std::shared_ptr<Vertex> v2){
                        double dist1 = (*v1 - *boundary[0]).norm();
                        double dist2 = (*v2 - *boundary[0]).norm();
                        return dist1 < dist2;
                    });
                    closeVertices.erase(closeVertices.begin());
                    closeVertices.pop_back();

                    boundary.insert(boundary.begin() + i, closeVertices.begin(), closeVertices.end());
                    i += closeVertices.size();
                }
            }
        }

        std::vector<uint> shell_triangles;
        std::map<uint, std::shared_ptr<Vertex> > pointsToVertices;
        std::vector<std::vector<uint> > polylines;
        std::vector<std::pair<uint, uint> > constraint_segments;
        std::vector<double*> points;
        std::vector<double*> constraints;
        std::vector<double*> holes;
        points.clear();
        constraints.clear();
        constraints.clear();
        pointsToVertices.clear();
        polylines.clear();
        constraint_segments.clear();


        std::map<std::string, std::vector<std::shared_ptr<Edge> > > verticesEdges;
        uint reached_id = 0;
        std::vector<uint> polyline;
        auto boundaries = building->getOutlines();

        for(uint j = 0; j < boundaries.size(); j++)
        {
            std::vector<uint> polyline;
            uint first = reached_id;


            for(uint k = 0; k < boundaries.at(j).size(); k++)
            {
                std::shared_ptr<Vertex> v = std::static_pointer_cast<Vertex>(boundaries.at(j).at(k));
                if(k < boundaries.at(j).size() - 1)
                {
                    auto pit = std::find_if(points.begin(), points.end(), [v](double* p) {
                        SemantisedTriangleMesh::Point p1(p[0], p[1], 0);
                        SemantisedTriangleMesh::Point p2(v->getX(), v->getY(), 0);
                        return  p1 == p2;
                    });

                    if(pit != points.end())
                    {
                        v = pointsToVertices.at(pit - points.begin());
                        boundaries.at(j).at(k) = v;
                    }

                    if(v->getId().compare("") == 0)
                    {
                        v = meshes[0]->addNewVertex(v->getX(), v->getY(), v->getZ());
                        v->setId(std::to_string(meshes[0]->getVerticesNumber() - 1));
                        boundaries.at(j).at(k) = v;
                    }

                    std::vector<std::shared_ptr<Edge> > connected = std::static_pointer_cast<Vertex>(boundaries.at(j).at(k))->getVE();

                    auto it = verticesEdges.find(v->getId());
                    if(it == verticesEdges.end())
                        verticesEdges.insert(std::make_pair(v->getId(), connected));

                    if(pit == points.end())
                    {
                        points.push_back(v->toDoubleArray());
                        pointsToVertices.insert(std::make_pair(reached_id, v));
                        polyline.push_back(reached_id++);
                    } else
                    {
                        polyline.push_back(pit - points.begin());
                        if(k == 0)
                            first = polyline.back();
                    }
                } else
                {
                    boundaries.at(j).pop_back();
                    boundaries.at(j).push_back(boundaries.at(j).at(0));
                    polyline.push_back(first);
                }
            }
            building->setOutlines(boundaries);

            polylines.push_back(polyline);

            if(j > 0)
            {
                double v[2] = {(boundaries.at(j)[1]->getX() - boundaries.at(j)[0]->getX()) * 1E-2,
                               (boundaries.at(j)[1]->getY() - boundaries.at(j)[0]->getY()) * 1E-2};
                double vec[2] = {-v[1], v[0]};
                double middle[2] = {(boundaries.at(j)[1]->getX() + boundaries.at(j)[0]->getX()) / 2,
                                    (boundaries.at(j)[1]->getY() + boundaries.at(j)[0]->getY()) / 2};
                double* innerpoint = new double(2);
                innerpoint[0] = middle[0] - vec[0];
                innerpoint[1] = middle[1] - vec[1];

                holes.push_back(innerpoint);
            }
        }

        TriHelper::TriangleHelper helper(points, polylines, holes, false, true);
        std::vector<double*> generated_points = helper.getAddedPoints();


        std::vector<std::shared_ptr<Vertex> > newVertices;
        for(auto gpit = generated_points.begin(); gpit != generated_points.end(); gpit++)
        {
            std::vector<std::shared_ptr<Edge> > incident;
            std::shared_ptr<Vertex> v = meshes[0]->addNewVertex((*gpit)[0], (*gpit)[1], 0);
            v->setId(std::to_string(meshes[0]->getVerticesNumber() - 1));
            verticesEdges.insert(std::make_pair(v->getId(), incident));
            newVertices.push_back(v);
        }
        std::vector<uint> enclosing_triangles = helper.getTriangles();

        std::vector<std::shared_ptr<Vertex> > debugPoints;
        std::vector<std::tuple<uint, uint, uint> > debugTriangles;

        for(uint j = 0; j < enclosing_triangles.size() / 3; j++)
        {

            std::shared_ptr<Vertex> v1, v2, v3;

            if(enclosing_triangles[j * 3] < points.size())
            {
                v1 = pointsToVertices[enclosing_triangles[j * 3]];
            } else
            {
                v1 = newVertices.at(enclosing_triangles[j * 3] - points.size());
            }

            if(enclosing_triangles[j * 3 + 1] < points.size())
            {
                v2 = pointsToVertices[enclosing_triangles[j * 3 + 1]];
            } else
            {
                v2 = newVertices.at(enclosing_triangles[j * 3 + 1] - points.size());
            }

            if(enclosing_triangles[j * 3 + 2] < points.size())
            {
                v3 = pointsToVertices[enclosing_triangles[j * 3 + 2]];
            } else
            {
                v3 = newVertices.at(enclosing_triangles[j * 3 + 2] - points.size());
            }

            auto searchEdgeContainingVertex = [verticesEdges](std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2){
                std::shared_ptr<Edge> edge = nullptr;
                std::vector<std::shared_ptr<Edge> > list1 = verticesEdges.at(v1->getId());
                auto it1 = std::find_if(list1.begin(), list1.end(), [v2](std::shared_ptr<Edge> e) { return e->hasVertex(v2);});
                if(it1 != list1.end())
                    edge = *it1;
                else
                {
                    std::vector<std::shared_ptr<Edge> > list2 = verticesEdges.at(v2->getId());
                    auto it2 = std::find_if(list2.begin(), list2.end(), [v1](std::shared_ptr<Edge> e) { return e->hasVertex(v1);});
                    if(it2 != list2.end())
                        edge = *it2;
                }

                return edge;
            };

            std::shared_ptr<Edge> e1 = searchEdgeContainingVertex(v1, v2);
            if(e1 == nullptr)
            {
                e1 = meshes[0]->addNewEdge(v1, v2);
                e1->setId(std::to_string(meshes[0]->getEdgesNumber()));
                verticesEdges[v1->getId()].push_back(e1);
                verticesEdges[v2->getId()].push_back(e1);
                if(v1->getE0() == nullptr)
                    v1->setE0(e1);
                if(v2->getE0() == nullptr)
                    v2->setE0(e1);
            }
            std::shared_ptr<Edge> e2 = searchEdgeContainingVertex(v2,v3);
            if(e2 == nullptr)
            {
                e2 = meshes[0]->addNewEdge(v2, v3);
                e2->setId(std::to_string(meshes[0]->getEdgesNumber()));
                verticesEdges[v2->getId()].push_back(e2);
                verticesEdges[v3->getId()].push_back(e2);
                if(v2->getE0() == nullptr)
                    v2->setE0(e2);
                if(v3->getE0() == nullptr)
                    v3->setE0(e2);
            }
            std::shared_ptr<Edge> e3 = searchEdgeContainingVertex(v3, v1);
            if(e3 == nullptr)
            {
                e3 = meshes[0]->addNewEdge(v3, v1);
                e3->setId(std::to_string(meshes[0]->getEdgesNumber()));
                verticesEdges[v3->getId()].push_back(e3);
                verticesEdges[v1->getId()].push_back(e3);
                if(v3->getE0() == nullptr)
                    v3->setE0(e3);
                if(v1->getE0() == nullptr)
                    v1->setE0(e3);
            }

            std::shared_ptr<Triangle> t = meshes[0]->addNewTriangle(e1, e2, e3);
            if(e1->getT1() == nullptr)
                e1->setT1(t);
            else if(e1->getT2() == nullptr)
                e1->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;

                meshes[0]->save("witherror.ply", 15);
                std::cerr << "Vertices: " << std::endl;
                e1->getV1()->print(std::cerr);
                e1->getV2()->print(std::cerr);
                std::cerr << "v1=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v1)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v1(:,1),v1(:,2),'o'))" << std::endl;
                std::cerr << "v2=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v2)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v2(:,1),v2(:,2),'o'))" << std::endl;
                std::cerr << "v3=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v3)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v3(:,1),v3(:,2),'o'))" << std::endl;
                uint counter = 0;
                std::cerr <<"%New building" << std::endl;
                const auto& boundaries = building->getOutlines();
                for(const auto& boundary : boundaries)
                {
                    std::cerr << "B" << counter << "=[" << std::endl;
                    for(const auto& p : boundary)
                    {
                        std::static_pointer_cast<SemantisedTriangleMesh::Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                    }
                    std::cerr << "];" << std::endl;
                    std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                }


                e1->getT1()->print(std::cerr);
                e1->getT2()->print(std::cerr);
                e1->getT1()->getV1()->print(std::cerr);
                e1->getT1()->getV2()->print(std::cerr);
                e1->getT1()->getV3()->print(std::cerr);
                e1->getT2()->getV1()->print(std::cerr);
                e1->getT2()->getV2()->print(std::cerr);
                e1->getT2()->getV3()->print(std::cerr);

                exit(123);
            }

            if(e2->getT1() == nullptr)
                e2->setT1(t);
            else if(e2->getT2() == nullptr)
                e2->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;

                meshes[0]->save("witherror.ply", 15);

                std::cerr << "Vertices: " << std::endl;
                e1->getV1()->print(std::cerr);
                e1->getV2()->print(std::cerr);
                std::cerr << "v1=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v1)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v1(:,1),v1(:,2),'o'))" << std::endl;
                std::cerr << "v2=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v2)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v2(:,1),v2(:,2),'o'))" << std::endl;
                std::cerr << "v3=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v3)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v3(:,1),v3(:,2),'o'))" << std::endl;
                uint counter = 0;
                std::cerr <<"%New building" << std::endl;
                auto boundaries = building->getOutlines();
                for(const auto &boundary : boundaries)
                {
                    std::cerr << "B" << counter << "=[" << std::endl;
                    for(const auto &p : boundary)
                    {
                        std::static_pointer_cast<SemantisedTriangleMesh::Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                    }
                    std::cerr << "];" << std::endl;
                    std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                }
                std::cerr << "Structure" << std::endl;
                for_each(points.begin(), points.end(), [](double* p) { std::cerr << p[0] << " " << p[1] << std::endl;});
                for_each(enclosing_triangles.begin(), enclosing_triangles.end(), [](uint id) { std::cerr << id << std::endl;});
                std::cerr << "Triangle:" << std::endl;
                std::cerr << points[enclosing_triangles[j * 3]][0] << " " << points[enclosing_triangles[j * 3]][1] << std::endl;
                std::cerr << points[enclosing_triangles[j * 3 + 1]][0] << " " << points[enclosing_triangles[j * 3 + 1]][1] << std::endl;
                std::cerr << points[enclosing_triangles[j * 3 + 2]][0] << " " << points[enclosing_triangles[j * 3 + 2]][1] << std::endl;
                exit(123);
            }

            if(e3->getT1() == nullptr)
                e3->setT1(t);
            else if(e3->getT2() == nullptr)
                e3->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;

                meshes[0]->save("witherror.ply", 15);
                std::cerr << "Vertices: " << std::endl;
                e1->getV1()->print(std::cerr);
                e1->getV2()->print(std::cerr);
                std::cerr << "v1=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v1)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v1(:,1),v1(:,2),'o'))" << std::endl;
                std::cerr << "v2=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v2)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v2(:,1),v2(:,2),'o'))" << std::endl;
                std::cerr << "v3=";
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(v3)->print(std::cerr, BracketsType::SQUARE, " ");
                std::cerr << "plot(v3(:,1),v3(:,2),'o'))" << std::endl;
                uint counter = 0;
                std::cerr <<"%New building" << std::endl;
                auto boundaries = building->getOutlines();
                for(auto boundary : boundaries)
                {
                    std::cerr << "B" << counter << "=[" << std::endl;
                    for(auto p : boundary)
                    {
                        std::static_pointer_cast<SemantisedTriangleMesh::Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                    }
                    std::cerr << "];" << std::endl;
                    std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                }


                std::cerr << "%Triangulation holes" << std::endl;
                std::cerr << "figure; hold all;" << std::endl;

                exit(123);
            }

        }

    }

    for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
    {
        std::shared_ptr<Vertex> v = meshes[0]->getVertex(i);
        v->setX(v->getX() * city_size + origin->getX());
        v->setY(v->getY() * city_size + origin->getY());
        v->setZ(v->getZ() * city_size + origin->getZ());
    }

    std::cout << "Ended" << std::endl << "Adapting elevations to DTM" << std::endl;

    origin.reset();

    meshes[0]->save("prova.ply", 15);
    std::cout << "Ended" << std::endl << "Associating elevations" << std::endl;

    for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        meshes[0]->getVertex(i)->clearFlags();

    for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
    {
        auto v = meshes[0]->getVertex(i);
        if(v->getZ() == 0.0)
        {
            uint size = 1, counter = 0;
            double mean = 0;

            std::vector<std::shared_ptr<Vertex>> used = {v};
            std::queue<std::shared_ptr<Vertex>> queue;
            queue.push(v);
            v->addFlag(FlagType::USED);
            uint level = 0;
            do
            {
                auto v_ = queue.front();
                queue.pop();

                std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > neighbourhood;
                auto oneRing = v_->getVV();
                for(const auto& vTMP : oneRing)
                    if(vTMP->searchFlag(FlagType::USED) == -1)
                    {
                        vTMP->addFlag(FlagType::USED);
                        used.push_back(vTMP);
                        queue.push(vTMP);
                        neighbourhood.push_back(vTMP);
                    }

                for(auto &vTMP : neighbourhood)
                {
                    if(vTMP->getZ() != 0.0)
                    {
                        mean += vTMP->getZ();
                        counter++;
                    }
                }
                if(counter != 0)
                    mean /= counter;
                level++;
            } while(counter == 0 && level < 100);
            for(const auto& v_ : used)
                v_->clearFlags();
            v->setZ(mean);

        }
    }
    for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        meshes[0]->getVertex(i)->clearFlags();

    std::cout << "Associating heights to buildings." << std::endl;
    for(uint i = 0; i < buildings.size(); i++)
    {
        int row_min = -1, col_min = -1, row_max = 0, col_max = 0;
        double elevation_min = std::numeric_limits<double>::max(),
               elevation_max = -std::numeric_limits<double>::max(),
               elevation_mean = 0;

        auto boundaries = buildings.at(i)->getOutlines();
        uint boundariesPointsNumber = 0;
        for(uint j = 0; j < boundaries.size(); j++)
        {
            for(uint k = 0; k < boundaries.at(j).size() - 1; k++)
            {
                double d = boundaries.at(j).at(k)->getZ();
                elevation_mean += d;
                if(elevation_min > d)
                    elevation_min = d;
                if(elevation_max < d)
                    elevation_max = d;
            }
            boundariesPointsNumber += boundaries.at(j).size() - 1;
        }

        elevation_mean /= boundariesPointsNumber;

        boundaries = buildings.at(i)->getOutlines();
        for(uint j = 0; j < boundaries.size(); j++)
            for(uint k = 0; k < boundaries.at(j).size() - 1; k++)
                boundaries.at(j).at(k)->setZ(elevation_max);
    }
    std::cout <<"Ended!" << std::endl;

    meshes[0]->save("cacca.ply",15);

    std::cout <<"Creating annotations." << std::endl;
    unsigned char building_color[] = {255, 215, 0};
    unsigned char street_color[] = {0, 0, 0};

    std::map<Node*, std::shared_ptr<Vertex> > traversed_nodes;

    uint i = 0;
    for(auto ait = streetsArcs.begin(); ait != streetsArcs.end(); ait++)
    {
        std::shared_ptr<LineAnnotation> annotation = std::make_shared<LineAnnotation>();
        std::vector<std::shared_ptr<Vertex> > annotationPolyline;
        annotation->setId(std::to_string(i));
        annotation->setTag("street n° " + std::to_string(i));
        annotation->setColor(street_color);
        std::vector<uint> flaggedMeshVertices;
        double epsilon = 1e-5;

        for(uint j = 1; j < arcsPoints.at(i).size(); j++)
        {
            auto v1 = arcsPoints.at(i).at(j - 1);
            auto v2 = arcsPoints.at(i).at(j);

            auto p = ((*v1) + (*v2)) / 2;
            auto point = {p.getX(), p.getY(), p.getZ()};
            auto neighbourhood = meshes[0]->getNearestNeighbours(p, (*v2 - *v1).norm() / 2 + SemantisedTriangleMesh::Point::EPSILON);
            for(auto pi : neighbourhood)
            {
                pi->addFlag(FlagType::INSIDE);
                flaggedMeshVertices.push_back(std::stoi(pi->getId()));
            }

        }

        for(uint j = 1; j < arcsPoints.at(i).size(); j++)
        {
            auto v1 = arcsPoints.at(i).at(j - 1);
            auto v2 = arcsPoints.at(i).at(j);
            auto path = this->meshes[0]->getVerticesCloseToLine(*v1, *v2);
            double dist = 0.0;
            for(uint k = 1; k < path.size(); k++)
                dist += (*path[k] - *path[k - 1]).norm();
            if(path.size() > 1 && dist < 2 * (*v2 - *v1).norm())
            {
                path.pop_back();
                arcsPoints.at(i).insert(arcsPoints.at(i).begin() + j, path.begin(), path.end());
                j += path.size();
            }
        }

        for(auto p : flaggedMeshVertices)
            meshes[0]->getVertex(p)->removeFlag(FlagType::INSIDE);

        for(uint j = 0; j < arcsPoints.at(i).size(); j++)
        {
            std::shared_ptr<Vertex> v = std::static_pointer_cast<Vertex>(arcsPoints.at(i).at(j));
            uint id = static_cast<uint>(stoi(v->getId()));
            if(arcsPoints.at(i).at(j)->getInfo() != nullptr)
            {
                Node* n = static_cast<Node*>(arcsPoints.at(i).at(j)->getInfo());
                if(traversed_nodes.find(n) == traversed_nodes.end())
                    traversed_nodes.insert(std::make_pair(n, v));
            }
            if(id > meshes[0]->getVerticesNumber())
                std::cout << "Found deleted vertex in street: " << i << " " << j << std::endl;
            else
            {
                annotationPolyline.push_back(v);
            }
        }
        annotation->addPolyLine(annotationPolyline);

        std::cout << i * 100 / streetsArcs.size() << "%\r" << std::flush;
        i++;
        std::shared_ptr<SemanticAttribute> attribute = std::make_shared<SemanticAttribute>();
        attribute->setId(0);
        attribute->setKey("osmid");
        attribute->setValue(ait->c_str());
        annotation->addAttribute(attribute);

        annotation->setMesh(meshes[0]);
        meshes[0]->addAnnotation(annotation);

    }

    meshes[0]->save("cacca2.ply", 15);
    i = 0;
    for(auto it = traversed_nodes.begin(); it != traversed_nodes.end(); it++)
    {

        std::shared_ptr<PointAnnotation> annotation = std::make_shared<PointAnnotation>();
        annotation->setId(std::to_string((streetsArcs.size() - 1 + i)));
        annotation->setTag("node n° " + std::to_string(i++));
        annotation->setColor(street_color);
        annotation->addPoint(it->second);

        std::shared_ptr<SemanticAttribute> attribute = std::make_shared<SemanticAttribute>();
        attribute->setId(0);
        attribute->setKey("osmid");
        attribute->setValue(it->first->getId());
        annotation->addAttribute(attribute);

        annotation->setMesh(meshes[0]);
        meshes[0]->addAnnotation(annotation);

    }
    meshes[0]->save("cacca3.ply", 15);

    uint bid = streetsArcs.size() + traversed_nodes.size();
    uint counter=0;
    for(uint i = 0; i < buildings.size(); i++)
    {
        auto boundaries = buildings[i]->getOutlines();
        std::shared_ptr<SurfaceAnnotation> annotation = std::make_shared<SurfaceAnnotation>();
        annotation->setId(std::to_string(bid));
        annotation->setTag("building n° " + std::to_string(bid++));
        annotation->setColor(building_color);

        annotation->setOutlines(boundaries);

        annotation->setMesh(meshes[0]);
        meshes[0]->addAnnotation(annotation);
        auto height = boundaries[0][0]->getZ();

        auto involved = annotation->getInvolvedVertices();

        for(uint j = 0; j < involved.size(); j++)
            involved.at(j)->setZ(height);
    }
    meshes[0]->save("cacca4.ply", 15);

    for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        meshes[0]->getVertex(i)->setId(std::to_string(i));
    for(uint i = 0; i < meshes[0]->getEdgesNumber(); i++)
        meshes[0]->getEdge(i)->setId(std::to_string(i));
    for(uint i = 0; i < meshes[0]->getTrianglesNumber(); i++)
        meshes[0]->getTriangle(i)->setId(std::to_string(i));
    SemantisedTriangleMesh::SemanticsFileManager manager;
    manager.setMesh(meshes[0]);
    manager.writeAnnotations("annotations.ant");
    meshes[0]->save("level0.ply", 15);

    return 0;
}

int CityGMLCore::buildLevel1() {

    std::cout << "Building level 1" << std::endl;
    //float** heights = dsm->GetRasterBand(1);
    this->meshes[1] = std::make_shared<TriangleMesh>(this->meshes[0]);
    this->meshes[0].reset();
    lidarDTMPoints.clear();
    auto heights = extractBuildingsHeightsFromLidar();
    //Questa cosa va risolta assolutamente
    for(uint i = 0; i < this->meshes[1]->getAnnotations().size(); i++)
        meshes[1]->getAnnotations().at(i)->setMesh(meshes[1]);

    uint counter = 0, annotationPos = 0;
    for(uint i = 0; i < buildings.size(); i++)
    {
        auto boundaries = buildings[i]->getOutlines();

        //int row_min = -1, col_min = -1, row_max = 0, col_max = 0;
        double height_min = std::numeric_limits<double>::max(),
               height_max = -std::numeric_limits<double>::max(),
               height_mean = 0;
        //Computing min-max-mean height values

        for(uint j = 0; j < heights.at(i).size(); j++)
        {
            height_mean += heights.at(i).at(j);
            if(height_min > heights.at(i).at(j))
                height_min = heights.at(i).at(j);

            if(height_max < heights.at(i).at(j))
                height_max = heights.at(i).at(j);

        }
        height_mean /= heights.at(i).size();

        std::shared_ptr<Annotation> annotation;
        std::string tag;
        do
        {
            annotation = meshes[1]->getAnnotation(annotationPos++);
            tag = annotation->getTag();
            std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c) { return std::tolower(c); });
        } while (tag.find("building") == std::string::npos);


        std::shared_ptr<SurfaceAnnotation> buildingAnnotation = std::dynamic_pointer_cast<SurfaceAnnotation>(annotation); //Non proprio l'ideale
        auto outlines = buildingAnnotation->getOutlines();
        auto triangles = buildingAnnotation->getTriangles();


        for(auto it = triangles.begin(); it != triangles.end(); it++)
//        {
            (*it)->addFlag(FlagType::TO_BE_REMOVED);
//        }

        std::vector<std::vector<std::shared_ptr<Vertex> > > cloned_outlines;
        std::map<uint, std::vector<std::shared_ptr<Edge> > > vertices_edges;

        std::vector<double*> points;
        std::vector<double*> holes;
        std::vector<std::vector<uint> > polylines;
        std::map<uint, std::shared_ptr<Vertex> > pointsToVertices;
        uint reached_id = 0;
        std::vector<std::vector<std::shared_ptr<Triangle> > > boundaryTriangles;

        for(uint j = 0; j < outlines.size(); j++)
        {
            std::vector<std::shared_ptr<Vertex> > cloned_outline;
            std::vector<std::shared_ptr<Triangle> > boundaryTrianglesJ;
            std::vector<uint> polyline;
            uint first = reached_id;
            for(uint k = 0; k < outlines.at(j).size() - 1; k++)
            {
                auto v = outlines.at(j).at(k);
                cloned_outline.push_back(meshes[1]->addNewVertex(*std::static_pointer_cast<SemantisedTriangleMesh::Point>(v)));
                cloned_outline.back()->setId(std::to_string(meshes[1]->getVerticesNumber() - 1));
                cloned_outline.back()->setZ(height_mean);
                vertices_edges.insert(std::make_pair(stoi(v->getId()), v->getVE() ));
                std::vector<std::shared_ptr<Edge> > connected_list;
                vertices_edges.insert(std::make_pair(meshes[1]->getVerticesNumber() - 1, connected_list));
                points.push_back(cloned_outline[k]->toDoubleArray());
                pointsToVertices.insert(std::make_pair(reached_id, cloned_outline[k]));
                polyline.push_back(reached_id++);
                boundaryTrianglesJ.push_back(v->getCommonEdge(outlines.at(j).at(k + 1))->getLeftTriangle(v));
            }
            polyline.push_back(first);
            polylines.push_back(polyline);
            cloned_outline.push_back(cloned_outline.at(0));
            cloned_outlines.push_back(cloned_outline);
            boundaryTriangles.push_back(boundaryTrianglesJ);
            if(j > 0)
            {
                double v[2] = {(cloned_outlines.at(j)[1]->getX() - cloned_outlines.at(j)[0]->getX()) * 1E-2,
                               (cloned_outlines.at(j)[1]->getY() - cloned_outlines.at(j)[0]->getY()) * 1E-2};
                double vec[2] = {-v[1], v[0]};
                double middle[2] = {(cloned_outlines.at(j)[1]->getX() + cloned_outlines.at(j)[0]->getX()) / 2,
                                    (cloned_outlines.at(j)[1]->getY() + cloned_outlines.at(j)[0]->getY()) / 2};
                double* innerpoint = new double(2);
                innerpoint[0] = middle[0] - vec[0];
                innerpoint[1] = middle[1] - vec[1];

                holes.push_back(innerpoint);
            }
        }

        for(uint j = 0; j < outlines.size(); j++)
        {

            for(uint k = 0; k < outlines.at(j).size() - 1; k++)
            {

                unsigned int pos1 = k;
                unsigned int pos2 = k + 1;
                std::shared_ptr<Vertex> v1 = std::static_pointer_cast<Vertex>(outlines.at(j).at(pos1));
                std::shared_ptr<Vertex> v2 = std::static_pointer_cast<Vertex>(outlines.at(j).at(pos2));
                std::shared_ptr<Vertex> v3 = std::static_pointer_cast<Vertex>(cloned_outlines.at(j).at(pos1));
                std::shared_ptr<Vertex> v4 = std::static_pointer_cast<Vertex>(cloned_outlines.at(j).at(pos2));

                std::shared_ptr<Edge> e1 = meshes[1]->searchEdgeContainingVertex(vertices_edges.at(stoi(v1->getId())), v2);
                if(e1 == nullptr)
                {
                    for(uint k = 0; k < outlines.at(j).size(); k++)
                        std::static_pointer_cast<SemantisedTriangleMesh::Point>(outlines.at(j).at(k))->print(std::cout, BracketsType::NONE, " ");
                    std::cout << std::endl;
                    v1->print(std::cout);
                    v2->print(std::cout);
                    v3->print(std::cout);
                    v4->print(std::cout);
                    std::cout << std::endl;
                    std::static_pointer_cast<SemantisedTriangleMesh::Point>(v1)->print(std::cout, BracketsType::NONE, " ");
                    std::static_pointer_cast<SemantisedTriangleMesh::Point>(v2)->print(std::cout, BracketsType::NONE, " ");
                    std::static_pointer_cast<SemantisedTriangleMesh::Point>(v3)->print(std::cout, BracketsType::NONE, " ");
                    std::static_pointer_cast<SemantisedTriangleMesh::Point>(v4)->print(std::cout, BracketsType::NONE, " ");
                }
                std::shared_ptr<Edge> e2 = meshes[1]->addNewEdge(v2, v3);
                e2->setId(std::to_string(meshes[1]->getEdgesNumber() - 1));

                std::shared_ptr<Edge> e3 = meshes[1]->searchEdgeContainingVertex(vertices_edges.at(stoi(v1->getId())), v3);
                if(e3 == nullptr)
                {
                    e3 = meshes[1]->addNewEdge(v3, v1);
                    e3->setId(std::to_string(meshes[1]->getEdgesNumber() - 1));
                }

                std::shared_ptr<Edge> e4 = meshes[1]->searchEdgeContainingVertex(vertices_edges.at(stoi(v2->getId())), v4);
                if(e4 == nullptr)
                {
                    e4 = meshes[1]->addNewEdge(v2, v4);
                    e4->setId(std::to_string(meshes[1]->getEdgesNumber() - 1));
                }

                std::shared_ptr<Edge> e5 = meshes[1]->addNewEdge(v4, v3);
                e5->setId(std::to_string(meshes[1]->getEdgesNumber() - 1));

                std::shared_ptr<Triangle> t1 = meshes[1]->addNewTriangle(e1, e2, e3);
                t1->setId(std::to_string(meshes[1]->getTrianglesNumber() - 1));
                std::shared_ptr<Triangle> t2 = meshes[1]->addNewTriangle(e2, e4, e5);
                t2->setId(std::to_string(meshes[1]->getTrianglesNumber() - 1));

                if(v3->getE0() == nullptr)
                    v3->setE0(e2);
                if(v4->getE0() == nullptr)
                    v4->setE0(e4);

                e2->setT1(t1);
                e2->setT2(t2);

                if(e3->getT1() == nullptr)
                    e3->setT1(t1);
                else if(e3->getT2() == nullptr)
                    e3->setT2(t1);
                else
                    std::cout << "Found" << std::endl;

                if(e4->getT1() == nullptr)
                    e4->setT1(t2);
                else if(e4->getT2() == nullptr)
                    e4->setT2(t2);
                else
                    std::cout << "Found" << std::endl;

                e5->setT1(t2);
                e1->setTriangle(boundaryTriangles.at(j).at(k), t1);
                v1->setE0(e1);
                v2->setE0(e1);
                vertices_edges.at(stoi(v1->getId())).push_back(e3);
                vertices_edges.at(stoi(v2->getId())).push_back(e2);
                vertices_edges.at(stoi(v2->getId())).push_back(e4);
                vertices_edges.at(stoi(v3->getId())).push_back(e2);
                vertices_edges.at(stoi(v3->getId())).push_back(e3);
                vertices_edges.at(stoi(v3->getId())).push_back(e5);
                vertices_edges.at(stoi(v4->getId())).push_back(e4);
                vertices_edges.at(stoi(v4->getId())).push_back(e5);
            }

        }


        TriHelper::TriangleHelper helper(points, polylines, holes, false);
        std::vector<double*> generated_points = helper.getAddedPoints();

        std::vector<std::shared_ptr<Vertex> > newVertices;
        for(auto gpit = generated_points.begin(); gpit != generated_points.end(); gpit++)
        {
            std::vector<std::shared_ptr<Edge> > incident;
            std::shared_ptr<Vertex> v = meshes[1]->addNewVertex((*gpit)[0], (*gpit)[1], 0);
            v->setId(std::to_string(meshes[1]->getVerticesNumber() - 1));
            pointsToVertices.insert(std::make_pair(points.size() + (gpit - generated_points.begin()), v));
            vertices_edges.insert(std::make_pair(stoi(v->getId()), incident));
            newVertices.push_back(v);
        }
        std::vector<uint> enclosing_triangles = helper.getTriangles();

        for(uint j = 0; j < enclosing_triangles.size() / 3; j++)
        {

            std::shared_ptr<Vertex> v1, v2, v3;
            if(enclosing_triangles[j * 3] < points.size())
            {
                v1 = pointsToVertices[enclosing_triangles[j * 3]];
            } else
            {
                v1 = newVertices.at(enclosing_triangles[j * 3] - points.size());
            }

            if(enclosing_triangles[j * 3 + 1] < points.size())
            {
                v2 = pointsToVertices[enclosing_triangles[j * 3 + 1]];
            } else
            {
                v2 = newVertices.at(enclosing_triangles[j * 3 + 1] - points.size());
            }

            if(enclosing_triangles[j * 3 + 2] < points.size())
            {
                v3 = pointsToVertices[enclosing_triangles[j * 3 + 2]];
            } else
            {
                v3 = newVertices.at(enclosing_triangles[j * 3 + 2] - points.size());
            }

            auto searchEdgeContainingVertex = [vertices_edges](std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2){
                std::shared_ptr<Edge> edge = nullptr;
                std::vector<std::shared_ptr<Edge> > list1 = vertices_edges.at(stoi(v1->getId()));
                auto it1 = std::find_if(list1.begin(), list1.end(), [v2](std::shared_ptr<Edge> e) { return e->hasVertex(v2);});
                if(it1 != list1.end())
                    edge = *it1;
                else
                {
                    std::vector<std::shared_ptr<Edge> > list2 = vertices_edges.at(stoi(v2->getId()));
                    auto it2 = std::find_if(list2.begin(), list2.end(), [v1](std::shared_ptr<Edge> e) { return e->hasVertex(v1);});
                    if(it2 != list2.end())
                        edge = *it2;
                }

                return edge;
            };

            std::shared_ptr<Edge> e1 = searchEdgeContainingVertex(v1, v2);
            if(e1 == nullptr)
            {
                e1 = meshes[1]->addNewEdge(v1, v2);
                e1->setId(std::to_string(meshes[1]->getEdgesNumber()));
                vertices_edges[stoi(v1->getId())].push_back(e1);
                vertices_edges[stoi(v2->getId())].push_back(e1);
                v1->setE0(e1);
                v2->setE0(e1);
            }
            std::shared_ptr<Edge> e2 = searchEdgeContainingVertex(v2,v3);
            if(e2 == nullptr)
            {
                e2 = meshes[1]->addNewEdge(v2, v3);
                e2->setId(std::to_string(meshes[1]->getEdgesNumber()));
                vertices_edges[stoi(v2->getId())].push_back(e2);
                vertices_edges[stoi(v3->getId())].push_back(e2);
                v2->setE0(e2);
                v3->setE0(e2);
            }
            std::shared_ptr<Edge> e3 = searchEdgeContainingVertex(v3, v1);
            if(e3 == nullptr)
            {
                e3 = meshes[1]->addNewEdge(v3, v1);
                e3->setId(std::to_string(meshes[1]->getEdgesNumber()));
                vertices_edges[stoi(v3->getId())].push_back(e3);
                vertices_edges[stoi(v1->getId())].push_back(e3);
                v3->setE0(e3);
                v1->setE0(e3);
            }

            std::shared_ptr<Triangle> t = meshes[1]->addNewTriangle(e1, e2, e3);

            if(e1->getT1() == nullptr)
                e1->setT1(t);
            else if(e1->getT2() == nullptr)
                e1->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;
                meshes[1]->save("witherror.ply",15);
                exit(123);
            }

            if(e2->getT1() == nullptr)
                e2->setT1(t);
            else if(e2->getT2() == nullptr)
                e2->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;
                meshes[1]->save("witherror.ply",15);
                exit(123);
            }

            if(e3->getT1() == nullptr)
                e3->setT1(t);
            else if(e3->getT2() == nullptr)
                e3->setT2(t);
            else
            {
                std::cerr << "Non manifold configuration!" << std::endl;
                meshes[1]->save("witherror.ply",15);
                exit(123);
            }



        }
//        tid = 0;
//        std::cout << "figure; hold all;" << std::endl;
        for(auto it = triangles.begin(); it != triangles.end(); it++)
        {
            (*it)->addFlag(FlagType::TO_BE_REMOVED);
//            std::cout << "T" << tid << "=[" << std::endl;
//            std::static_pointer_cast<Point>((*it)->getV1())->print(std::cout, BracketsType::NONE, " ");
//            std::static_pointer_cast<Point>((*it)->getV2())->print(std::cout, BracketsType::NONE, " ");
//            std::static_pointer_cast<Point>((*it)->getV3())->print(std::cout, BracketsType::NONE, " ");
//            std::static_pointer_cast<Point>((*it)->getV1())->print(std::cout, BracketsType::NONE, " ");
//            std::cout << "];" << std::endl;
//            std::cout << "plot(T" << tid << "(:,1), T" << tid << "(:,2));" << std::endl;
//            std::cout << "labels=[" << (*it)->getV1()->getId() << " " <<
//                                       (*it)->getV2()->getId() << " " <<
//                                       (*it)->getV3()->getId() << " " <<
//                                       (*it)->getV1()->getId() << "];" << std::endl;
//            std::cout << "labels=strsplit(num2str(labels));" << std::endl;
//            std::cout << "text(T" << tid << "(:,1), T" << tid++ << "(:,2), labels);" << std::endl;
//            meshes[1]->removeTriangle((*it)->getId());
        }

//        meshes[1]->save(std::to_string(i).append(".ply"), 15);

    }

    meshes[1]->removeFlaggedTriangles();
    meshes[1]->removeIsolatedVertices();
    for(uint i = 0; i < meshes[1]->getVerticesNumber(); i++)
        meshes[1]->getVertex(i)->setId(std::to_string(i));
    for(uint i = 0; i < meshes[1]->getEdgesNumber(); i++)
        meshes[1]->getEdge(i)->setId(std::to_string(i));
    for(uint i = 0; i < meshes[1]->getTrianglesNumber(); i++)
        meshes[1]->getTriangle(i)->setId(std::to_string(i));
    meshes[1]->save("level1.ply", 15);
    std::vector<std::shared_ptr<Annotation> > buildingsAnnotations;
    uint pos = 0;
    SemanticsFileManager manager;
    manager.setMesh(meshes[1]);
    manager.writeAnnotations("AnnotationsLOD1.ant");

    std::vector<std::shared_ptr<Annotation> > annotations = meshes[1]->getAnnotations();
    for(uint i = 0; i < meshes[1]->getAnnotations().size(); )
    {
        auto annotation = meshes[1]->getAnnotations().at(i);
        auto tag = annotation->getTag();
        std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c){return std::tolower(c); });
        if(tag.find("building") != std::string::npos)
        {
            buildingsAnnotations.push_back(annotation);
            std::cout << meshes[1]->getAnnotations().size() << std::endl;
            meshes[1]->removeAnnotation(i);
        } else
            i++;

    }

    manager.writeAnnotations("streetsLOD1.ant");
    meshes[1]->setAnnotations(buildingsAnnotations);
    manager.writeAnnotations("buildingsLOD1.ant");
    return 0;
}

void CityGMLCore::setLevel0(std::string meshFileName, std::string annotationFileName)
{

    uint reachedId = 0;
    this->meshes[0] = std::make_shared<TriangleMesh>();
    this->meshes[0]->load(meshFileName);
    SemanticsFileManager manager;
    manager.setMesh(this->meshes[0]);
    manager.readAnnotations(annotationFileName);

    uint counter = 0;
    for(auto annotation : meshes[0]->getAnnotations())
    {
        auto tag = annotation->getTag();
        std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c) { return std::tolower(c); });
        if(tag.find("building") != std::string::npos)
        {
            auto buildingAnnotation = std::dynamic_pointer_cast<SurfaceAnnotation>(annotation);
            auto building = std::make_shared<Building>();
            building->setOutlines(buildingAnnotation->getOutlines());
            building->setId(std::to_string(reachedId++));
            buildings.push_back(building);
        }
    }
    //pixelToBuildingAssociation = extractBuildingsHeights(true);
}

void CityGMLCore::setLevel1(std::string meshFileName, std::string annotationFileName)
{

}
