#include <citygmlcore.h>

#include <trianglehelper1.h>
#include <utilities.h>
#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <semanticattribute.h>
#include <tinyxml2.h>
#include <coordsconverter.h>
#include <KDTree.hpp>
#include <annotationfilemanager.h>

#include <cctype>
#include <algorithm>
#include <buildingsgroup.h>
#include <tuple>
#include <liblas/liblas.hpp>
#include <shapefil.h>

#include <chrono>
using namespace std::chrono;
#define MATERA_EPSG_CODE 32633

VertexList pointToVertexList(PointList points)
{
    VertexList vertices;
    for(auto p : points)
        vertices.push_back(std::static_pointer_cast<Vertex>(p));
    return vertices;
}

PointList vertexToPointList(VertexList vertices)
{
    PointList points;
    for(auto v : vertices)
        points.push_back(std::static_pointer_cast<Point>(v));
    return points;
}


CityGMLCore::CityGMLCore()
{
    osmIsLoaded = false;
    dtmIsLoaded = false;
    dsmIsLoaded = false;
    osmFilename = "";
    dtmFilename = "";
    dsmFilename = "";
}

CityGMLCore::CityGMLCore(std::string osmFilename, std::string dtmFilename, std::string dsmFilename, std::string lidarFilename, std::string boundsFile)
{
    this->osmFilename = osmFilename;
    this->dtmFilename = dtmFilename;
    this->dsmFilename = dsmFilename;

    int retValue = loadOSM(osmFilename, nodes, arcs, relations);
    if(retValue == 0)
        osmIsLoaded = true;
    else
        std::cerr << "Error loading OSM file" << std::endl;

    std::cout << "Loading DTM" << std::endl;

    dtm = std::make_shared<GeoTiff>(dtmFilename.c_str());
    if(dtm != nullptr)
        dtmIsLoaded = true;
    else
        std::cerr << "Error loading DTM file" << std::endl;

    std::cout << "Loading DSM" << std::endl;
    dsm = std::make_shared<GeoTiff>(dsmFilename.c_str());
    if(dsm != nullptr)
        dsmIsLoaded = true;
    else
        std::cerr << "Error loading DSM file" << std::endl;


    Utilities::load_shapefile_shp(boundsFile, bounds);
    int retValue_ = readLiDAR(lidarFilename);
}

int CityGMLCore::loadOSM(std::string filename, std::map<std::string, std::shared_ptr<OSMNode> > &nodes,
            std::map<std::string, std::shared_ptr<OSMWay> > &ways, std::map<std::string, std::shared_ptr<OSMRelation> > &relations)
{
    if(filename.compare("") == 0)
        return 1;
    tinyxml2::XMLDocument document;
    document.LoadFile(filename.c_str());

    tinyxml2::XMLElement* osmRoot = document.FirstChildElement();
    tinyxml2::XMLElement* xmlNode = osmRoot->FirstChildElement("node");
    CoordsConverter converter(MATERA_EPSG_CODE);

    while(xmlNode != nullptr){

        std::shared_ptr<OSMNode> node = std::make_shared<OSMNode>();
        std::vector<std::pair<std::string, std::string> > tags;
        const char* id = xmlNode->Attribute("id");
        if(id == nullptr)
            return -1;
        node->setId(id);
        const char* userName = xmlNode->Attribute("user");
        if(userName == nullptr)
            return -2;
        node->setUserName(userName);
        const char* userId = xmlNode->Attribute("uid");
        if(userId == nullptr)
            return -3;
        node->setUserId(userId);
        const char* latitude = xmlNode->Attribute("lat");
        const char* longitude = xmlNode->Attribute("lon");
        if(latitude == nullptr || longitude == nullptr)
            return -4;


        double lat = strtod(latitude, nullptr), lon = strtod(longitude, nullptr), x, y;
        converter.convertToUTM(lat, lon, x, y);
        std::shared_ptr<Point> coordinates = std::make_shared<Point>(x, y, 0);
        coordinates->setInfo(static_cast<void*>(node.get()));
        node->setCoordinates(coordinates);

        const char* isVisible = xmlNode->Attribute("visible");
        if(isVisible == nullptr)
            return -5;
        node->setIsVisible(strcmp(isVisible, "true") == 0);

        const char* version = xmlNode->Attribute("version");
        if(version == nullptr)
            return -6;
        node->setVersion(version);

        const char* changeset = xmlNode->Attribute("changeset");
        if(changeset == nullptr)
            return -7;
        node->setChangeset(changeset);


        const char* timestamp = xmlNode->Attribute("timestamp");
        if(timestamp == nullptr)
            return -8;
        node->setTimestamp(timestamp);

        tinyxml2::XMLElement* xmlTag = xmlNode->FirstChildElement("tag");
        while(xmlTag != nullptr){
            const char* key = xmlTag->Attribute("k");
            const char* value = xmlTag->Attribute("v");
            if(key == nullptr || value == nullptr)
                return -8;

            tags.push_back(std::make_pair(key, value));
            xmlTag = xmlTag->NextSiblingElement("tag");
        }

        node->setTags(tags);
        xmlNode = xmlNode->NextSiblingElement("node");

        nodes.insert(std::make_pair(node->getId(), node));
    }

    tinyxml2::XMLElement* xmlWay = osmRoot->FirstChildElement("way");

    while(xmlWay != nullptr){

        std::shared_ptr<OSMWay> way = std::make_shared<OSMWay>();
        std::vector<std::pair<std::string, std::string> > tags;
        std::vector<std::shared_ptr<OSMNode> > connected;
        const char* id = xmlWay->Attribute("id");
        if(id == nullptr)
            return -9;
        way->setId(id);
        const char* userName = xmlWay->Attribute("user");
        if(userName == nullptr)
            return -10;
        way->setUserName(userName);
        const char* userId = xmlWay->Attribute("uid");
        if(userId == nullptr)
            return -11;
        way->setUserId(userId);

        const char* isVisible = xmlWay->Attribute("visible");
        if(isVisible == nullptr)
            return -12;
        way->setIsVisible(strcmp(isVisible, "true") == 0);

        const char* version = xmlWay->Attribute("version");
        if(version == nullptr)
            return -13;
        way->setVersion(version);

        const char* changeset = xmlWay->Attribute("changeset");
        if(changeset == nullptr)
            return -14;
        way->setChangeset(changeset);

        const char* timestamp = xmlWay->Attribute("timestamp");
        if(timestamp == nullptr)
            return -15;
        way->setTimestamp(timestamp);

        tinyxml2::XMLElement* xmlConnected = xmlWay->FirstChildElement("nd");
        while(xmlConnected != nullptr){
            const char* ref = xmlConnected->Attribute("ref");
            if(ref == nullptr)
                return -16;

            connected.push_back(nodes.at(ref));
            xmlConnected = xmlConnected->NextSiblingElement("nd");
        }

        way->setNodes(connected);
        tinyxml2::XMLElement* xmlTag = xmlWay->FirstChildElement("tag");
        while(xmlTag != nullptr){
            const char* key = xmlTag->Attribute("k");
            const char* value = xmlTag->Attribute("v");
            if(key == nullptr || value == nullptr)
                return -17;

            tags.push_back(std::make_pair(key, value));
            xmlTag = xmlTag->NextSiblingElement("tag");
        }

        way->setTags(tags);
        xmlWay = xmlWay->NextSiblingElement("way");

        ways.insert(std::make_pair(way->getId(), way));
    }


    tinyxml2::XMLElement* xmlRelation = osmRoot->FirstChildElement("relation");

    while(xmlRelation != nullptr){

        std::shared_ptr<OSMRelation> relation = std::make_shared<OSMRelation>();
        std::vector<std::pair<std::string, std::string> > tags;
        const char* id = xmlRelation->Attribute("id");
        if(id == nullptr)
            return -18;
        relation->setId(id);

        const char* userName = xmlRelation->Attribute("user");
        if(userName == nullptr)
            return -19;
        relation->setUserName(userName);
        const char* userId = xmlRelation->Attribute("uid");
        if(userId == nullptr)
            return -20;
        relation->setUserId(userId);

        const char* isVisible = xmlRelation->Attribute("visible");
        if(isVisible == nullptr)
            return -21;
        relation->setIsVisible(strcmp(isVisible, "true") == 0);

        const char* version = xmlRelation->Attribute("version");
        if(version == nullptr)
            return -22;
        relation->setVersion(version);

        const char* changeset = xmlRelation->Attribute("changeset");
        if(changeset == nullptr)
            return -23;
        relation->setChangeset(changeset);

        const char* timestamp = xmlRelation->Attribute("timestamp");
        if(timestamp == nullptr)
            return -24;
        relation->setTimestamp(timestamp);

        tinyxml2::XMLElement* xmlConnected = xmlRelation->FirstChildElement("member");
        while(xmlConnected != nullptr){
            const char* type = xmlConnected->Attribute("type");
            const char* member_ref = xmlConnected->Attribute("ref");
            const char* role_pointer = xmlConnected->Attribute("role");
            if(type == nullptr || member_ref == nullptr || role_pointer == nullptr)
                return -25;
            std::string role(role_pointer);

            //relation->setType(type);
            if(strcmp(type, "node") == 0)
                if(nodes.find(member_ref) != nodes.end())
                    relation->addNode(std::make_pair(nodes.at(member_ref), role));
                else
                    relation->addNode(std::make_pair(nullptr, role));
            else if(strcmp(type, "way") == 0)
                if(ways.find(member_ref) != ways.end())
                    relation->addWay(std::make_pair(ways.at(member_ref), role));
                else
                    relation->addWay(std::make_pair(nullptr, role));
            else if(strcmp(type, "relation") == 0)
                if(relations.find(member_ref) != relations.end())
                {
                    relation->addRelation(std::make_pair(relations.at(member_ref), role));
                } else
                {
                    relation->addRelation(std::make_pair(nullptr, role));
                }
            else
                return -26;

            xmlConnected = xmlConnected->NextSiblingElement("member");
        }


        tinyxml2::XMLElement* xmlTag = xmlRelation->FirstChildElement("tag");
        while(xmlTag != nullptr){
            const char* key = xmlTag->Attribute("k");
            const char* value = xmlTag->Attribute("v");
            if(key == nullptr || value == nullptr)
                return -27;

            tags.push_back(std::make_pair(key, value));
            xmlTag = xmlTag->NextSiblingElement("tag");
        }

        relation->setTags(tags);
        xmlRelation = xmlRelation->NextSiblingElement("relation");

        relations.insert(std::make_pair(relation->getId(), relation));
    }

    return 0;
}

int CityGMLCore::readLiDAR(std::string filename)
{

        std::ifstream ifs;        
        ifs.open(filename, std::ios::in | std::ios::binary);

        if(ifs.is_open())
        {
            liblas::ReaderFactory f;
            std::string s;
            liblas::Reader reader = f.CreateWithStream(ifs);
            liblas::Header const& header = reader.GetHeader();

            std::cout << "Compressed: " << header.Compressed() << std::endl;
            std::cout << "Signature: " << header.GetFileSignature() << std::endl;
            std::cout << "Points count: " << header.GetPointRecordsCount() << std::endl;

            lidarDTMPoints.clear();
            lidarDSMPoints.clear();
            while (reader.ReadNextPoint())
            {
                liblas::Point const& p = reader.GetPoint();
                std::shared_ptr<Point> p_ = std::make_shared<Point>(p.GetX(), p.GetY(), p.GetZ());
                std::string s = p.GetClassification().GetClassName();

                if(!isPointOutsideBounds(p_))
                {
                    if(s.compare("Ground") == 0)
                        lidarDTMPoints.push_back(p_);
                    else if (p.GetReturnNumber() == p.GetNumberOfReturns())
                        lidarDSMPoints.push_back(p_);
                }
            }

            ifs.close();
            return 0;
        }
        return -1;

}


void CityGMLCore::fixWay(std::shared_ptr<OSMWay> way)
{
    auto nodes = way->getNodes();
    for (uint i = 1; i < nodes.size(); i++) {

        while(i < nodes.size() - 1 &&
              *nodes.at(i)->getCoordinates() == *nodes.at(i - 1)->getCoordinates())
        {
            std::shared_ptr<OSMNode> tmp = nodes.at(i);
            way->removeNode(nodes.at(i)->getId());
            tmp.reset();
        }

        std::shared_ptr<OSMNode> tmp = nodes.at(i);
        auto comparator = [tmp](std::shared_ptr<OSMNode> n) { return n != nullptr && (*(n->getCoordinates())) == (*(tmp->getCoordinates())); };

        std::shared_ptr<OSMNode> n = nullptr;

        for(auto nit = nodes.begin() + i + 1; nit != nodes.end(); nit++)
        {
            if(*((*nit)->getCoordinates()) == *(tmp->getCoordinates()))
            {
                n = (*nit);
                break;
            }
        }
        std::vector<std::shared_ptr<OSMNode> >::iterator it = std::find_if(nodes.begin() + i + 1, nodes.end(), comparator);
        while(n != nullptr && (n)->getId().compare(tmp->getId()) != 0)
        {
            n = nullptr;
            way->removeNode(tmp->getId());
            tmp.reset();
            i--;
            it = std::find_if(nodes.begin() + i + 1, nodes.end(), comparator);
            for(auto nit = nodes.begin() + i + 1; nit != nodes.end(); nit++)
            {
                if(*((*nit)->getCoordinates()) == *(tmp->getCoordinates()))
                {
                    n = (*nit);
                    break;
                }
            }
            tmp = nodes.at(i);
        }

    }
}

void CityGMLCore::removeOSMWayNodesFromBuildings(std::vector<std::string> &ways, std::vector<std::vector<PointList > > polygons)
{
    pointVec points_vector;
    std::map<uint, uint > pointPolygonLink;
    uint id = 0, i = 0;
    double meanDiagonal = 0;
    for(auto buildingPolygons : polygons)
    {
        for(auto p : buildingPolygons.at(0))
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
        std::vector<std::shared_ptr<OSMNode> > nodes = arcs.at(*lit)->getNodes();
        for(auto nit = nodes.begin(); nit != nodes.end(); nit++){
            PointList intersections;
            std::shared_ptr<Point> v = (*nit)->getCoordinates();
            std::vector<std::pair<size_t, double> > neighbors_distances;
            std::vector<uint> neighboringPolygons;
            point_t query_point = {v->getX(), v->getY(), 0};
            neighbors_distances.clear();
            std::vector<size_t> neighbors = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            for(std::vector<size_t >::iterator it = neighbors.begin(); it != neighbors.end(); it++){
                std::vector<uint>::iterator pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(*it));
            }

            for(auto npit = neighboringPolygons.begin(); npit != neighboringPolygons.end(); npit++)
            {
                bool insideOuter = Utilities::isPointInsidePolygon(v, polygons.at(*npit).at(0));
                bool outsideInners = true;

                for(auto pit = polygons.at((*npit)).begin(); pit != polygons.at((*npit)).end(); pit++)
                    outsideInners = outsideInners && !Utilities::isPointInsidePolygon(v, *pit);

                if(insideOuter && outsideInners)
                {
                    bool onBoundary = false;
                    for(auto pit = polygons.at((*npit)).begin(); pit != polygons.at((*npit)).end(); pit++)
                    {
                        for(uint m = 1; m < pit->size(); m++)
                        {
                            std::shared_ptr<Point> p1 = pit->at(m - 1);
                            std::shared_ptr<Point> p2 = pit->at(m);
                            if(Utilities::isPointInSegment(p1.get(), p2.get(), v.get()))
                            {
                                onBoundary = true;
                                break;
                            }
                        }
                    }

                    if(!onBoundary)
                    {
                        arcs.at(*lit)->removeNode((*nit)->getId());
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
    for(auto building : buildings)
    {
        auto boundaries = building->getBoundaries();
        for(auto p : boundaries.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              0};
            points_vector.push_back(point);
            pointBuildingLink.insert(std::make_pair(id++, i));
        }
        auto bb = Utilities::bbExtraction(vertexToPointList(building->getBoundaries().at(0)));
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
                auto building_boundaries = buildings.at(neighboringPolygons.at(k))->getBoundaries();
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
        auto boundaries = building->getBoundaries();
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
                auto polygons = building->getBoundaries();

                std::vector<std::pair<uint, uint> > modifiedBoundaries;
                for(uint l = 0; l < polygons.size(); l++)
                {
                    for(uint m = 0; m < polygons.at(l).size(); m++)
                    {
                        std::shared_ptr<Vertex> pout = std::make_shared<Vertex>();


                        if(Utilities::segmentsIntersection(lines.at(i).at(j - 1), lines.at(i).at(j), polygons.at(l).at(m), polygons.at(l).at((m + 1) % polygons.at(l).size()), *pout))
                        {
                            auto it = std::find_if(intersections.begin(), intersections.end(), [pout](std::shared_ptr<Point> p) { return *p == *pout; });
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
                    buildings.at(neighboringPolygons.at(k))->setBoundaries(polygons); //Boh, ottimizzare?
                }
            }
            if(intersections.size() > 1)
            {

                std::sort(intersections.begin(), intersections.end(),
                          [lines, i, j](std::shared_ptr<Point> p1, std::shared_ptr<Point> p2)
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
    if(*p1[0] == *p1.back()) p1.erase(p1.begin() + p1.size() - 1); //pop_back?
    if(*p2[0] == *p2.back()) p2.erase(p2.begin() + p2.size() - 1);//pop_back?
    std::vector<std::shared_ptr<Point> > poly1(p1.begin(), p1.end());
    std::vector<std::shared_ptr<Point> > poly2(p2.begin(), p2.end());
    if(Utilities::isPolygonClockwise(poly1))
        std::reverse(poly1.begin(), poly1.end());
    if(!Utilities::isPolygonClockwise(poly2))
        std::reverse(poly2.begin(), poly2.end());
    for(uint i = 0; i < poly1.size(); i++)
        for(uint j = 0; j < poly2.size(); j++)
            if(*poly1[i] == *poly2[j] && *poly1[Utilities::mod(i + 1, poly1.size())] == *poly2[Utilities::mod(j + 1, poly2.size())])
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

void CityGMLCore::removeAlreadyExistingPoints(std::vector<std::vector<VertexList> > boundaries, std::vector<VertexList> arcs)
{
    pointVec points_vector;
    double epsilon = Point::EPSILON;
    for(uint i = 0; i < lidarDTMPoints.size(); i++)
    {
        auto p = lidarDTMPoints.at(i);
        point_t point = { p->getX(),
                          p->getY(),
                          0};
        points_vector.push_back(point);
    }
    KDTree tree(points_vector);

    int used = 9009;

    #pragma omp parallel for num_threads(31)
    for(uint i = 0; i < boundaries.size(); i++)
    {
        for(uint j = 0; j < boundaries.at(i).size(); j++)
        {
            for(uint k = 0; k < boundaries.at(i).at(j).size(); k++)
            {
                auto p = boundaries.at(i).at(j).at(k);
                point_t point = {p->getX(), p->getY(), 0.0};
                auto neighbors = tree.neighborhood_indices(point, epsilon);
                for(uint i = 0; i < neighbors.size(); i++)
                {
                    #pragma omp critical
                    {
                        lidarDTMPoints[neighbors[i]]->setInfo(static_cast<void*>(&used));
                    }
                }
            }
        }
    }

    for(uint i = 0; i < lidarDTMPoints.size(); i++)
    {
        if(lidarDTMPoints[i]->getInfo() != nullptr && *static_cast<int*>(lidarDTMPoints[i]->getInfo()) == used)
            lidarDTMPoints.erase(lidarDTMPoints.begin() + i);
    }

}

VertexList CityGMLCore::createDTMVertices(double scale_factor, Point origin)
{

    VertexList dtmVertices;
    pointVec points_vector;
    std::map<uint, uint > pointBuildingLink;
    uint id = 0, i = 0;
    double meanDiagonal = 0;
    double* geoTransform = dtm->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;

    for(auto building : buildings)
    {
        auto boundaries = building->getBoundaries();
        for(auto p : boundaries.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              p->getZ()};
            points_vector.push_back(point);
            pointBuildingLink.insert(std::make_pair(id++, i));
        }
        auto bb = Utilities::bbExtraction(vertexToPointList(building->getBoundaries().at(0)));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    meanDiagonal /= static_cast<double>(buildings.size());

    KDTree tree(points_vector);

    for(uint i = 0; i < dtm->GetDimensions()[0]; i++)
    {
        for(uint j = 0; j < dtm->GetDimensions()[1]; j++)
        {
            std::shared_ptr<Vertex> v = std::make_shared<Vertex>();
            v->setX((geoTransform[0] + i * geoTransform[1]));
            v->setY((geoTransform[3] + j * geoTransform[5]));
            v->setZ(0);
            bool inside = false;
            std::vector<std::pair<size_t, double> > neighbors_distances;
            std::vector<uint> neighboringPolygons;
            point_t query_point = {v->getX(), v->getY(), v->getZ()};
            neighbors_distances.clear();
            auto neighbors = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            for(auto it = neighbors.begin(); it != neighbors.end(); it++){
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointBuildingLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointBuildingLink.at(*it));
            }

            for(auto npit = neighboringPolygons.begin(); npit != neighboringPolygons.end(); npit++)
            {
                auto building_boundaries = buildings.at(*npit)->getBoundaries();
                bool insideOuter = Utilities::isPointInsidePolygon(v, vertexToPointList(building_boundaries.at(0)));
                bool outsideInners = true;

                for(auto pit = building_boundaries.begin(); pit != building_boundaries.end(); pit++)
                    outsideInners = outsideInners && !Utilities::isPointInsidePolygon(v, vertexToPointList(*pit));

                if(insideOuter && outsideInners)
                {
                    inside = true;
                    break;
                }
            }
            if(!inside)
            {
                dtmVertices.push_back(v);
            }
        }
    }

    return dtmVertices;
}

void CityGMLCore::removeOSMWayNodesFromBuildings(std::vector<std::string> &ways)
{
    pointVec points_vector;
    std::map<uint, uint > pointBuildingLink;
    uint id = 0, i = 0;
    double meanDiagonal = 0;
    for(auto building : buildings)
    {
        auto boundaries = building->getBoundaries();
        for(auto p : boundaries.at(0))
        {
            point_t point = { p->getX(),
                              p->getY(),
                              0};
            points_vector.push_back(point);
            pointBuildingLink.insert(std::make_pair(id++, i));
        }
        auto bb = Utilities::bbExtraction(vertexToPointList(building->getBoundaries().at(0)));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
        i++;
    }

    meanDiagonal /= static_cast<double>(buildings.size());

    KDTree tree(points_vector);
    uint counter = 0;


    #pragma omp parallel for num_threads(31)
    for(auto lit = ways.begin(); lit != ways.end(); lit++)
    {
        auto nodes = arcs.at(*lit)->getNodes();
        for(auto nit = nodes.begin(); nit != nodes.end(); nit++){
            PointList intersections;
            auto v = (*nit)->getCoordinates();
            double vz = v->getZ();
            v->setZ(0);
            std::vector<std::pair<size_t, double> > neighbors_distances;
            std::vector<uint> neighboringPolygons;
            point_t query_point = {v->getX(), v->getY(), 0.0};
            neighbors_distances.clear();
            auto neighbors = tree.neighborhood_indices(query_point, meanDiagonal / 2);

            for(auto it = neighbors.begin(); it != neighbors.end(); it++){
                auto pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointBuildingLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointBuildingLink.at(*it));
            }

            for(auto npit = neighboringPolygons.begin(); npit != neighboringPolygons.end(); npit++)
            {
                auto building_boundaries = buildings.at(*npit)->getBoundaries();
                bool insideOuter = Utilities::isPointInsidePolygon(v, vertexToPointList(building_boundaries.at(0)));
                bool outsideInners = true;

                for(auto pit = building_boundaries.begin(); pit != building_boundaries.end(); pit++)
                    outsideInners = outsideInners && !Utilities::isPointInsidePolygon(v, vertexToPointList(*pit));

                if(insideOuter && outsideInners)
                {
                    bool onBoundary = false;
                    for(auto pit = building_boundaries.begin(); pit != building_boundaries.end(); pit++)
                    {
                        for(uint m = 1; m < pit->size(); m++)
                        {
                            std::shared_ptr<Point> p1 = pit->at(m - 1);
                            std::shared_ptr<Point> p2 = pit->at(m);
                            double p1z = p1->getZ();
                            p1->setZ(0);
                            double p2z = p2->getZ();
                            p2->setZ(0);
                            if(Utilities::isPointInSegment(p1.get(), p2.get(), v.get()))
                            {
                                p1->setZ(p1z);
                                p2->setZ(p2z);
                                onBoundary = true;
                                break;
                            }
                            p1->setZ(p1z);
                            p2->setZ(p2z);
                        }
                    }

                    if(!onBoundary)
                    {
                        arcs.at(*lit)->removeNode((*nit)->getId());
                        nodes.erase(nit);
                        v.reset();
                        break;
                    }
                }
            }


            v->setZ(vz);
        }

        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / ways.size() << "%\r" << std::flush;
        }
    }
    std::cout << std::endl;
}

std::vector<std::vector<std::pair<unsigned int, unsigned int> > > CityGMLCore::extractBuildingsHeights(bool is_dtm_tiff = true, double scale_factor = 1.0, Point origin = Point(0,0,0))
{
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > heights;
    unsigned int counter = 0;
    std::map<unsigned int, std::pair<unsigned int, unsigned int> > index_to_rowcol;
    for(auto building : buildings)
    {
        std::vector<std::pair<unsigned int, unsigned int> > building_heights;
        heights.push_back(building_heights);
    }

    auto dhm_tiff = is_dtm_tiff ? dtm : dsm;
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    std::cout << dhm_tiff->GetDimensions()[0] << " " << dhm_tiff->GetDimensions()[1] << std::endl;
    std::cout << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << std::endl;
    std::cout << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << std::endl;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;


//    counter = 0;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1] + i * geoTransform[2],
                                          geoTransform[3] + j * geoTransform[4] + i * geoTransform[5], //NEGATIVO???
                                          0};
            points_vector.push_back(point);
            index_to_rowcol.insert(std::make_pair(counter++, std::make_pair(i,j)));
        }
    }
    KDTree tree(points_vector);

    counter = 0;

    #pragma omp parallel for num_threads(31)
    for(uint i = 0; i < buildings.size(); i++)
    {
        auto boundaries = buildings.at(i)->getBoundaries();
        std::vector<VertexList> boundaries2D;
        for(auto boundary : boundaries)
        {
            VertexList boundary2D;
            for(auto v : boundary)
            {
                boundary2D.push_back(std::make_shared<Vertex>(v->getX(), v->getY(), 0.0));
            }
            boundaries2D.push_back(boundary2D);
        }
        std::vector<std::shared_ptr<Point> > bb = Utilities::bbExtraction(boundaries2D[0]);
        double sphere_radius = std::max(pixel_diagonal_length, ((*bb[0]) - (*bb[2])).norm());
        Point middle(0,0,0);
        std::for_each(boundaries2D[0].begin(), boundaries2D[0].end(), [&middle](std::shared_ptr<Vertex> v){
            middle += *v;
        });
        middle /= boundaries2D[0].size();

        std::vector<size_t> neighbors_indices;
        std::vector<std::pair<unsigned int, unsigned int> > neighbors;
        point_t query_point = {middle.getX(), middle.getY(), 0.0};
        neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);

        for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
            neighbors.push_back(index_to_rowcol.at(*it));
        }
        uint m = 0;

        for(unsigned int j = 0; j < neighbors.size(); j++)
        {
            double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
            double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
            std::vector<std::shared_ptr<Point> > frame;
            frame.push_back(std::make_shared<Point>(min_x, min_y, 0));
            frame.push_back(std::make_shared<Point>(min_x + geoTransform[1], min_y, 0));
            frame.push_back(std::make_shared<Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
            frame.push_back(std::make_shared<Point>(min_x, frame.back()->getY(), 0.0));
            frame.push_back(frame[0]);

            bool frameInBuilding = false, buildingInFrame = false;
            //Check frame interno (o intersecante) all'edificio
            for(uint k = 0; k < 4; k++)
                if(Utilities::isPointInsidePolygon(frame[k], boundaries2D[0]))
                {
                    frameInBuilding = true;
                    for(uint l = 1; l < boundaries2D.size(); l++)
                        if(Utilities::isPointInsidePolygon(frame[0], boundaries2D[l]))
                        {
                            frameInBuilding = false;
                            break;
                        }
                    if(frameInBuilding)
                        break;
                }

            if(!frameInBuilding)
            {
                //Check bounding box interna al frame
                for(uint k = 0; k < 4; k++)
                {
                    if(Utilities::isPointInsidePolygon(bb[k], frame))
                        for(uint l = 0; l < boundaries2D[0].size(); l++)
                        {
                            buildingInFrame = Utilities::isPointInsidePolygon(boundaries2D[0][l], frame);
                            if(buildingInFrame)
                                break;
                        }

                    if(buildingInFrame)
                        break;
                }
            }

            std::vector<std::shared_ptr<Point> > boundaryPoints(boundaries2D[0].begin(), boundaries2D[0].end());
            if(frameInBuilding || buildingInFrame || Utilities::polygonsIntersect(frame, boundaryPoints))
                heights[i].push_back(std::make_pair(neighbors.at(j).first, neighbors.at(j).second));

            for(unsigned int l = 0; l < 4; l++)
                frame.at(l).reset();
        }

        if(heights[i].size() == 0)
        {
            #pragma omp critical
            {
                std::cerr << "Impossible case: boundary does not include any pixel while no pixel wholly include the boundary and boundary intersects no pixel" << std::endl;
                std::cerr << i << std::endl;
                std::cerr << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << " " << std::endl;
                std::cerr << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << " " << std::endl << std::flush;
                std::cerr << dhm_tiff->GetDimensions()[0] << " " << dhm_tiff->GetDimensions()[1] << std::endl;

                for(unsigned int j = 0; j < neighbors.size(); j++)
                {
                    double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
                    double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
                    std::vector<std::shared_ptr<Point> > frame;
                    frame.push_back(std::make_shared<Point>(min_x, min_y, 0));
                    frame.push_back(std::make_shared<Point>(min_x + geoTransform[1], min_y, 0));
                    frame.push_back(std::make_shared<Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
                    frame.push_back(std::make_shared<Point>(min_x, frame.back()->getY(), 0.0));
                    frame.push_back(frame[0]);
                }
                exit(6);
            }
        }

        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / buildings.size() << "%\r" << std::flush;
        }
    }
    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];

    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
    return heights;
}

std::vector<std::vector<double> > CityGMLCore::extractBuildingsHeightsFromLidar(double scale_factor = 1.0, Point origin = Point(0,0,0))
{
    std::vector<std::vector<double> > heights;
    unsigned int counter = 0;
    for(auto building : buildings)
    {
        std::vector<double > building_heights;
        heights.push_back(building_heights);
    }
    pointVec points_vector;
    for(auto p : lidarDSMPoints)
    {
        std::vector<double> point = { p->getX(), p->getY(), 0.0};
        points_vector.push_back(point);
    }
    KDTree tree(points_vector);

    counter = 0;

    std::cout << "0%" << std::endl;
    #pragma omp parallel for num_threads(31)
    for(uint i = 0; i < buildings.size(); i++)
    {
        auto boundaries = buildings.at(i)->getBoundaries();
        std::vector<VertexList> boundaries2D;
        for(auto boundary : boundaries)
        {
            VertexList boundary2D;
            for(auto v : boundary)
            {
                boundary2D.push_back(std::make_shared<Vertex>(v->getX(), v->getY(), 0.0));
            }
            boundaries2D.push_back(boundary2D);
        }
        std::vector<std::shared_ptr<Point> > bb = Utilities::bbExtraction(boundaries2D[0]);
        double sphere_radius = ((*bb[0]) - (*bb[2])).norm();
        Point middle(0,0,0);
        std::for_each(boundaries2D[0].begin(), boundaries2D[0].end(), [&middle](std::shared_ptr<Vertex> v){
            middle += *v;
        });
        middle /= boundaries2D[0].size();

        std::vector<size_t> neighbors_indices;
        std::vector<std::pair<unsigned int, unsigned int> > neighbors;
        point_t query_point = {middle.getX(), middle.getY(), 0.0};
        neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);

        for(unsigned int j = 0; j < neighbors_indices.size(); j++)
        {
            auto p = lidarDSMPoints.at(neighbors_indices.at(j));
            bool insideOuter = false, outsideInner = true;
            if(Utilities::isPointInsidePolygon(p, boundaries2D[0]))
            {
                insideOuter = true;
                for(uint l = 1; l < boundaries2D.size(); l++)
                    if(Utilities::isPointInsidePolygon(p, boundaries2D[l]))
                    {
                        outsideInner = false;
                        break;
                    }
            }
            if(insideOuter && outsideInner)
                heights[i].push_back(lidarDSMPoints.at(neighbors_indices[j])->getZ());
        }

        if(heights[i].size() == 0)
            heights[i].push_back(buildings[i]->getBoundaries()[0][0]->getZ());


        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / buildings.size() << "%\r" << std::flush;
        }
    }
    return heights;

}

void CityGMLCore::associateElevations(std::vector<std::vector<VertexList> > triangulationHoles, std::vector<VertexList> streetsArcsPoints)
{
    unsigned int counter = 0;
    pointVec points_vector;
    for(unsigned int i = 0; i < lidarDTMPoints.size(); i++)
    {
        std::vector<double> point = { lidarDTMPoints[i]->getX(), lidarDTMPoints[i]->getY(), 0.0};
        points_vector.push_back(point);
    }

    KDTree tree(points_vector);

    #pragma omp parallel for num_threads(31)
    for(uint i = 0; i < triangulationHoles.size(); i++)
    {
        auto boundaries = triangulationHoles[i];
        for(uint j = 0; j < boundaries.size() - 1; j++)
            for(uint k = 0; k < boundaries[j].size(); k++)
            {
                std::shared_ptr<Point> p = boundaries[j][k];
                point_t point = {p->getX(), p->getY(), 0.0};
                size_t ret_index = tree.nearest_index(point);
                p->setZ(lidarDTMPoints[ret_index]->getZ());
            }

        #pragma omp critical
        {
            std::cout << counter++ * 100 / triangulationHoles.size() << "%\r" << std::flush;
        }
    }

    #pragma omp parallel for num_threads(31)
    for(uint i = 0; i < streetsArcsPoints.size(); i++)
    {
        auto polylines = streetsArcsPoints[i];
        for(uint j = 0; j < polylines.size(); j++)
        {
            std::shared_ptr<Point> p = polylines[j];
            point_t point = {p->getX(), p->getY(), 0.0};
            size_t ret_index = tree.nearest_index(point);
            p->setZ(lidarDTMPoints[ret_index]->getZ());
        }

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

std::string CityGMLCore::getDtmFilename() const
{
    return dtmFilename;
}

void CityGMLCore::setDtmFilename(const std::string &value)
{
    dtmFilename = value;
}

std::string CityGMLCore::getDsmFilename() const
{
    return dsmFilename;
}

void CityGMLCore::setDsmFilename(const std::string &value)
{
    dsmFilename = value;
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

bool isPointOutsideBounds(const Point p, const Point min, const Point max)
{
    return p.getX() < min.getX() || p.getY() < min.getY() ||
           p.getX() > max.getX() || p.getY() > max.getY();
}

bool CityGMLCore::isPointOutsideBounds(const std::shared_ptr<Point> p)
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

int CityGMLCore::buildLevel(uint level)
{

    switch (level) {
        case 0:
            return buildLevel0();
        case 1:
            return buildLevel1();
    }
    return -1;
}


int CityGMLCore::buildLevel0()
{
    if(osmIsLoaded && dtmIsLoaded)
    {

        if(meshes[0] != nullptr)
            meshes[0].reset();

        meshes[0] = std::make_shared<TriangleMesh>();

        /* Following GDAL definition:
         *  GT(0) x-coordinate of the upper-left corner of the upper-left pixel.
         *  GT(1) w-e pixel resolution / pixel width.
         *  GT(2) row rotation (typically zero).
         *  GT(3) y-coordinate of the upper-left corner of the upper-left pixel.
         *  GT(4) column rotation (typically zero).
         *  GT(5) n-s pixel resolution / pixel height (negative value for a north-up image).
         */
        double* geoTransform = dtm->GetGeoTransform();
        Point min(std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  0);
        Point max(-std::numeric_limits<double>::max(),
                  -std::numeric_limits<double>::max(),
                  0);

        for(uint i = 0; i < lidarDTMPoints.size(); i++)
        {
            if(lidarDTMPoints.at(i)->getX() < min.getX())
                min.setX(lidarDTMPoints.at(i)->getX());
            if(lidarDTMPoints.at(i)->getY() < min.getY())
                min.setY(lidarDTMPoints.at(i)->getY());
            if(lidarDTMPoints.at(i)->getX() > max.getX())
                max.setX(lidarDTMPoints.at(i)->getX());
            if(lidarDTMPoints.at(i)->getY() > max.getY())
                max.setY(lidarDTMPoints.at(i)->getY());
        }

        std::vector<PointList > boundaries;

        std::cout << "Cleaning polygons/lines" << std::endl;

        for(auto ait = arcs.begin(); ait != arcs.end(); ait++)
        {
            bool erased = false;
            //Keep only arcs with all the nodes inside the bounds defined by the DTM
            for(uint j = 0; j < ait->second->getNodes().size(); j++)
            {
                if(isPointOutsideBounds(ait->second->getNodes().at(j)->getCoordinates()))
                {
                    for(uint j = 0; j < ait->second->getNodes().size(); j++)
                    {
                        ait->second->getNodes().at(j)->getCoordinates().reset();
                        ait->second->getNodes().at(j).reset();
                    }
                    ait = arcs.erase(ait);
                    ait--;
                    erased = true;
                    break;
                }
            }
            if(erased)
                continue;

            //Separate streets' arcs from buildings' arcs

            if(ait->second->checkTag(std::make_pair("building","")))
                buildingsArcs.push_back(ait->first);

            if(ait->second->checkTag(std::make_pair("highway", "")) && (!ait->second->checkTag(std::make_pair("area", "")) || ait->second->checkTag(std::make_pair("area", "no"))))
            {
                fixWay(ait->second);
                if(ait->second->getNodes().size() > 1)
                    streetsArcs.push_back(ait->first);
            }

        }

        std::set<std::string> usedWays;

        //TO DO: IMPLEMENTARE PULIZIA BOUNDARY QUI

        for(auto rit = relations.begin(); rit != relations.end(); rit++)
        {
            auto tags = rit->second->getTags();

            auto checkMultipolygon = [](std::pair<std::string, std::string> p)
                { return ((p.first.compare("type") == 0) && p.second.compare("multipolygon") == 0); };

            auto checkBuilding = [](std::pair<std::string, std::string> p)
                { return (p.first.compare("building")) == 0; };

            //We only take into consideration relations defining buildings as multipolygons
            if(std::find_if(tags.begin(), tags.end(), checkMultipolygon) != tags.end() &&
               std::find_if(tags.begin(), tags.end(), checkBuilding) != tags.end() )
            {
                auto buildingBoundaries = rit->second->getWays();
                auto building = std::make_shared<Building>();
                std::vector<VertexList> buildingBoundariesPoints;
                bool hasOuter = false;
                bool invalidRelation = false;
                for(uint i = 0; i < buildingBoundaries.size(); i++)
                {
                    if(buildingBoundaries.at(i).first == nullptr)
                        continue;
                    //Checking if we already used the way: may be unnecessary, but better be sure
                    auto it = std::find(usedWays.begin(), usedWays.end(), buildingBoundaries.at(i).first->getId());
                    if(it == usedWays.end())
                    {
                        usedWays.insert(buildingBoundaries.at(i).first->getId());

                        auto boundaryNodes = buildingBoundaries.at(i).first->getNodes();
                        VertexList boundaryPoints;

                        //The isPolygonClockwise function requires the polygon to be open (the last segment is implicit)
                        if(boundaryNodes.at(0)->getId() == boundaryNodes.back()->getId())
                            boundaryNodes.pop_back();

                        for(uint j = 0; j < boundaryNodes.size(); j++)
                        {
                            auto newVertex = std::dynamic_pointer_cast<Vertex>(boundaryNodes.at(j)->getCoordinates());
                            if(newVertex == nullptr)
                            {
                                newVertex = std::make_shared<Vertex>(boundaryNodes.at(j)->getCoordinates());
                                boundaryNodes.at(j)->setCoordinates(newVertex);
                            }
                            boundaryPoints.push_back(newVertex);
                        }

                        bool isClockwise = Utilities::isPolygonClockwise(vertexToPointList(boundaryPoints));

                        //Re-closing the polygon
                        boundaryPoints.push_back(boundaryPoints.at(0));

                        //Next part is performed only if we are in the outmost boundary of the multipolygon
                        if(buildingBoundaries.at(i).second.compare("outer") == 0)
                        {
                            if(!hasOuter)
                            {
                                hasOuter = true;
                                //Exterior boundaries are in anti-clockwise order by convention.
                                if(isClockwise)
                                    std::reverse(boundaryPoints.begin(), boundaryPoints.end());
                                //The first boundary of the list is the outmost one.
                                buildingBoundariesPoints.insert(buildingBoundariesPoints.begin(), boundaryPoints);
                                building->addOuterBoundary(boundaryPoints);
                            } else
                            {
                                //TO DO: il caso di boundary esterni multipli è da discutere e gestire di conseguenza. Al momento si eliminano i surplus.
                                buildingsArcs.push_back(buildingBoundaries.at(i).first->getId());
                                rit->second->removeWay(buildingBoundaries.at(i).first);
                                usedWays.erase(buildingBoundaries.at(i).first->getId());
                            }
                        }else
                        {
                            //Interior boundaries are in clockwise order by convention.
                            if(!isClockwise)
                                std::reverse(boundaryPoints.begin(), boundaryPoints.end());
                            buildingBoundariesPoints.push_back(boundaryPoints);
                            building->addInnerBoundary(boundaryPoints);
                        }
                    } else
                    {
                        invalidRelation = true;
                        break;
                    }

                }

                //Ways need to be used only once
                if(!invalidRelation)
                {
                    building->setId(std::to_string(buildings.size()));
                    buildings.push_back(building);
                }
                else
                {
                    rit = relations.erase(rit);
                    rit--;
                }
            }
        }


        //Remaining buildings are defined by simple OSMWays
        for(auto ait = buildingsArcs.begin(); ait != buildingsArcs.end(); ait++)
        {
            //Ways need to be used only once
            if(std::find(usedWays.begin(), usedWays.end(), *ait) == usedWays.end())
            {
                std::vector<VertexList > buildingBoundariesPoints;
                VertexList boundaryPoints;
                std::vector<std::shared_ptr<OSMNode> > nodes = arcs.at(*ait)->getNodes();
                //The isPolygonClockwise function requires the polygon to be open (the last segment is implicit)
                if(nodes.at(0)->getId() == nodes.back()->getId())
                    nodes.pop_back();

                for(uint j = 0; j < nodes.size(); j++)
                {
                    auto newVertex = std::dynamic_pointer_cast<Vertex>(nodes.at(j)->getCoordinates());
                    if(newVertex == nullptr)
                    {
                        newVertex = std::make_shared<Vertex>(nodes.at(j)->getCoordinates());
                        nodes.at(j)->setCoordinates(newVertex);
                    }

                    boundaryPoints.push_back(newVertex);
                }

                bool isClockwise = Utilities::isPolygonClockwise(vertexToPointList(boundaryPoints));
                //Re-closing the polygon
                boundaryPoints.push_back(boundaryPoints.at(0));

                //Exterior boundaries are in anti-clockwise order by convention.
                if(isClockwise)
                    std::reverse(boundaryPoints.begin(), boundaryPoints.end());

                buildings.push_back(std::make_shared<Building>());
                buildings.back()->addOuterBoundary(boundaryPoints);
                buildings.back()->setId(std::to_string(buildings.size()));
            }

        }

        //Removing buildings with points outside the bounds
        for(auto iit = buildings.begin(); iit != buildings.end(); iit++)
        {
            std::vector<VertexList> building_boundaries = (*iit)->getBoundaries();
            for(auto jit = building_boundaries.begin(); jit != building_boundaries.end(); jit++)
            {
                bool erased = false;
                for(auto kit = jit->begin(); kit != jit->end(); kit++)
                {
                    if(isPointOutsideBounds(*kit))
                    {
                        for(auto kit = jit->begin(); kit != jit->end(); kit++)
                            kit->reset();
                        jit->clear();
                        iit = buildings.erase(iit);
                        iit--;
                        erased = true;
                        break;
                    }
                }
                if(erased)
                    break;
            }
        }

        for(auto it = buildings.begin(); it != buildings.end(); it++)
        {
            auto building1 = *it;
            auto containedIt = std::find_if(buildings.begin(), buildings.end(), [building1](std::shared_ptr<Building> building2)
                {
                    if(building1->getId().compare(building2->getId()) == 0)
                        return false;
                    bool contained = false;
                    auto boundaries1 = building1->getBoundaries();
                    auto boundaries2 = building2->getBoundaries();

                    std::vector<std::shared_ptr<Point> > boundary1(boundaries1[0].begin(), boundaries1[0].end());
                    std::vector<std::shared_ptr<Point> > boundary2(boundaries2[0].begin(), boundaries2[0].end());

                    if(Utilities::isPolygonInsidePolygon(boundary1, boundary2))
                    {
                        contained = true;
                        for(uint j = 1; j < boundaries2.size(); j++)
                        {

                            std::vector<std::shared_ptr<Point> > boundary3(boundaries2[j].begin(), boundaries2[j].end());

                            if(Utilities::isPolygonInsidePolygon(boundary1, boundary3))
                            {
                                contained = false;
                                break;
                            }
                        }
                        if(!contained)
                            return false;
                    }

                    return contained;
                });

            if(containedIt != buildings.end())
            {
                it = buildings.erase(it);
                it--;
            }
        }


        std::shared_ptr<Point> origin = std::make_shared<Point>(0,0,0);
        uint points_number = 0;

        for(auto building : buildings)
            for(auto polygon : building->getBoundaries())
                for(auto point : polygon)
                {
                    (*origin) += (*point);
                    points_number++;
                }

        for(uint j = 0; j < static_cast<uint>(dtm->GetDimensions()[0]); j++)
            for(uint k = 0; k < static_cast<uint>(dtm->GetDimensions()[1]); k++)
            {

                Point p(geoTransform[0] + (k + 0.5) * geoTransform[1] + j * geoTransform[2],
                          geoTransform[3] + k * geoTransform[4] + (j + 0.5) * geoTransform[5], 0);

                (*origin) += p;
                points_number++;
            }


        double city_size = (max - min).norm();
        (*origin) /= points_number;

        std::cout.precision(15);
        std::cout << "size:" << city_size << std::endl;
        origin->print(std::cout);
        for(auto node : nodes)
        {
            std::shared_ptr<Point> pos = node.second->getCoordinates();
            *pos -= *origin;
            *pos /= city_size;
        }

        for(auto p : lidarDTMPoints)
        {
            *p -= *origin;
            *p /= city_size;
        }

        for(auto p : lidarDSMPoints)
        {
            *p -= *origin;
            *p /= city_size;
        }
        //Fixing building boundaries
        for(auto bit = buildings.begin(); bit != buildings.end(); bit++)
        {
            (*bit)->fixBoundaries();
            if((*bit)->getBoundaries().size() == 0)
            {
                bit = buildings.erase(bit);
                bit--;
            }
        }



        std::cout << "Removing nodes into buildings"<< std::endl;
        removeOSMWayNodesFromBuildings(streetsArcs);
        std::vector<VertexList > arcsPoints, constraints;

        for(auto ait = streetsArcs.begin(); ait != streetsArcs.end(); ait++)
            if(arcs.at(*ait)->getNodes().size() < 2)
            {
                ait = streetsArcs.erase(ait);
                ait--;
            } else
            {
                VertexList arc_points;
                std::vector<std::shared_ptr<OSMNode> > arc_nodes = arcs.at(*ait)->getNodes();

                for(auto node : arc_nodes)
                {
                    auto newVertex = std::dynamic_pointer_cast<Vertex>(node->getCoordinates());
                    if(newVertex == nullptr)
                    {

                        newVertex = std::make_shared<Vertex>(node->getCoordinates());
                        node->setCoordinates(newVertex);
                    }
                    arc_points.push_back(newVertex);
                }

                arcsPoints.push_back(arc_points);
            }


        std::cout << "Refining lines:" << std::endl;
        Utilities::refineLines(arcsPoints, dtm.get(), city_size, *origin);
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
        auto keptClippings = removeSegmentsIntersectingBuildings(arcsPoints);  //VA SISTEMATO, DEVE USARE I GRUPPI DI EDIFICI
        std::cout << "Ended!" << std::endl;

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

        constraints.insert(constraints.end(), arcsPoints.begin(), arcsPoints.end());
        std::vector<std::vector<VertexList> > triangulationHoles = {external_bb};

        std::set<std::string> usedBuildings;
        std::vector<std::shared_ptr<BuildingsGroup> > groups;

        std::cout << "Creating buildings' groups" << std::endl;
        unsigned int reachedGroupId = 0;
        for(auto bit1 = buildings.begin(); bit1 != buildings.end(); bit1++)
        {
            auto it = usedBuildings.find((*bit1)->getId());
            if(it == usedBuildings.end())
                usedBuildings.insert((*bit1)->getId());
            auto group = std::make_shared<BuildingsGroup>();
            std::queue<std::shared_ptr<Building> > queue;
            queue.push((*bit1));
            while(!queue.empty())
            {
                auto building = queue.front();
                if(building->getId().compare((*bit1)->getId()) != 0)
                {
                    (*bit1)->addAdjacentBuilding(building);
                    building->addAdjacentBuilding(*bit1);
                }
                queue.pop();
                group->addBuilding(building);
                for(auto bit2 = buildings.begin(); bit2 != buildings.end(); bit2++)
                {
                    auto it = usedBuildings.find((*bit2)->getId());
                    if((*bit1)->getId().compare((*bit2)->getId()) != 0 && it == usedBuildings.end() &&
                        arePolygonsAdjacent(building->getBoundaries().at(0), (*bit2)->getBoundaries().at(0)))
                    {
                        usedBuildings.insert((*bit2)->getId());
                        queue.push(*bit2);
                    }
                }
            }
            if(group->getBuildings().size() > 1)
            {
                group->setId(std::to_string(reachedGroupId++));
                groups.push_back(group);
            }

        }
        std::cout << "Ended!" << std::endl;
        usedBuildings.clear();

        std::cout << "Extracting overall polygons" << std::endl;
        for(auto group : groups)
        {
            group->computeAdjacencyGraph();
            auto groupBoundaries = group->extractOverallBasePolygon();
            uint counter = 0;
            auto buildingsToBeTriangulated = group->getBuildings();
            std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

            for(auto building : group->getBuildings())
            {

                auto it = std::find(usedBuildings.begin(), usedBuildings.end(), building->getId());
                if(it != usedBuildings.end())
                {
                    std::cerr << "Impossible, all buildings that are part of a group must be in that group. There couldn't be more groups containing a building.";
                    exit(12);
                }

                usedBuildings.insert(building->getId());
            }
            triangulationHoles.push_back(groupBoundaries);
        }

        std::cout << "Ended!" << std::endl;


        //check if redundant with previous check
        for(auto it = buildings.begin(); it != buildings.end(); it++)
        {
            auto building = *it;
            auto usedIt = std::find_if(usedBuildings.begin(), usedBuildings.end(), [building](std::string s){ return building->getId().compare(s) == 0;});
            if(usedIt == usedBuildings.end())
            {
                auto containedIt = std::find_if(triangulationHoles.begin() + 1, triangulationHoles.end(), [building](std::vector<VertexList> list)
                    {
                        bool contained = false;
                        auto boundaries = building->getBoundaries();

                        std::vector<std::shared_ptr<Point> > boundary1(boundaries[0].begin(), boundaries[0].end());
                        std::vector<std::shared_ptr<Point> > boundary2(list[0].begin(), list[0].end());
                        if(Utilities::isPolygonInsidePolygon(boundary1, boundary2))
                        {
                            contained = true;
                            for(uint j = 1; j < list.size(); j++)
                            {

                                std::vector<std::shared_ptr<Point> > boundary3(list[j].begin(), list[j].end());

                                if(Utilities::isPolygonInsidePolygon(boundary1, boundary3))
                                {
                                    contained = false;
                                    break;
                                }
                            }
                            if(!contained)
                                return false;
                        }

                        return contained;
                    });
                if(containedIt == triangulationHoles.end())
                {
                    triangulationHoles.push_back(building->getBoundaries());
                } else
                {
                    it = buildings.erase(it);
                }
            }
        }



        removeAlreadyExistingPoints(triangulationHoles, arcsPoints);
        VertexList constraintVertices = createLiDARDTMVertices(/*triangulationHoles, arcsPoints/*city_size, *origin*/);

        meshes[0]->triangulate(triangulationHoles, arcsPoints, constraintVertices);
        for(uint i = 0; i < constraintVertices.size(); i++)
        {
            double z = meshes[0]->getVertex(i)->getZ();
            if(z > 5000)
                std::cout << std::endl;
        }
        meshes[0]->save("cacco1.ply", 15);


        for(uint i = 0; i < keptClippings.size(); i++)
        {
            uint numberOfNewLines = keptClippings.at(i).second - keptClippings.at(i).first;
            for(uint j = 0; j < numberOfNewLines; j++)
            {
                VertexList prevLine = arcsPoints.at(keptClippings.at(i).first);
                VertexList clipping = arcsPoints.at(keptClippings.at(i).first + j + 1);
                if(clipping.at(0)->getId().compare(clipping.at(1)->getId()) == 0)
                    clipping.erase(clipping.begin());
                if(prevLine.at(prevLine.size() - 2)->getId().compare(prevLine.at(prevLine.size() - 1)->getId()) == 0)
                    arcsPoints.at(keptClippings.at(i).first).pop_back();
                arcsPoints.at(keptClippings.at(i).first).insert(arcsPoints.at(keptClippings.at(i).first).end(), clipping.begin(), clipping.end());
            }
            arcsPoints.erase(arcsPoints.begin() + keptClippings.at(i).first + 1, arcsPoints.begin() + keptClippings.at(i).second + 1);
            for(uint j = i + 1; j < keptClippings.size(); j++)
            {
                keptClippings.at(j).first -= numberOfNewLines;
                keptClippings.at(j).second -= numberOfNewLines;
            }
        }

        std::cout << "Ended" << std::endl;

        uint buildingPos = 0;

        std::cout << "Starting buildings' triangulation" << std::endl <<  "0%";

        pointVec points_vector;
        for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        {
           auto v = meshes[0]->getVertex(i);
           auto p = {v->getX(), v->getY(), 0.0};
           points_vector.push_back(p);

        }
        std::shared_ptr<KDTree> meshKDTree = std::make_shared<KDTree>(points_vector);

        for(uint i = 1; i < triangulationHoles.size(); i++)
        {
            std::vector<uint> shell_triangles;
            std::vector<double*> holes;
            std::vector<double*> points;
            std::map<uint, std::shared_ptr<Vertex> > pointsToVertices;
            std::vector<std::vector<uint> > polylines;
            std::vector<double*> constraints;
            std::vector<std::pair<uint, uint> > constraint_segments;
            std::map<std::string, std::vector<std::shared_ptr<Edge> > > verticesEdges;
            uint reached_id = 0;
            std::vector<uint> flaggedMeshVertices;

            for(uint j = 0; j < triangulationHoles.at(i).size(); j++)
                for(uint k = 1; k < triangulationHoles.at(i).at(j).size(); k++)
                {
                    auto v1 = triangulationHoles.at(i).at(j).at(k - 1);
                    auto v2 = triangulationHoles.at(i).at(j).at(k);
                    auto p = ((*v1) + (*v2)) / 2;
                    auto point = {p.getX(), p.getY(), 0.0};
                    auto neighbourhood = meshKDTree->neighborhood(point, (*v2 - *v1).norm() / 2 + Point::EPSILON);
                    for(auto pi : neighbourhood)
                    {
                        meshes[0]->getVertex(pi.second)->addFlag(FlagType::INSIDE);
                        flaggedMeshVertices.push_back(pi.second);
                    }

                }

            std::vector<std::shared_ptr<Building> > buildingsToBeTriangulated;
            if(i < groups.size() + 1)
            {
                buildingsToBeTriangulated = groups.at(i - 1)->getBuildings();
                std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

                for(auto building : buildingsToBeTriangulated)
                {
                    auto boundaries = building->getBoundaries();
                    int pos = -1;
                    for(uint j = 1; j < boundaries[0].size(); j++)
                    {
                        auto it = std::find_if(triangulationHoles[i][0].begin(), triangulationHoles[i][0].end(), [boundaries, j](std::shared_ptr<Vertex> v){
                            return *v == *boundaries[0][j];
                        });

                        if(it != triangulationHoles[i][0].end())
                        {

                            int newPos = it - triangulationHoles[i][0].begin();
                            if (newPos == 0)
                                newPos = triangulationHoles[i][0].size() - 1;
                            if(pos != -1 && std::abs(newPos - pos) != 1)
                            {
                                pos = -1;
                                continue;
                            }
                            pos = newPos;
                           if(*boundaries[0][j - 1] == *triangulationHoles[i][0][pos - 1])
                            {
                                std::shared_ptr<Vertex> v1 = std::static_pointer_cast<Vertex>(boundaries[0][j - 1]);
                                std::shared_ptr<Vertex> v2 = std::static_pointer_cast<Vertex>(boundaries[0][j]);

                                if(v1 == nullptr || v2 == nullptr ||
                                   v1->getId().compare("") == 0 || v1->getE0() == nullptr ||
                                   v2->getId().compare("") == 0 || v2->getE0() == nullptr)
                                {
                                    uint counter = 0;
                                    std::cout << "Non va bene!" << std::endl;
                                    for(auto vertexList : triangulationHoles.at(i))
                                    {
                                        std::cout << "TH" << counter << "=[" << std::endl;
                                        for(auto v : vertexList)
                                        {
                                            std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                        }
                                        std::cout << "];" << std::endl;
                                        std::cout << "plot(TH" << counter << "(:,1), TH" << counter << "(:,2));" << std::endl;
                                        std::cout << "labels=num2cell(0:length(TH" << counter << ")-1);" << std::endl;
                                        std::cout << "text(TH" << counter << "(:,1), TH" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                    }
                                    counter = 0;
                                    for(auto boundary : boundaries)
                                    {
                                        std::cout << "B" << counter << "=[" << std::endl;
                                        for(auto v : boundary)
                                        {
                                            std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                        }
                                        std::cout << "];" << std::endl;
                                        std::cout << "plot(B" << counter << "(:,1), B" << counter << "(:,2));" << std::endl;
                                        std::cout << "labels=num2cell(0:length(B" << counter << ")-1);" << std::endl;
                                        std::cout << "text(B" << counter << "(:,1), B" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                    }
                                    std::cout << std::endl << std::flush;
                                }

                                /****************LA SEGUENTE E' UNA DELLE PEGGIORI SCHIFEZZE CHE ABBIA MAI FATTO, DA RIMUOVERE ASSOLUTAMENTE*******************************/

                                std::vector<std::shared_ptr<Vertex> > shortestPath;
                                std::vector<std::shared_ptr<Vertex> > shortestPath1 = Utilities::polylineDijkstra(v1, v2, true);
                                double dist1 = std::numeric_limits<double>::max(), dist2 = std::numeric_limits<double>::max();
                                if(shortestPath1.size() > 0)
                                {
                                    dist1 = ((*shortestPath1[0]) - (*v1)).norm();
                                    for(uint l = 1; l < shortestPath1.size(); l++)
                                       dist1 += ((*shortestPath1[l]) - (*shortestPath1[l - 1])).norm();
                                }
                                if(dist1 > 2  * (*v1 - *v2).norm())
                                {
                                    std::vector<std::shared_ptr<Vertex> > shortestPath2 = Utilities::polylineDijkstra(v2, v1, true);
                                    if(shortestPath2.size() > 0)
                                    {
                                        shortestPath2.erase(shortestPath2.begin() + shortestPath2.size() - 1);
                                        std::reverse(shortestPath2.begin(), shortestPath2.end());
                                        shortestPath2.insert(shortestPath2.end(), v2);
                                        dist2 = ((*shortestPath2[0]) - (*v1)).norm();
                                        for(uint l = 1; l < shortestPath2.size(); l++)
                                           dist2 += ((*shortestPath2[l]) - (*shortestPath2[l - 1])).norm();
                                    }
                                    if(dist1 == std::numeric_limits<double>::max() && dist2 == std::numeric_limits<double>::max())
                                    {
                                        std::cerr << "IMPOSSIBLE CASE" << std::endl;
                                        meshes[0]->save("witherror.ply", 15);

                                        uint counter = 0;
                                        for(auto vertexList : triangulationHoles.at(i))
                                        {
                                            std::cout << "TH" << counter << "=[" << std::endl;
                                            for(auto v : vertexList)
                                            {
                                                std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                            }
                                            std::cout << "];" << std::endl;
                                            std::cout << "plot(TH" << counter << "(:,1), TH" << counter << "(:,2));" << std::endl;
                                            std::cout << "labels=num2cell(0:length(TH" << counter << ")-1);" << std::endl;
                                            std::cout << "text(TH" << counter << "(:,1), TH" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                        }
                                        std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

                                        counter = 0;
                                        for(auto building : buildingsToBeTriangulated)
                                        {
                                            auto boundaries = building->getBoundaries();
                                            for(auto boundary : boundaries)
                                            {
                                                std::cout << "B" << counter << "=[" << std::endl;
                                                for(auto v : boundary)
                                                {
                                                    std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                                }
                                                std::cout << "];" << std::endl;
                                                std::cout << "plot(B" << counter << "(:,1), B" << counter << "(:,2));" << std::endl;
                                                std::cout << "labels=num2cell(0:length(B" << counter << ")-1);" << std::endl;
                                                std::cout << "text(B" << counter << "(:,1), B" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                            }
                                        }
                                        exit(223);
                                    }
                                    shortestPath = dist1 < dist2 ? shortestPath1 : shortestPath2;
                                } else
                                    shortestPath = shortestPath1;


                                if(shortestPath.size() > 1)
                                {
                                    shortestPath.pop_back();
                                    boundaries[0].insert(boundaries[0].begin() + j, shortestPath.begin(), shortestPath.end());
                                    j += shortestPath.size();
                                    triangulationHoles.at(i).at(0).insert(triangulationHoles.at(i).at(0).begin() + pos,
                                                                           shortestPath.begin(), shortestPath.end());
                                    pos += shortestPath.size();
                                }
                            }
                        } else
                        {
                            auto it = std::find_if(groupInnerVertices.begin(), groupInnerVertices.end(), [boundaries, j](std::shared_ptr<Vertex> v){ return *v == *boundaries[0][j];});
                            if(it == groupInnerVertices.end())
                            {
                                auto v = meshes[0]->addNewVertex(boundaries[0][j]);
                                *boundaries[0][j] = *v;
                                if(v->getId().compare("") == 0)
                                {
                                    v->setId(std::to_string(meshes[0]->getVerticesNumber() - 1));
                                }
                                groupInnerVertices.push_back(v);
                            }
                        }
                    }
                    for(uint j = 1; j < boundaries.size(); j++)
                    {
                        for(uint k = 1; k < boundaries.at(j).size(); k++)
                        {
                            std::shared_ptr<Vertex> v1 = std::static_pointer_cast<Vertex>(boundaries[j][k - 1]);
                            std::shared_ptr<Vertex> v2 = std::static_pointer_cast<Vertex>(boundaries[j][k]);

                            if(v1 == nullptr || v2 == nullptr ||
                               v1->getId().compare("") == 0 || v1->getE0() == nullptr ||
                               v2->getId().compare("") == 0 || v2->getE0() == nullptr)
                            {
                                std::cout << "Disconnected vertex!" << std::endl;
                                v1->print(std::cerr);
                                v2->print(std::cerr);
                                meshes[0]->save("witherror.ply", 15);

                                uint counter = 0;
                                for(auto vertexList : triangulationHoles.at(i))
                                {
                                    std::cout << "TH" << counter << "=[" << std::endl;
                                    for(auto v : vertexList)
                                    {
                                        std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                    }
                                    std::cout << "];" << std::endl;
                                    std::cout << "plot(TH" << counter << "(:,1), TH" << counter << "(:,2));" << std::endl;
                                    std::cout << "labels=num2cell(0:length(TH" << counter << ")-1);" << std::endl;
                                    std::cout << "text(TH" << counter << "(:,1), TH" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                }
                                std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

                                counter = 0;
                                for(auto building : buildingsToBeTriangulated)
                                {
                                    auto boundaries = building->getBoundaries();
                                    for(auto boundary : boundaries)
                                    {
                                        std::cout << "B" << counter << "=[" << std::endl;
                                        for(auto v : boundary)
                                        {
                                            std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                        }
                                        std::cout << "];" << std::endl;
                                        std::cout << "plot(B" << counter << "(:,1), B" << counter << "(:,2));" << std::endl;
                                        std::cout << "labels=num2cell(0:length(B" << counter << ")-1);" << std::endl;
                                        std::cout << "text(B" << counter << "(:,1), B" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                    }
                                }
                                std::cout << std::flush;
                                exit(88);
                            }
                            /****************LA SEGUENTE E' UNA DELLE PEGGIORI SCHIFEZZE CHE ABBIA MAI FATTO, DA RIMUOVERE ASSOLUTAMENTE*******************************/
                            std::vector<std::shared_ptr<Vertex> > shortestPath;
                            std::vector<std::shared_ptr<Vertex> > shortestPath1 = Utilities::dijkstra(v1, v2);
                            double dist1 = std::numeric_limits<double>::max(), dist2 = std::numeric_limits<double>::max();
                            if(shortestPath1.size() > 0)
                            {
                                dist1 = ((*shortestPath1[0]) - (*v1)).norm();
                                for(uint l = 1; l < shortestPath1.size(); l++)
                                   dist1 += ((*shortestPath1[l]) - (*shortestPath1[l - 1])).norm();
                            }

                            if(dist1 > 2 * (*v1 - *v2).norm())
                            {
                                std::vector<std::shared_ptr<Vertex> > shortestPath2 = Utilities::dijkstra(v2, v1);
                                if(shortestPath2.size() > 0)
                                {
                                    shortestPath2.erase(shortestPath2.begin() + shortestPath2.size() - 1);
                                    std::reverse(shortestPath2.begin(), shortestPath2.end());
                                    shortestPath2.insert(shortestPath2.end(), v2);
                                    dist2 = ((*shortestPath2[0]) - (*v1)).norm();
                                    for(uint l = 1; l < shortestPath2.size(); l++)
                                       dist2 += ((*shortestPath2[l]) - (*shortestPath2[l - 1])).norm();
                                }
                                if(dist1 == std::numeric_limits<double>::max() && dist2 == std::numeric_limits<double>::max())
                                {
                                    std::cerr << "IMPOSSIBLE CASE" << std::endl;
                                    meshes[0]->save("witherror.ply", 15);

                                    uint counter = 0;
                                    for(auto vertexList : triangulationHoles.at(i))
                                    {
                                        std::cout << "TH" << counter << "=[" << std::endl;
                                        for(auto v : vertexList)
                                        {
                                            std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                        }
                                        std::cout << "];" << std::endl;
                                        std::cout << "plot(TH" << counter << "(:,1), TH" << counter << "(:,2));" << std::endl;
                                        std::cout << "labels=num2cell(0:length(TH" << counter << ")-1);" << std::endl;
                                        std::cout << "text(TH" << counter << "(:,1), TH" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                    }
                                    std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

                                    counter = 0;
                                    for(auto building : buildingsToBeTriangulated)
                                    {
                                        auto boundaries = building->getBoundaries();
                                        for(auto boundary : boundaries)
                                        {
                                            std::cout << "B" << counter << "=[" << std::endl;
                                            for(auto v : boundary)
                                            {
                                                std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                            }
                                            std::cout << "];" << std::endl;
                                            std::cout << "plot(B" << counter << "(:,1), B" << counter << "(:,2));" << std::endl;
                                            std::cout << "labels=num2cell(0:length(B" << counter << ")-1);" << std::endl;
                                            std::cout << "text(B" << counter << "(:,1), B" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                        }
                                    }
                                    std::cout << std::flush;
                                    exit(223);
                                }

                                shortestPath = dist1 < dist2 ? shortestPath1 : shortestPath2;

                            } else
                                shortestPath = shortestPath1;

                            if(shortestPath.size() > 1)
                            {
                                shortestPath.pop_back();
                                boundaries[j].insert(boundaries[j].begin() + k, shortestPath.begin(), shortestPath.end());
                                k += shortestPath.size();
                            }

                        }
                    }

                    building->setBoundaries(boundaries);
                }


            } else
            {
                auto list = buildings;
                while(std::find_if(usedBuildings.begin(), usedBuildings.end(), [list, buildingPos](std::string s){
                                   return list.at(buildingPos)->getId().compare(s) == 0;
                                }) != usedBuildings.end())
                    buildingPos++;

                auto boundaries = buildings.at(buildingPos)->getBoundaries();

                for(uint j = 0; j < boundaries.size(); j++)
                {
                    for(uint k = 1; k < boundaries.at(j).size(); k++)
                    {
                        std::shared_ptr<Vertex> v1 = std::static_pointer_cast<Vertex>(boundaries[j][k - 1]);
                        std::shared_ptr<Vertex> v2 = std::static_pointer_cast<Vertex>(boundaries[j][k]);

                        if(v1 == nullptr || v2 == nullptr ||
                           v1->getId().compare("") == 0 || v1->getE0() == nullptr ||
                           v2->getId().compare("") == 0 || v2->getE0() == nullptr)
                        {
                            std::cout << "Disconnected vertex!" << std::endl;
                            v1->print(std::cerr);
                            v2->print(std::cerr);
                            meshes[0]->save("witherror.ply", 15);

                            for(uint j = 0; j < boundaries.size(); j++)
                            {
                                std::cout << "B"<< j << "=[" << std::endl;
                                for(uint k = 0; k < boundaries.at(j).size(); k++)
                                {
                                    std::static_pointer_cast<Point>(boundaries.at(j).at(k))->print(std::cout, BracketsType::NONE, " ");
                                }
                                std::cout << "];" << std::endl;
                                std::cout << "plot(B" << j << "(:,1), B" << j << "(:,2));" << std::endl;
                                std::cout << "labels=num2cell(0:length(B" << j << ")-1);" << std::endl;
                                std::cout << "text(B" << j << "(:,1), B" << j << "(:,2), labels);" << std::endl << std::endl;
                            }
                            std::cout << std::flush;
                            exit(88);
                        }
                        /****************LA SEGUENTE E' UNA DELLE PEGGIORI SCHIFEZZE CHE ABBIA MAI FATTO, DA RIMUOVERE ASSOLUTAMENTE*******************************/
                        std::vector<std::shared_ptr<Vertex> > shortestPath;
                        std::vector<std::shared_ptr<Vertex> > shortestPath1 = Utilities::polylineDijkstra(v1, v2);
                        double dist1 = std::numeric_limits<double>::max(), dist2 = std::numeric_limits<double>::max();
                        if(shortestPath1.size() > 0)
                        {
                            dist1 = ((*shortestPath1[0]) - (*v1)).norm();
                            for(uint l = 1; l < shortestPath1.size(); l++)
                               dist1 += ((*shortestPath1[l]) - (*shortestPath1[l - 1])).norm();
                        }

                        if(dist1 > 2 * (*v1 - *v2).norm())
                        {
                            std::vector<std::shared_ptr<Vertex> > shortestPath2 = Utilities::polylineDijkstra(v2, v1);
                            if(shortestPath2.size() > 0)
                            {
                                shortestPath2.erase(shortestPath2.begin() + shortestPath2.size() - 1);
                                std::reverse(shortestPath2.begin(), shortestPath2.end());
                                shortestPath2.insert(shortestPath2.end(), v2);
                                dist2 = ((*shortestPath2[0]) - (*v1)).norm();
                                for(uint l = 1; l < shortestPath2.size(); l++)
                                   dist2 += ((*shortestPath2[l]) - (*shortestPath2[l - 1])).norm();
                            }
                            if(dist1 == std::numeric_limits<double>::max() && dist2 == std::numeric_limits<double>::max())
                            {
                                std::cerr << "IMPOSSIBLE CASE" << std::endl;
                                meshes[0]->save("witherror.ply", 15);

                                uint counter = 0;
                                for(auto vertexList : triangulationHoles.at(i))
                                {
                                    std::cout << "TH" << counter << "=[" << std::endl;
                                    for(auto v : vertexList)
                                    {
                                        std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                                    }
                                    std::cout << "];" << std::endl;
                                    std::cout << "plot(TH" << counter << "(:,1), TH" << counter << "(:,2));" << std::endl;
                                    std::cout << "labels=num2cell(0:length(TH" << counter << ")-1);" << std::endl;
                                    std::cout << "text(TH" << counter << "(:,1), TH" << counter++ << "(:,2), labels);" << std::endl << std::endl;
                                }
                                std::vector<std::shared_ptr<Vertex> > groupInnerVertices;

                                std::cout << std::flush;
                                exit(223);
                            }

                            shortestPath = dist1 < dist2 ? shortestPath1 : shortestPath2;
                        } else
                            shortestPath = shortestPath1;

                        if(shortestPath.size() > 1)
                        {
                            shortestPath.pop_back();
                            boundaries[j].insert(boundaries[j].begin() + k, shortestPath.begin(), shortestPath.end());
                            k += shortestPath.size();
                        }
                    }
                }
                buildings.at(buildingPos)->setBoundaries(boundaries);

                buildingsToBeTriangulated = {buildings.at(buildingPos)};
                buildingPos++;
            }



            for(auto building : buildingsToBeTriangulated)
            {
                std::vector<uint> polyline;
                auto boundaries = building->getBoundaries();

                for(uint j = 0; j < boundaries.size(); j++)
                {
                    std::vector<uint> polyline;
                    uint first = reached_id;
                    for(uint k = 0; k < boundaries.at(j).size(); k++)
                    {
                        if(k < boundaries.at(j).size() - 1)
                        {
                            std::shared_ptr<Vertex> v = std::static_pointer_cast<Vertex>(boundaries.at(j).at(k));
                            std::vector<std::shared_ptr<Edge> > connected = std::static_pointer_cast<Vertex>(boundaries.at(j).at(k))->getVE();

                            auto it = verticesEdges.find(v->getId());
                            if(it == verticesEdges.end())
                                verticesEdges.insert(std::make_pair(v->getId(), connected));

                            auto pit = std::find_if(points.begin(), points.end(), [v](double* p) { return *v == p; });
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
                            polyline.push_back(first);
                    }

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

                for(auto pi : flaggedMeshVertices)
                    meshes[0]->getVertex(pi)->removeFlag(FlagType::INSIDE);

            }

            TriHelper::TriangleHelper helper(points, polylines, holes, false);
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
                t->setId(std::to_string(meshes[0]->getTrianglesNumber()));

                if(e1->getT1() == nullptr)
                    e1->setT1(t);
                else if(e1->getT2() == nullptr)
                    e1->setT2(t);
                else
                {
                    std::cerr << "Non manifold configuration!" << std::endl;
                    meshes[0]->save("witherror.ply",15);

                    std::cerr << "Buildings" << std::endl;
                    uint counter = 0;
                    for(auto building : buildingsToBeTriangulated)
                    {
                        std::cerr <<"%New building" << std::endl;
                        auto boundaries = building->getBoundaries();
                        for(auto boundary : boundaries)
                        {
                            std::cerr << "B" << counter << "=[" << std::endl;
                            for(auto p : boundary)
                            {
                                std::static_pointer_cast<Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                            }
                            std::cerr << "];" << std::endl;
                            std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                        }
                    }

                    std::cerr << "Triangulation holes" << std::endl;

                    for(uint k = 0; k < triangulationHoles.at(i).size(); k++)
                    {
                        std::cerr << "B" << k << "=[" << std::endl;
                        for(uint l = 0; l < triangulationHoles.at(i).at(k).size(); l++)
                        {
                            triangulationHoles.at(i).at(k).at(l)->print(std::cerr);
                        }
                        std::cerr << "];" << std::endl;

                    }
                    exit(123);
                }

                if(e2->getT1() == nullptr)
                    e2->setT1(t);
                else if(e2->getT2() == nullptr)
                    e2->setT2(t);
                else
                {
                    std::cerr << "Non manifold configuration!" << std::endl;


                    std::cerr << "Buildings" << std::endl;
                    uint counter = 0;
                    for(auto building : buildingsToBeTriangulated)
                    {
                        std::cerr <<"%New building" << std::endl;
                        auto boundaries = building->getBoundaries();
                        for(auto boundary : boundaries)
                        {
                            std::cerr << "B" << counter << "=[" << std::endl;
                            for(auto p : boundary)
                            {
                                std::static_pointer_cast<Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                            }
                            std::cerr << "];" << std::endl;
                            std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                        }
                    }

                    std::cerr << "Triangulation holes" << std::endl;

                    for(uint k = 0; k < triangulationHoles.at(i).size(); k++)
                    {
                        std::cerr << "B" << k << "=[" << std::endl;
                        for(uint l = 0; l < triangulationHoles.at(i).at(k).size(); l++)
                        {
                            triangulationHoles.at(i).at(k).at(l)->print(std::cerr);
                        }
                        std::cerr << "];" << std::endl;

                    }


                    std::cout << "Structure" << std::endl;
                    for_each(points.begin(), points.end(), [](double* p) { std::cout << p[0] << " " << p[1] << std::endl;});
                    for_each(enclosing_triangles.begin(), enclosing_triangles.end(), [](uint id) { std::cout << id << std::endl;});
                    std::cout << "Triangle:" << std::endl;
                    std::cout << points[enclosing_triangles[j * 3]][0] << " " << points[enclosing_triangles[j * 3]][1] << std::endl;
                    std::cout << points[enclosing_triangles[j * 3 + 1]][0] << " " << points[enclosing_triangles[j * 3 + 1]][1] << std::endl;
                    std::cout << points[enclosing_triangles[j * 3 + 2]][0] << " " << points[enclosing_triangles[j * 3 + 2]][1] << std::endl;
                    exit(123);
                }

                if(e3->getT1() == nullptr)
                    e3->setT1(t);
                else if(e3->getT2() == nullptr)
                    e3->setT2(t);
                else
                {
                    std::cerr << "Non manifold configuration!" << std::endl;

                    std::cerr << "Buildings" << std::endl;
                    uint counter = 0;
                    for(auto building : buildingsToBeTriangulated)
                    {
                        std::cerr <<"%New building" << std::endl;
                        auto boundaries = building->getBoundaries();
                        for(auto boundary : boundaries)
                        {
                            std::cerr << "B" << counter << "=[" << std::endl;
                            for(auto p : boundary)
                            {
                                std::static_pointer_cast<Point>(p)->print(std::cerr, BracketsType::NONE, " ");
                            }
                            std::cerr << "];" << std::endl;
                            std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
                        }
                    }

                    std::cerr << "Triangulation holes" << std::endl;

                    for(uint k = 0; k < triangulationHoles.at(i).size(); k++)
                    {
                        std::cerr << "B" << k << "=[" << std::endl;
                        for(uint l = 0; l < triangulationHoles.at(i).at(k).size(); l++)
                        {
                            triangulationHoles.at(i).at(k).at(l)->print(std::cerr);
                        }
                        std::cerr << "];" << std::endl;

                    }
                    exit(123);
                }

            }
            //meshes[0]->save("debug.ply");
            debugPoints.clear();
            debugTriangles.clear();


            std::cout << i * 100 / triangulationHoles.size() << "%\r" << std::flush;
        }


        points_vector.clear();
        meshKDTree.reset();
//        for(auto p : nodes)
//        {
//            *p.second->getCoordinates() *= city_size;
//            *p.second->getCoordinates() += *origin;
//        }

        for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        {
            std::shared_ptr<Vertex> v = meshes[0]->getVertex(i);
            v->setX(v->getX() * city_size + origin->getX());
            v->setY(v->getY() * city_size + origin->getY());
            v->setZ(v->getZ() * city_size + origin->getZ());
        }

        //pixelToBuildingAssociation = extractBuildingsHeights(true);
        //auto buildingsHeights = extractBuildingsHeightsFromLidar();
        std::cout << "Ended" << std::endl << "Adapting elevations to DTM" << std::endl;
        //Utilities::associateHeights(meshes[0].get(), dtm.get(), 1, Point(0,0,0));

        origin.reset();

        std::cout << "Ended" << std::endl << "Associating elevations" << std::endl;
        //associateElevations(triangulationHoles, arcsPoints);

        for(auto building : buildings)
        {
            auto boundaries = building->getBoundaries();
            for(auto boundary : boundaries)
                for(auto v : boundary)
                    v->addFlag(FlagType::ON_BOUNDARY);
        }

        uint c = 0;
        std::cout << "0%";
        for(auto building : buildings)
        {
            auto boundaries = building->getBoundaries();
            for(auto boundary : boundaries)
            {
                for(uint i = 1; i < boundary.size(); i++)
                {
                    auto v = boundary[i];
                    uint size = 1, counter = 0;
                    double mean;
                    do
                    {
                        mean = 0;
                        std::vector<std::shared_ptr<Vertex> > neighbourhood = v->getNeighbourhood(size++);
                        if(neighbourhood.size() == 0)
                        {
                            std::cerr << "Disconnected vertex" << v->getId() << std::endl;
                            break;
                        }
                        for(auto v_ : neighbourhood)
                            if(v_->searchFlag(FlagType::ON_BOUNDARY) < 0 && v_->getZ() != 0.0)
                            {
                                mean += v_->getZ();
                                counter++;
                            }
                        if(counter != 0)
                            mean /= counter;
                    } while(counter == 0);
                    v->setZ(mean);
                }
            }
            std::cout << c++ * 100 / buildings.size() << "%\r" << std::flush;

        }
        for(auto building : buildings)
        {
            auto boundaries = building->getBoundaries();
            for(auto boundary : boundaries)
                for(auto v : boundary)
                    v->removeFlag(FlagType::ON_BOUNDARY);
        }

        for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        {
            auto v = meshes[0]->getVertex(i);
            if(v->getZ() == 0.0)
            {
                uint size = 1, counter = 0;
                double mean;
                do
                {
                    mean = 0;
                    std::vector<std::shared_ptr<Vertex> > neighbourhood = v->getNeighbourhood(size++);
                    if(neighbourhood.size() == 0)
                    {
                        std::cerr << "Disconnected vertex" << v->getId() << std::endl;
                        break;
                    }
                    for(auto v_ : neighbourhood)
                        if(v_->getZ() != 0.0)
                        {
                            mean += v_->getZ();
                            counter++;
                        }
                    if(counter != 0)
                        mean /= counter;
                } while(counter == 0);
                v->setZ(mean);
            }
        }
        std::cout << "Associating heights to buildings." << std::endl;
        float** elevations = dtm->GetRasterBand(1);

        meshes[0]->save("cacco2.ply",15);
        for(uint i = 0; i < buildings.size(); i++)
        {
            int row_min = -1, col_min = -1, row_max = 0, col_max = 0;
            double elevation_min = std::numeric_limits<double>::max(),
                   elevation_max = -std::numeric_limits<double>::max(),
                   elevation_mean = 0;
            //Computing min-max-mean height values
//            for(uint k = 0; k < pixelToBuildingAssociation.at(i).size(); k++)
//            {

//                elevation_mean += static_cast<double>(elevations[pixelToBuildingAssociation.at(i)[k].first][pixelToBuildingAssociation.at(i)[k].second]);
//                if(elevation_min > static_cast<double>(elevations[pixelToBuildingAssociation.at(i)[k].first][pixelToBuildingAssociation.at(i)[k].second]))
//                {
//                    row_min = static_cast<int>(pixelToBuildingAssociation.at(i)[k].first);
//                    col_min = static_cast<int>(pixelToBuildingAssociation.at(i)[k].second);
//                    elevation_min = static_cast<double>(elevations[pixelToBuildingAssociation.at(i)[k].first][pixelToBuildingAssociation.at(i)[k].second]);
//                }

//                if(elevation_max < static_cast<double>(elevations[pixelToBuildingAssociation.at(i)[k].first][pixelToBuildingAssociation.at(i)[k].second]))
//                {
//                    row_max = static_cast<int>(pixelToBuildingAssociation.at(i)[k].first);
//                    col_max = static_cast<int>(pixelToBuildingAssociation.at(i)[k].second);
//                    elevation_max = static_cast<double>(elevations[pixelToBuildingAssociation.at(i)[k].first][pixelToBuildingAssociation.at(i)[k].second]);
//                }

//            }

//            elevation_mean /= static_cast<double>(pixelToBuildingAssociation.at(i).size());

//            std::for_each(buildingsHeights.at(i).begin(), buildingsHeights.at(i).end(), [&elevation_mean, &elevation_min, &elevation_max](double d){
//                elevation_mean += d;
//                if(elevation_min > d)
//                    elevation_min = d;
//                if(elevation_max < d)
//                    elevation_max = d;
//            });
//            elevation_mean /= buildingsHeights.at(i).size();

            auto boundaries = buildings.at(i)->getBoundaries();
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

            boundaries = buildings.at(i)->getBoundaries();
            for(uint j = 0; j < boundaries.size(); j++)
                for(uint k = 0; k < boundaries.at(j).size() - 1; k++)
                    boundaries.at(j).at(k)->setZ(elevation_mean);
        }
        std::cout <<"Ended!" << std::endl;

        /*std::cout << "Reconstructing streets' arcs" << std::endl;

        for(uint i = 0; i < buildings.size(); i++)
        {
            auto boundaries = buildings[i]->getBoundaries();
            for(uint j = 0; j < boundaries.size(); j++)
                for(uint k = 0; k < boundaries.at(j).size() - 1; k++)
                    boundaries.at(j).at(k)->addFlag(FlagType::ON_BOUNDARY);
        }

        for(uint i = 0; i < arcsPoints.size(); i++)
        {
            for(uint j = 1; j < arcsPoints.at(i).size(); j++)
            {
                auto v1 = arcsPoints.at(i).at(j - 1);
                auto v2 = arcsPoints.at(i).at(j);
                if(v1->searchFlag(FlagType::ON_BOUNDARY) < 0 && v2->searchFlag(FlagType::ON_BOUNDARY) < 0 )
                {
                    std::vector<std::shared_ptr<Vertex> > shortestPath1 = Utilities::dijkstra(v1, v2);
                    std::vector<std::shared_ptr<Vertex> > shortestPath2 = Utilities::dijkstra(v2, v1);
                    double dist1 = std::numeric_limits<double>::max(), dist2 = std::numeric_limits<double>::max();
                    if(shortestPath1.size() > 0)
                    {
                        dist1 = ((*shortestPath1[0]) - (*v1)).norm();
                        for(uint l = 1; l < shortestPath1.size(); l++)
                           dist1 += ((*shortestPath1[l]) - (*shortestPath1[l - 1])).norm();
                    }

                    if(shortestPath2.size() > 0)
                    {
                        shortestPath2.erase(shortestPath2.begin() + shortestPath2.size() - 1);
                        std::reverse(shortestPath2.begin(), shortestPath2.end());
                        shortestPath2.insert(shortestPath2.end(), v2);
                        dist2 = ((*shortestPath2[0]) - (*v1)).norm();
                        for(uint l = 1; l < shortestPath2.size(); l++)
                           dist2 += ((*shortestPath2[l]) - (*shortestPath2[l - 1])).norm();
                    }
                    if(dist1 == std::numeric_limits<double>::max() && dist2 == std::numeric_limits<double>::max())
                    {
                        std::cerr << "IMPOSSIBLE CASE" << std::endl;
                        meshes[0]->save("witherror.ply", 15);

                        std::cout << i << " " << j << std::endl;

                        std::cout << "A=[" << std::endl;
                        for(auto v : arcsPoints.at(i))
                        {
                            std::static_pointer_cast<Point>(v)->print(std::cout, BracketsType::NONE, " ");
                        }
                        std::cout << "];" << std::endl;

                        exit(223);
                    }

                    std::vector<std::shared_ptr<Vertex> > shortestPath = dist1 < dist2 ? shortestPath1 : shortestPath2;

                    if(shortestPath.size() > 1)
                    {
                        shortestPath.pop_back();
                        arcsPoints.at(i).insert(arcsPoints.at(i).begin() + j, shortestPath.begin(), shortestPath.end());
                        j += shortestPath.size();
                    }
                }
            }
        }

        for(uint i = 0; i < buildings.size(); i++)
        {
            auto boundaries = buildings[i]->getBoundaries();
            for(uint j = 0; j < boundaries.size(); j++)
                for(uint k = 0; k < boundaries.at(j).size() - 1; k++)
                    boundaries.at(j).at(k)->removeFlag(FlagType::ON_BOUNDARY);
        }

        std::cout << "Ended!" << std::endl;*/
        std::cout <<"Creating annotations." << std::endl;
        unsigned char building_color[] = {255, 215, 0};

        unsigned char street_color[] = {0, 0, 0};
//        unsigned char green[] = {0, 255, 0};
//        unsigned char yellowgreen[] = {210,255,105};
//        unsigned char yellow[] = {255,255,0};
//        unsigned char orange[] = {255,131,0};
//        unsigned char salmon[] = {255,160,122};
//        unsigned char red[] = {255, 0, 0};
//        unsigned char violet[] = {155,38,182};
        std::map<OSMNode*, std::shared_ptr<Vertex> > traversed_nodes;

        for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
        {
           auto v = meshes[0]->getVertex(i);
           auto p = {v->getX(), v->getY(), 0.0};
           points_vector.push_back(p);

        }
        meshKDTree = std::make_shared<KDTree>(points_vector);
        uint i = 0;
        for(auto ait = streetsArcs.begin(); ait != streetsArcs.end(); ait++)
        {
            std::shared_ptr<LineAnnotation> annotation = std::make_shared<LineAnnotation>();
            std::vector<std::shared_ptr<Vertex> > annotationPolyline;
            annotation->setId(i);
            annotation->setTag("street n° " + std::to_string(i));
            annotation->setHierarchyLevel(0);
            annotation->setColor(street_color);
            std::vector<uint> flaggedMeshVertices;
            for(uint j = 1; j < arcsPoints.at(i).size(); j++)
            {
                auto v1 = arcsPoints.at(i).at(j - 1);
                auto v2 = arcsPoints.at(i).at(j);

                auto p = ((*v1) + (*v2)) / 2;
                auto point = {p.getX(), p.getY(), 0.0};
                auto neighbourhood = meshKDTree->neighborhood(point, (*v2 - *v1).norm() / 2 + Point::EPSILON);
                for(auto pi : neighbourhood)
                {
                    meshes[0]->getVertex(pi.second)->addFlag(FlagType::INSIDE);
                    flaggedMeshVertices.push_back(pi.second);
                }

            }
            for(uint j = 1; j < arcsPoints.at(i).size(); j++)
            {
                auto v1 = arcsPoints.at(i).at(j - 1);
                auto v2 = arcsPoints.at(i).at(j);

                auto path = Utilities::polylineDijkstra(v1, v2, true);
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

            std::ofstream stream("annotation_positions.m");
            for(auto p : flaggedMeshVertices)
                meshes[0]->getVertex(p)->removeFlag(FlagType::INSIDE);
            for(uint j = 0; j < arcsPoints.at(i).size(); j++)
            {
                std::shared_ptr<Vertex> v = std::static_pointer_cast<Vertex>(arcsPoints.at(i).at(j));
                uint id = static_cast<uint>(stoi(v->getId()));
                if(arcsPoints.at(i).at(j)->getInfo() != nullptr)
                {
                    OSMNode* n = static_cast<OSMNode*>(arcsPoints.at(i).at(j)->getInfo());
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

        i = 0;
        for(auto it = traversed_nodes.begin(); it != traversed_nodes.end(); it++)
        {

            std::shared_ptr<PointAnnotation> annotation = std::make_shared<PointAnnotation>();
            annotation->setId(static_cast<uint>(streetsArcs.size()) - 1 + i);
            annotation->setTag("node n° " + std::to_string(i++));
            annotation->setHierarchyLevel(0);
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



        uint bid = static_cast<uint>(streetsArcs.size()) + traversed_nodes.size();


        for(uint i = 0; i < buildings.size(); i++)
        {

            auto boundaries = buildings[i]->getBoundaries();
            std::shared_ptr<SurfaceAnnotation> annotation = std::make_shared<SurfaceAnnotation>();
            annotation->setId(bid);
            annotation->setTag("building n° " + std::to_string(bid++));
            annotation->setHierarchyLevel(0);
            annotation->setColor(building_color);

            for(uint j = 0; j < boundaries.size(); j++)
            {
                std::vector<std::shared_ptr<Vertex> > annotationBoundary;
                for(uint k = 0; k < boundaries.at(j).size(); k++)
                    annotationBoundary.push_back(std::static_pointer_cast<Vertex>(boundaries.at(j).at(k)));
                annotation->addOutline(annotationBoundary);
            }
            annotation->setMesh(meshes[0]);
            meshes[0]->addAnnotation(annotation);
            auto height = annotation->getOutlines()[0][0]->getZ();
            auto involved = annotation->getInvolvedVertices();

            for(uint j = 0; j < involved.size(); j++)
                involved.at(j)->setZ(height);
        }
        meshes[0]->removeIsolatedVertices();
        for(uint i = 0; i < meshes[0]->getVerticesNumber(); i++)
            meshes[0]->getVertex(i)->setId(std::to_string(i));
        for(uint i = 0; i < meshes[0]->getEdgesNumber(); i++)
            meshes[0]->getEdge(i)->setId(std::to_string(i));
        for(uint i = 0; i < meshes[0]->getTrianglesNumber(); i++)
            meshes[0]->getTriangle(i)->setId(std::to_string(i));
        AnnotationFileManager manager;
        manager.setMesh(meshes[0]);
        manager.writeAnnotations("annotations.ant");
        meshes[0]->save("level0.ply", 15);


    } else
        return -1;
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
        auto boundaries = buildings[i]->getBoundaries();

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
                cloned_outline.push_back(meshes[1]->addNewVertex(*std::static_pointer_cast<Point>(v)));
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
    AnnotationFileManager manager;
    manager.setMesh(meshes[1]);
    manager.writeAnnotations("AnnotationsLOD1.ant");

    std::vector<std::shared_ptr<Annotation> > annotations = meshes[1]->getAnnotations();
    for(auto annotation : annotations)
    {
        auto tag = annotation->getTag();
        std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c){return std::tolower(c); });
        if(tag.find("building") != std::string::npos)
        {
            buildingsAnnotations.push_back(annotation);
            meshes[1]->removeAnnotation(pos);
            pos--;
        }
        pos++;
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
    AnnotationFileManager manager;
    manager.setMesh(this->meshes[0]);
    manager.readAnnotations(annotationFileName);

    uint counter = 0;
    std::ofstream stream("buidlings.m");
    for(auto annotation : meshes[0]->getAnnotations())
    {
        auto tag = annotation->getTag();
        std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c) { return std::tolower(c); });
        if(tag.find("building") != std::string::npos)
        {
            auto buildingAnnotation = std::dynamic_pointer_cast<SurfaceAnnotation>(annotation);
            auto building = std::make_shared<Building>();
            building->setBoundaries(buildingAnnotation->getOutlines());
            building->setId(std::to_string(reachedId++));
            buildings.push_back(building);
        }
    }
    //pixelToBuildingAssociation = extractBuildingsHeights(true);
}

void CityGMLCore::setLevel1(std::string meshFileName, std::string annotationFileName)
{

}
