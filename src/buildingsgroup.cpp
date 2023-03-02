#include "buildingsgroup.h"
#include "utilities.h"
#include "node.h"

#include <map>
#include <queue>
#include <set>

using namespace SemantisedTriangleMesh;
typedef unsigned int uint;

BuildingsGroup::BuildingsGroup()
{
    adjacencyGraph = new GraphTemplate::Graph<std::shared_ptr<Building> > ();
}

BuildingsGroup::~BuildingsGroup()
{
    delete adjacencyGraph;
}

std::vector<std::shared_ptr<Building> > BuildingsGroup::getBuildings() const
{
    return buildings;
}

void BuildingsGroup::setBuildings(const std::vector<std::shared_ptr<Building> > &value)
{
    buildings = value;
}

void BuildingsGroup::addBuilding(const std::shared_ptr<Building> &value)
{
    buildings.push_back(value);
}

void BuildingsGroup::computeAdjacencyGraph()
{
    if(adjacencyGraph != nullptr)
        delete adjacencyGraph;
    adjacencyGraph = new GraphTemplate::Graph<std::shared_ptr<Building> > ();
    std::map<std::string, GraphTemplate::Node<std::shared_ptr<Building> >* > buildingIdToNode;
    for(auto building : buildings)
    {
        auto n = new GraphTemplate::Node<std::shared_ptr<Building> >();
        n->setData(building);
        adjacencyGraph->addNode(n);
        buildingIdToNode.insert(std::make_pair(building->getId(), n));
    }
    for(auto building : buildings)
    {
        for(auto adjacent : building->getAdjacentBuildings())
        {
            uint adjacenciesCounter = 0;
            bool adjacencyBegun = false;
            auto boundaries1 = building->getBoundaries();
            auto boundaries2 = adjacent->getBoundaries();
            bool modified = false;
            std::vector<std::pair<uint, uint> > adjacenciesIndexes;
            uint begin, end;
            for (uint i = 0; i < boundaries1.at(0).size() - 1; i++) {
                auto it = std::find_if(boundaries2.at(0).begin(), boundaries2.at(0).end(), [&](std::shared_ptr<Point> p)
                {
                   return *boundaries1.at(0).at(i) == *p;
                });
                if(it != boundaries2.at(0).end())
                {
                    OpenStreetMap::Node* n1 = static_cast<OpenStreetMap::Node*>(boundaries1.at(0).at(i)->getInfo());
                    OpenStreetMap::Node* n2 = static_cast<OpenStreetMap::Node*>((*it)->getInfo());

                    if(n1 != nullptr && n2 != nullptr && n1->getId().compare(n2->getId()) != 0)
                    {
                        it = boundaries2.at(0).erase(it);
                        boundaries2.at(0).insert(it, boundaries1.at(0).at(i));
                        modified = true;
                        std::cout << "Repeated node" << std::endl;
                    }
                    if(!adjacencyBegun)
                    {
                        begin = i;
                        adjacenciesCounter++;
                    }
                    adjacencyBegun = true;
                    if(i == boundaries1.at(0).size() - 2 && adjacenciesIndexes.size() > 0)
                        adjacenciesIndexes.at(0).first = begin;
                } else if(adjacencyBegun)
                {
                    end = i - 1;
                    adjacenciesIndexes.push_back(std::make_pair(begin, end));
                }
            }
            if(modified)
                adjacent->setBoundaries(boundaries2);
            auto a = new GraphTemplate::Arc<std::shared_ptr<Building> >();
            a->setN1(buildingIdToNode.at(building->getId()));
            a->setN2(buildingIdToNode.at(adjacent->getId()));
            a->setDirected(false);
            a->setLabel("adjacency");
            adjacencyGraph->addArc(a);
        }
    }

}

std::vector<VertexList> BuildingsGroup::extractOverallBasePolygon()
{

    std::vector<VertexList> overallBasePolygons;

    std::vector<std::pair<std::shared_ptr<Vertex>, std::vector<std::pair<uint, uint> > > > pointToSegments; //Segments starting from the point in counterclockwise order (1 for each polygon)

    uint buildingPos = 0;
    std::shared_ptr<Vertex> guaranteedOnOuter;
    double minX = std::numeric_limits<double>::max();

    uint counter = 0;
    for(auto building : buildings)
    {
        uint position = 0;
        auto boundary = building->getBoundaries()[0];
        for(uint i = 0; i < boundary.size() - 1; i++)
        {
            auto point = boundary.at(i);

            if(point->getX() < minX)
            {
                minX = point->getX();
                guaranteedOnOuter = point;
            }
            auto p = std::make_pair(buildingPos, position++);
            auto it = std::find_if(pointToSegments.begin(), pointToSegments.end(),
                        [point](std::pair<std::shared_ptr<Vertex>, std::vector<std::pair<uint, uint> > > pair)
                        {
                           return *(pair.first) == *point;
                        });
            if(it == pointToSegments.end())
            {
                std::vector<std::pair<uint, uint> > segments = {p};
                pointToSegments.push_back(std::make_pair(point, segments));
            } else
            {
                it->second.push_back(p);
            }
        }
        buildingPos++;
    }

    std::shared_ptr<Vertex> p1 = guaranteedOnOuter, p2;
    minX = std::numeric_limits<double>::max();
    auto it = std::find_if(pointToSegments.begin(), pointToSegments.end(),
                [p1](std::pair<std::shared_ptr<Vertex>, std::vector<std::pair<uint, uint> > > pair)
                {
                   return *(pair.first) == *p1;
                });
    for(auto segment : it->second)
    {
        auto p = buildings.at(segment.first)->getBoundaries()[0].at(segment.second + 1);
        if(p->getX() < minX)
        {
            minX = p->getX();
            p2 = p;
        }
    }

    //p1 and p2 are guaranteed to be on the final outer boundary
    auto begin = p1;
    VertexList outsideBoundary;
    while(*p2 != *begin)
    {
        outsideBoundary.push_back(p1);
        it = std::find_if(pointToSegments.begin(), pointToSegments.end(),
                        [p2](std::pair<std::shared_ptr<Vertex>, std::vector<std::pair<uint, uint> > > pair)
                        {
                           return *(pair.first) == *p2;
                        });
        auto segments = it->second;
        size_t chosen = segments.size(), pos = 0;
        double maxAngle = -std::numeric_limits<double>::max();
        for(auto segment : segments)
        {
            auto p = buildings.at(segment.first)->getBoundaries()[0].at(segment.second + 1);

            double p1z = p1->getZ();
            double p2z = p2->getZ();
            double pz = p->getZ();
            p1->setZ(0);
            p2->setZ(0);
            p->setZ(0);
            double angle = Point::orientation(*p1, *p2, *p) * (*p2 - *p1).computeAngle(*p - *p2);
            p1->setZ(p1z);
            p2->setZ(p2z);
            p->setZ(pz);
            if(angle > maxAngle)
            {
                maxAngle = angle;
                chosen = pos;
            }

            pos++;
        }
        if(chosen < segments.size())
        {
            auto segment = segments.at(chosen);
            p1 = p2;
            p2 = buildings.at(segment.first)->getBoundaries()[0].at(segment.second + 1);
        } else
        {
            std::cerr << "IMPOSSIBLE: no segment is selected as next candidate for exterior boundary extraction" << std::endl;
            exit(12); ///PROBLEM!
        }
    }
    outsideBoundary.push_back(p1);
    outsideBoundary.push_back(begin);
    overallBasePolygons.push_back(outsideBoundary);

    for(auto building : buildings)
    {
        auto boundaries = building->getBoundaries();
        if(building->getBoundaries().size() > 1)
            overallBasePolygons.insert(overallBasePolygons.end(), boundaries.begin() + 1, boundaries.end());
    }

    return overallBasePolygons;
}

std::string BuildingsGroup::getId() const
{
    return id;
}

void BuildingsGroup::setId(const std::string &value)
{
    id = value;
}
