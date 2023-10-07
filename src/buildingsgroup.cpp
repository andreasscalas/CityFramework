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
    for(const auto& building : buildings)
    {
        auto n = new GraphTemplate::Node<std::shared_ptr<Building> >();
        n->setData(building);
        adjacencyGraph->addNode(n);
        buildingIdToNode.insert(std::make_pair(building->getId(), n));
    }
    for(const auto& building : buildings)
    {
        for(auto adjacent : building->getAdjacentBuildings())
        {
            uint adjacenciesCounter = 0;
            bool adjacencyBegun = false;
            auto boundaries1 = building->getOutlines();
            auto boundaries2 = adjacent->getOutlines();
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
                adjacent->setOutlines(boundaries2);
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

    std::map<std::shared_ptr<Vertex>, std::vector<std::pair<uint, uint> > > pointToSegments; //Segments starting from the point in counterclockwise order (1 for each polygon)

    uint buildingPos = 0;
    std::shared_ptr<Vertex> guaranteedOnOuter;
    double minX = std::numeric_limits<double>::max();

    uint counter = 0;
    if(id.compare("20") == 0)
    {
        for(const auto &building : buildings)
        {
            auto boundary = building->getOutlines()[0];
            std::cerr << "B" << counter << "=[" << std::endl;
            for(const auto& p : boundary)
                p->print(std::cerr);
            std::cerr << "];" << std::endl;
            std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;
        }

        counter = 0;
        for(const auto &building : buildings)
        {
            auto boundary = building->getOutlines()[0];
            std::cerr << "B" << counter << "=[" << std::endl;
            for(const auto& p : boundary)
            {
                std::static_pointer_cast<SemantisedTriangleMesh::Point>(p)->print(std::cerr, BracketsType::NONE, " ");
            }
            std::cerr << "];" << std::endl;
            std::cerr << "plot(B" << counter << "(:,1), B" << counter++ << "(:,2));" << std::endl;

        }
    }
    for(const auto &building : buildings)
    {
        uint position = 0;
        auto boundary = building->getOutlines()[0];
        for(uint i = 0; i < boundary.size() - 1; i++)
        {
            auto point = boundary.at(i);

            if(point->getX() < minX)
            {
                minX = point->getX();
                guaranteedOnOuter = point;
            }
            auto p = std::make_pair(buildingPos, position++);
            auto it = pointToSegments.find(point);
            if(it == pointToSegments.end())
            {
                std::vector<std::pair<uint, uint> > segments = {p};
                pointToSegments.insert(std::make_pair(point, segments));
            } else
            {
                it->second.push_back(p);
            }
        }
        buildingPos++;
    }

    std::shared_ptr<Vertex> p1 = guaranteedOnOuter, p2;
    minX = std::numeric_limits<double>::max();
    auto it = pointToSegments.find(p1);
    for(auto segment : it->second)
    {
        auto p = buildings.at(segment.first)->getOutlines()[0].at(segment.second + 1);
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
        p1->addFlag(FlagType::OUTSIDE);
        it = pointToSegments.find(p2);
        auto segments = it->second;
        size_t chosen = segments.size(), pos = 0;
        double maxAngle = -std::numeric_limits<double>::max();
        for(auto segment : segments)
        {
            auto p = buildings.at(segment.first)->getOutlines()[0].at(segment.second + 1);

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
            p2 = buildings.at(segment.first)->getOutlines()[0].at(segment.second + 1);
        } else
        {
            std::cerr << "IMPOSSIBLE: no segment is selected as next candidate for exterior boundary extraction" << std::endl;
            exit(12); ///PROBLEM!
        }
    }
    outsideBoundary.push_back(p1);
    p1->addFlag(FlagType::OUTSIDE);
    outsideBoundary.push_back(begin);
    overallBasePolygons.push_back(outsideBoundary);
    counter = 0;

    std::queue<std::shared_ptr<Vertex> > queue;
    for(uint i = 0; i < buildings.size(); i++)
        for(uint j = i + 1; j < buildings.size(); j++)
        {
            bool boundary1restarted = false;
            auto boundary1 = buildings.at(i)->getOutlines().at(0);
            auto boundary2 = buildings.at(j)->getOutlines().at(0);
            bool breaked = false;
            std::reverse(boundary2.begin(), boundary2.end());
            uint k = 0, l = 0;
            while (k < boundary1.size() - 1 && l < boundary2.size() - 1) {
                if (boundary1.at(k).get() == boundary2.at(l).get() || *boundary1.at(k) == *boundary2.at(l)) {

                    if(k > 0)
                    {
                        if(boundary1.at(k)->searchFlag(FlagType::OUTSIDE) == -1)
                            if(boundary1.at(k)->searchFlag(FlagType::ON_HOLE) == -1)
                            {
                                queue.push(boundary1.at(k));
                                boundary1.at(k)->print(std::cout);
                                boundary1.at(k)->addFlag(FlagType::ON_HOLE);
                            } else
                                boundary1.at(k)->addFlag(FlagType::INSIDE);
                    }
                    while (boundary1.at(k) == boundary2.at(l)) {
                        k++;
                        l++;
                        if( k == boundary1.size() - 1)
                            if(!boundary1restarted)
                            {
                                k = 0;
                                boundary1restarted = true;
                            } else
                            {
                                breaked = true;
                                break;
                            }

                        if(l == boundary2.size() - 1)
                            l = 0;
                    }

                    if(!breaked)
                    {
                        uint pos = Utilities::mod(static_cast<int>(k) - 1, boundary1.size() - 1);

                        if(boundary1.at(pos)->searchFlag(FlagType::OUTSIDE) == -1)
                            if(boundary1.at(pos)->searchFlag(FlagType::ON_HOLE) == -1)
                            {
                                queue.push(boundary1.at(pos));
                                boundary1.at(pos)->print(std::cout);
                                boundary1.at(pos)->addFlag(FlagType::ON_HOLE);
                            } else
                                boundary1.at(pos)->addFlag(FlagType::INSIDE);
                    }
                } else
                {
                    if(l + 1 == boundary2.size() - 1)
                    {
                        k++;
                        l = 0;
                    } else
                        l++;
                }
            }
        }

    for(uint i = 0; i < buildings.size(); i++)
    {
        auto boundary1 = buildings.at(i)->getOutlines().at(0);
        for(uint j = 0; j < boundary1.size(); j++)
        {
            uint counter = 0;
            for(uint k = 0; k < buildings.size(); k++)
            {
                if(i == k) continue;
                auto boundary2 = buildings.at(k)->getOutlines().at(0);
                for(uint l = 0; l < boundary2.size(); l++)
                {
                    if(boundary1.at(j).get() == boundary2.at(l).get())
                        counter++;
                }
            }
            if(counter == 0 &&
               boundary1.at(j)->searchFlag(FlagType::ON_HOLE) == -1 &&
               boundary1.at(j)->searchFlag(FlagType::OUTSIDE) == -1 )
            {
                boundary1.at(j)->addFlag(FlagType::ON_HOLE);

            }

        }
    }


    while(queue.size() > 0)
    {
        uint counter=0;

        auto v = queue.front();
        queue.pop();
        if(v->searchFlag(FlagType::USED) >= 0 || v->searchFlag(FlagType::INSIDE) >= 0) continue;

        std::vector<std::shared_ptr<Vertex> > innerBoundary;


        auto begin = v;
        std::cout << "Begin" << std::endl;
        std::dynamic_pointer_cast<Point>(begin)->print(std::cout, BracketsType::NONE, " ");
        std::cout << "Vertices" << std::endl;
        do
        {

            std::shared_ptr<Vertex> prev = nullptr;
            if(innerBoundary.size() > 0)
                prev = innerBoundary.back();
            innerBoundary.push_back(v);

            if(id.compare("20") == 0)
            {
                v->print(std::cout);
            }
            auto segments = pointToSegments.at(v);
            if(segments.size() == 1)
            {
                auto s = segments.at(0);
                v = buildings.at(s.first)->getOutlines()[0].at(s.second + 1);
            } else {
                bool changed = false;
                std::vector<std::pair<std::shared_ptr<Vertex>, double> > ordered;
                for(uint pos = 0; pos < segments.size(); pos++)
                {
                    auto s = segments.at(pos);
                    auto p = buildings.at(s.first)->getOutlines()[0].at(s.second + 1);
                    if(p->searchFlag(FlagType::OUTSIDE) >= 0) continue;
                    if(prev != nullptr)
                    {
                        double p1z = prev->getZ();
                        double p2z = v->getZ();
                        double pz = p->getZ();
                        prev->setZ(0);
                        v->setZ(0);
                        p->setZ(0);
                        double angle = Point::orientation(*prev, *v, *p) * (*v - *prev).computeAngle(*p - *v);
                        prev->setZ(p1z);
                        v->setZ(p2z);
                        p->setZ(pz);
                        ordered.push_back(std::make_pair(p, angle));
                    } else
                        ordered.push_back(std::make_pair(p, 0));
                }
                std::sort(ordered.begin(), ordered.end(), [pointToSegments](const std::pair<std::shared_ptr<Vertex>, double> p1,
                                                             const std::pair<std::shared_ptr<Vertex>, double> p2){
                    bool firstOnHole = p1.first->searchFlag(FlagType::ON_HOLE) >= 0;
                    bool secondOnHole = p2.first->searchFlag(FlagType::ON_HOLE) >= 0;
                    bool different = firstOnHole != secondOnHole;

                    return (different && firstOnHole) ||
                           (!different && p1.second > p2.second) ||
                           ((p1.second == 0.0 && p2.second == 0.0) &&
                            (pointToSegments.at(p1.first).size() < pointToSegments.at(p2.first).size()));

                });

                v = ordered.begin()->first;


            }
            v->addFlag(FlagType::USED);



        } while( v != begin );

        if(innerBoundary.begin()->get() != innerBoundary.back().get())
            innerBoundary.push_back(*innerBoundary.begin());

        for_each(innerBoundary.begin(), innerBoundary.end(), [](std::shared_ptr<Vertex> v){
            v->removeFlag(FlagType::ON_HOLE);
        });
        overallBasePolygons.push_back(innerBoundary);
    }

    for(auto &building : buildings)
    {
        auto boundaries = building->getOutlines();
        if(boundaries.size() > 1)
            overallBasePolygons.insert(overallBasePolygons.end(), boundaries.begin() + 1, boundaries.end());
        for(auto& b : boundaries.at(0))
        {
            b->removeFlag(FlagType::OUTSIDE);
            b->removeFlag(FlagType::INSIDE);
            b->removeFlag(FlagType::ON_HOLE);
        }
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
