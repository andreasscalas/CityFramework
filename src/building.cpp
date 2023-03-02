#include "building.h"

#include <map>
#include <algorithm>

using namespace SemantisedTriangleMesh;
typedef unsigned int uint;

Building::Building()
{

}

std::vector<VertexList> Building::getBoundaries() const
{
    return boundaries;
}

void Building::setBoundaries(const std::vector<VertexList> &value)
{
    boundaries = value;
}

void Building::addOuterBoundary(const VertexList boundary)
{
    boundaries.insert(boundaries.begin(), boundary);
}

void Building::addInnerBoundary(const VertexList boundary)
{
    boundaries.push_back(boundary);
}

bool Building::removeBoundary(uint pos)
{
    if(pos >= boundaries.size())
        return false;
    boundaries.erase(boundaries.begin() + pos);
    return true;
}

std::vector<std::shared_ptr<Building> > Building::getAdjacentBuildings() const
{
    return adjacentBuildings;
}

void Building::setAdjacentBuildings(const std::vector<std::shared_ptr<Building> > &value)
{
    adjacentBuildings = value;
}

void Building::addAdjacentBuilding(const std::shared_ptr<Building> building)
{
    adjacentBuildings.push_back(building);
}

bool Building::removeAdjacentBuilding(uint pos)
{
    if(pos >= adjacentBuildings.size())
        return false;
    adjacentBuildings.erase(adjacentBuildings.begin() + pos);
    return true;

}

void Building::fixBoundaries()
{

    for(auto iit = boundaries.begin(); iit != boundaries.end(); iit++)
    {
        auto boundary = *iit;
        for(auto jit = boundary.begin(); jit != boundary.begin() + boundary.size() - 2; jit++)
        {
            while(boundary.size() > 2 && (**jit) == (**(jit + 1)))
            {
                auto tmp = *(jit + 1);
                boundary.erase(jit + 1);
                tmp.reset();
            }

            auto end = boundary.begin() + boundary.size() - 1;
            auto it = std::find_if(jit + 1, end, [&] (std::shared_ptr<Vertex> p){ return *p == **jit; });
            while(it != end)
            {
                for(std::vector<std::shared_ptr<Vertex> >::iterator it1 = jit + 1; it1 <= it; it1++)
                    it1->reset();
                boundary.erase(jit + 1, it + 1);
                it = std::find_if(jit + 1, end, [&] (std::shared_ptr<Vertex> p){ return *p == **jit; });
            }

            if(jit != boundary.begin())
                if((**jit) == (**boundary.begin()))
                    boundary.erase(jit, boundary.end());
        }
        if(boundary.size() < 4)
        {
            iit = boundaries.erase(iit);
            iit--;
        }

    }
}

VertexList Building::mergeBoundaries(VertexList first, VertexList second)
{
    VertexList newBoundary;
    std::reverse(second.begin(), second.end());
    std::vector<std::pair<uint, uint> > toCollapse;

    size_t i, j = second.size();
    for(i = 0; i < first.size(); i++)
    {
        auto it2 = std::find_if(second.begin(), second.end(), [&](std::shared_ptr<Point> p){ return *first.at(i) == *p; });
        if(it2 != second.end())
        {
            j = it2 - second.begin();
            break;
        }
    }

    if(i != first.size() && j != second.size())
    {
        size_t i_prev = i;
        size_t i_next = (i + 1) % (first.size() - 1);
        size_t j_prev = j;
        size_t j_next = (j + 1) % (second.size() - 1);
        while(*first.at(i_prev) == *second.at(j_prev) && *first.at(i_next) == *second.at(j_next))
        {
            i_prev = i_next;
            j_prev = j_next;
            i_next = (i_next + 1) % (first.size() - 1);
            j_next = (j_next + 1) % (second.size() - 1);
        }

        if(j_prev > j)
        {
            newBoundary.insert(newBoundary.end(), second.begin() + j_prev, second.end() - 1);
            newBoundary.insert(newBoundary.end(), second.begin(), second.begin() + j + 1);
        } else
            newBoundary.insert(newBoundary.end(), second.begin() + j_prev, second.begin() + j + 1);
        std::reverse(newBoundary.begin(), newBoundary.end());
        if(i_prev > i)
        {
            newBoundary.insert(newBoundary.end(), first.begin() + i_prev + 1, first.end() - 1);
            newBoundary.insert(newBoundary.end(), first.begin(), first.begin() + i);
        } else
            newBoundary.insert(newBoundary.end(), second.begin() + i_prev + 1, second.begin() + i);
        newBoundary.push_back(newBoundary.front());
    }
    return newBoundary;
}

std::vector<VertexList> Building::mergeWithAdjacent(std::string adjID)
{
    std::vector<VertexList> newBoundaries;
    auto adjIt = std::find_if(adjacentBuildings.begin(), adjacentBuildings.end(),
                              [adjID](std::shared_ptr<Building> b) { return b->getId().compare(adjID) == 0;});
    if(adjIt != adjacentBuildings.end())
    {
        VertexList newBoundary = mergeBoundaries(boundaries.at(0), (*adjIt)->getBoundaries().at(0));
        if(newBoundary.size() != 0)
        {
            newBoundaries.push_back(newBoundary);
            newBoundaries.insert(newBoundaries.end(), boundaries.begin() + 1, boundaries.end());
        }

    }

    return newBoundaries;

}

std::vector<VertexList> Building::mergeWithAllAdjacents()
{
    std::vector<VertexList> newBoundaries = boundaries;
    for(auto b : adjacentBuildings)
    {
        VertexList newBoundary = mergeBoundaries(newBoundaries.at(0), b->getBoundaries().at(0));
        if(newBoundary.size() != 0)
        {
            newBoundaries.at(0) = newBoundary;
            for(auto p = b->getBoundaries().begin() + 1; p != b->getBoundaries().end(); p++)
                newBoundaries.push_back(*p);
        }
    }

    return newBoundaries;

}

std::string Building::getId() const
{
    return id;
}

void Building::setId(const std::string &value)
{
    id = value;
}
