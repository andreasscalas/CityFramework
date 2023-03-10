#ifndef BUILDING_H
#define BUILDING_H

#include <vector>
#include <memory>
#include <string>
#include "Point.hpp"
#include "Vertex.hpp"

typedef std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >  PointList;
typedef std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> >  VertexList;

/**
 * @brief The Building class is used to manage the buildings of the cities.
 * WARNING: TO BE COMPLETED, CURRENTLY ONLY MANAGES SINGLE OUTER BOUNDARIES
 */
class Building
{
public:
    Building();

    std::string getId() const;
    void setId(const std::string &value);

    std::vector<VertexList> getBoundaries() const;
    void setBoundaries(const std::vector<VertexList> &value);
    void addOuterBoundary(const VertexList boundary);
    void addInnerBoundary(const VertexList boundary);
    bool removeBoundary(unsigned int pos);

    std::vector<std::shared_ptr<Building> > getAdjacentBuildings() const;
    void setAdjacentBuildings(const std::vector<std::shared_ptr<Building> > &value);
    void addAdjacentBuilding(const std::shared_ptr<Building> building);
    bool removeAdjacentBuilding(unsigned int pos);

    void fixBoundaries();

    static VertexList mergeBoundaries(VertexList first, VertexList second);
    std::vector<VertexList> mergeWithAdjacent(std::string adjID);
    std::vector<VertexList> mergeWithAllAdjacents();

protected:
    std::string id;
    std::vector<VertexList> boundaries;
    std::vector<std::shared_ptr<Building> > adjacentBuildings;

};

#endif // BUILDING_H
