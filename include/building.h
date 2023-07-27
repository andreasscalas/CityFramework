#ifndef BUILDING_H
#define BUILDING_H

#include <vector>
#include <memory>
#include <string>
#include <Point.hpp>
#include <Vertex.hpp>
#include <surfaceannotation.hpp>

typedef std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >  PointList;
typedef std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> >  VertexList;

/**
 * @brief The Building class is used to manage the buildings of the cities.
 * WARNING: TO BE COMPLETED, CURRENTLY ONLY MANAGES SINGLE OUTER BOUNDARIES
 */
class Building : public SemantisedTriangleMesh::SurfaceAnnotation
{
public:
    Building();
    void addOuterBoundary(const VertexList boundary);
    void addInnerBoundary(const VertexList boundary);

    std::vector<std::shared_ptr<Building> > getAdjacentBuildings() const;
    void setAdjacentBuildings(const std::vector<std::shared_ptr<Building> > &value);
    void addAdjacentBuilding(const std::shared_ptr<Building> building);
    bool removeAdjacentBuilding(unsigned int pos);
    double computeSurfaceAreaToVolumeRatio();

    void fixBoundaries();

    static VertexList mergeBoundaries(VertexList first, VertexList second);
    std::vector<VertexList> mergeWithAdjacent(std::string adjID);
    std::vector<VertexList> mergeWithAllAdjacents();

protected:
    std::vector<std::shared_ptr<Building> > adjacentBuildings;

};

#endif // BUILDING_H
