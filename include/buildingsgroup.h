#ifndef BUILDINGSGROUP_H
#define BUILDINGSGROUP_H

#include <vector>
#include <memory>
#include "building.h"
#include "graph.h"

class BuildingsGroup
{
public:
    BuildingsGroup();
    ~BuildingsGroup();
    std::vector<std::shared_ptr<Building> > getBuildings() const;
    void setBuildings(const std::vector<std::shared_ptr<Building> > &value);
    void addBuilding(const std::shared_ptr<Building> &value);

    void computeAdjacencyGraph();
    std::vector<VertexList> extractOverallBasePolygon();
    std::string getId() const;
    void setId(const std::string &value);

protected:
    std::string id;
    std::vector<std::shared_ptr<Building> > buildings;
    GraphTemplate::Graph<std::shared_ptr<Building> >* adjacencyGraph;
    std::vector<VertexList> overallBasePolygon;


};

#endif // BUILDINGSGROUP_H
