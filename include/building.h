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
    enum class FlagType
    {
        USED
    };
    Building();
    void addOuterBoundary(const VertexList boundary);
    bool setOutline(uint pos, const VertexList boundary);
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

    std::vector<FlagType> getAssociatedFlags() const;
    void setAssociatedFlags(const std::vector<FlagType> &newAssociatedFlags);
    /**
         * @brief Add a flag to the associated flags of the vertex.
         * @param flag The flag to add to the associated flags.
         * @return True if the flag was successfully added, false otherwise.
         */
    bool addFlag(FlagType flag);

    /**
         * @brief Search for a specific flag in the associated flags of the vertex.
         * @param flag The flag to search for.
         * @return The index of the found flag if it exists, or -1 if the flag is not found.
         */
    int searchFlag(FlagType flag);

    /**
         * @brief Remove a flag from the associated flags of the vertex.
         * @param flag The flag to remove.
         * @return True if the flag was successfully removed, false otherwise.
         */
    bool removeFlag(FlagType flag);

    /**
         * @brief Remove a flag at the specified index from the associated flags of the vertex.
         * @param index The index of the flag to remove.
         * @return True if the flag at the specified index was successfully removed, false otherwise.
         */
    bool removeFlag(unsigned int index);

    bool operator==(const Building& other) const;
    bool operator!=(const Building& other) const;
protected:
    std::vector<std::shared_ptr<Building> > adjacentBuildings;
    std::vector<FlagType> associatedFlags;

};

#endif // BUILDING_H
