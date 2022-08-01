#ifndef EDGE_H
#define EDGE_H

#include <Vertex.h>
#include <Triangle.h>
#include <CommonDefinitions.h>
#include <vector>
#include <memory>

class Vertex;
class Triangle;

class Edge
{
public:
    Edge();
    Edge(std::shared_ptr<Vertex>, std::shared_ptr<Vertex>);
    Edge(std::shared_ptr<Edge>);
    ~Edge();

    std::shared_ptr<Vertex>  getCommonVertex(std::shared_ptr<Edge> e2);
    std::shared_ptr<Vertex>  getOppositeVertex(std::shared_ptr<Vertex>  v);
    std::shared_ptr<Triangle> getLeftTriangle(std::shared_ptr<Vertex> );
    std::shared_ptr<Triangle> getRightTriangle(std::shared_ptr<Vertex> v);
    std::shared_ptr<Triangle> getOppositeTriangle(std::shared_ptr<Triangle> v);
    bool hasVertex(std::shared_ptr<Vertex> );
    bool hasVertex(Vertex* );

    std::shared_ptr<Vertex> getV1() const;
    void setV1(std::shared_ptr<Vertex> newV1);
    std::shared_ptr<Vertex> getV2() const;
    void setV2(std::shared_ptr<Vertex> newV2);
    bool setVertex(std::shared_ptr<Vertex>,std::shared_ptr<Vertex>);

    std::shared_ptr<Triangle> getT1() const;
    void setT1(std::shared_ptr<Triangle> newT1);
    std::shared_ptr<Triangle> getT2() const;
    void setT2(std::shared_ptr<Triangle> newT2);
    bool setTriangle(std::shared_ptr<Triangle> t, std::shared_ptr<Triangle> newT);

    const std::vector<void *> &getInformation() const;
    void setInformation(const std::vector<void *> &newInformation);
    const std::vector<FlagType> &getAssociatedFlags() const;
    void setAssociatedFlags(const std::vector<FlagType> &newAssociatedFlags);

    bool addFlag(FlagType);
    int searchFlag(FlagType);
    bool removeFlag(FlagType);
    bool removeFlag(unsigned int);
    bool addInformation(void*);
    int searchInfo(void*);
    bool removeInfo(void*);
    bool removeInfo(unsigned int);
    bool clearInfo();
    std::string getId() const;
    void setId(std::string newId);

    double computeLength() const;
    void print(std::ostream&);

protected:
    std::string id;
    std::shared_ptr<Vertex>  v1;
    std::shared_ptr<Vertex>  v2;
    std::shared_ptr<Triangle> t1;
    std::shared_ptr<Triangle> t2;
    std::vector<void*> information;
    std::vector<FlagType> associated_flags;
};

#endif // EDGE_H
