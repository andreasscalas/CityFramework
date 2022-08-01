#ifndef VERTEX_H
#define VERTEX_H

#include <Point.h>
#include <Edge.h>
#include <Triangle.h>
#include <CommonDefinitions.h>
#include <memory>
#include <vector>

class Edge;
class Triangle;


class Vertex : public Point
{
public:
    Vertex();
    Vertex(double x, double y, double z);
    Vertex(double* p);
    Vertex(Point p);
    Vertex(std::shared_ptr<Point> p);
    Vertex(std::shared_ptr<Vertex> v);
    ~Vertex();

    std::shared_ptr<Edge> getE0() const;
    void setE0(std::shared_ptr<Edge> newE0);
    const std::vector<void *> &getInformation() const;
    void setInformation(const std::vector<void *> &newInformation);
    const std::vector<FlagType> &getAssociatedFlags() const;
    void setAssociatedFlags(const std::vector<FlagType> &newAssociatedFlags);
    std::vector<std::shared_ptr<Vertex> > getVV();
    std::vector<std::shared_ptr<Edge> > getVE();
    std::vector<std::shared_ptr<Triangle> > getVT();
    std::shared_ptr<Edge> getCommonEdge(std::shared_ptr<Vertex> v);
    Point computeNormal();
    std::vector<std::shared_ptr<Vertex> > getNeighbourhood(uint size);

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
    virtual void print(std::ostream&);

protected:
    std::string id;
    std::shared_ptr<Edge> e0;
    std::vector<void*> information;
    std::vector<FlagType> associated_flags;
};

#endif // VERTEX_H
