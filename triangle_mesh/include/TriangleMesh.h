#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include <Vertex.h>
#include <Edge.h>
#include <Triangle.h>
#include <annotation.h>
#include <memory>

class Annotation;
class TriangleMesh
{
public:
    TriangleMesh();
    TriangleMesh(std::shared_ptr<TriangleMesh>);
    ~TriangleMesh();
    unsigned int getVerticesNumber();
    unsigned int getEdgesNumber();
    unsigned int getTrianglesNumber();
    std::shared_ptr<Vertex> addNewVertex();
    std::shared_ptr<Vertex> addNewVertex(double, double, double);
    std::shared_ptr<Vertex> addNewVertex(Point);
    std::shared_ptr<Vertex> addNewVertex(std::shared_ptr<Vertex> );
    std::shared_ptr<Vertex> getVertex(unsigned int);
    std::shared_ptr<Vertex> getVertex(std::string);
    std::shared_ptr<Edge> getEdge(unsigned int);
    std::shared_ptr<Edge> getEdge(std::string);
    std::shared_ptr<Triangle> getTriangle(unsigned int);
    std::shared_ptr<Triangle> getTriangle(std::string);
    std::shared_ptr<Edge> addNewEdge();
    std::shared_ptr<Edge> addNewEdge(std::shared_ptr<Vertex>, std::shared_ptr<Vertex>);
    std::shared_ptr<Edge> addNewEdge(std::shared_ptr<Edge>);
    std::shared_ptr<Triangle> addNewTriangle();
    std::shared_ptr<Triangle> addNewTriangle(std::shared_ptr<Edge>,std::shared_ptr<Edge>, std::shared_ptr<Edge>);
    std::shared_ptr<Triangle> addNewTriangle(std::shared_ptr<Triangle>);

    bool removeTriangle(std::string tid);
    void removeFlaggedTriangles();
    int load(std::string filename);
    int save(std::string filename, unsigned int precision = 5);
    unsigned int removeIsolatedVertices();
    int triangulate(std::vector<std::vector<std::shared_ptr<Point> > > &boundaries, std::vector<std::vector<std::shared_ptr<Point> > > &constraints);
    int triangulate(std::vector<std::vector<std::vector<std::shared_ptr<Point> > > > &boundaries, std::vector<std::vector<std::shared_ptr<Point> > > &constraints);
    int triangulate(std::vector<std::vector<std::vector<std::shared_ptr<Vertex> > > > &boundaries, std::vector<std::vector<std::shared_ptr<Vertex> > > &constraints, std::vector<std::shared_ptr<Vertex> > constraintVertices);
    std::shared_ptr<Edge> searchEdgeContainingVertex(std::vector<std::shared_ptr<Edge> > list, std::shared_ptr<Vertex> v);

    const std::vector<std::shared_ptr<Annotation> > &getAnnotations() const;
    void clearAnnotations();
    void setAnnotations(const std::vector<std::shared_ptr<Annotation> > &newAnnotations);
    bool addAnnotation(const std::shared_ptr<Annotation> &newAnnotation);
    std::shared_ptr<Annotation> getAnnotation(unsigned int);
    bool removeAnnotation(unsigned int);

    void orientTrianglesCoherently();


protected:
    std::vector<std::shared_ptr<Vertex> > vertices;
    std::vector<std::shared_ptr<Edge> > edges;
    std::vector<std::shared_ptr<Triangle> > triangles;
    std::vector<std::shared_ptr<Annotation> > annotations;

    int loadPLY(std::string filename);
};


#endif // TRIANGLEMESH_H
