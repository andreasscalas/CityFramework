#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include <Vertex.h>
#include <Edge.h>
#include <Triangle.h>

class TriangleMesh
{
public:
    TriangleMesh();
protected:
    Vertex** vertices;
    Edge** edges;
    Triangle** triangles;
};

#endif // TRIANGLEMESH_H
