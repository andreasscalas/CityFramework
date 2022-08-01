#include "TriangleMesh.h"
#include <fstream>
#include <sstream>
#include <map>
#include <trianglehelper1.h>
#include <exception>
#include <surfaceannotation.h>
#include <pointannotation.h>
#include <lineannotation.h>

TriangleMesh::TriangleMesh()
{

}

TriangleMesh::TriangleMesh(const std::shared_ptr<TriangleMesh> other)
{

    for(uint i = 0; i < other->getVerticesNumber(); i++)
    {
        auto v = other->getVertex(i);
        auto newV = std::make_shared<Vertex>(v->getX(), v->getY(), v->getZ());
        newV->setId(v->getId());
        newV->setInfo(v->getInfo());
        newV->setAssociatedFlags(v->getAssociatedFlags());
        this->vertices.push_back(newV);
    }

    for(uint i = 0; i < other->getEdgesNumber(); i++)
    {
        auto e = other->getEdge(i);
        auto newE = std::make_shared<Edge>();
        newE->setId(e->getId());
        newE->setV1(this->getVertex(e->getV1()->getId()));
        newE->setV2(this->getVertex(e->getV2()->getId()));
        newE->setAssociatedFlags(e->getAssociatedFlags());
        newE->setInformation(e->getInformation());
        this->edges.push_back(newE);
    }

    for(uint i = 0; i < other->getVerticesNumber(); i++)
    {
        auto v = other->getVertex(i);
        auto newV = this->getVertex(i);
        newV->setE0(this->getEdge(v->getE0()->getId()));
    }

    for(uint i = 0; i < other->getTrianglesNumber(); i++)
    {
        auto t = other->getTriangle(i);
        auto newT = std::make_shared<Triangle>();
        newT->setId(t->getId());
        newT->setE1(this->getEdge(t->getE1()->getId()));
        newT->setE2(this->getEdge(t->getE2()->getId()));
        newT->setE3(this->getEdge(t->getE3()->getId()));
        newT->setAssociatedFlags(t->getAssociatedFlags());
        newT->setInformation(t->getInformation());
        this->triangles.push_back(newT);
    }
    for(uint i = 0; i < other->getEdgesNumber(); i++)
    {
        auto e = other->getEdge(i);
        auto newE = getEdge(i);
        if(e->getT1() == nullptr)
            newE->setT1(nullptr);
        else
            newE->setT1(this->getTriangle(e->getT1()->getId()));
        if(e->getT2() == nullptr)
            newE->setT2(nullptr);
        else
            newE->setT2(this->getTriangle(e->getT2()->getId()));
    }

    for(uint i = 0; i < other->getAnnotations().size(); i++)
    {
        switch(other->getAnnotation(i)->getType())
        {
            case AnnotationType::Point:
            {
                auto a = std::dynamic_pointer_cast<PointAnnotation>(other->getAnnotation(i));
                annotations.push_back(std::make_shared<PointAnnotation>(this, a));
                break;
            }
            case AnnotationType::Line:
            {
                auto a = std::dynamic_pointer_cast<LineAnnotation>(other->getAnnotation(i));
                annotations.push_back(std::make_shared<LineAnnotation>(this, a));
                break;
            }
            case AnnotationType::Surface:
            {
                auto a = std::dynamic_pointer_cast<SurfaceAnnotation>(other->getAnnotation(i));
                annotations.push_back(std::make_shared<SurfaceAnnotation>(this, a));
                break;
            }
            default :
            {
                std::cout << "This kind of annotation is still to be implemented" << std::endl;
                exit(44);
            }
        }

    }

}

TriangleMesh::~TriangleMesh()
{

    for(unsigned int i = 0; i < vertices.size(); i++)
        if(vertices.at(i) != nullptr)
            vertices.at(i).reset();
    for(unsigned int i = 0; i < edges.size(); i++)
        if(edges.at(i) != nullptr)
            edges.at(i).reset();
    for(unsigned int i = 0; i < triangles.size(); i++)
        if(triangles.at(i) != nullptr)
            triangles.at(i).reset();
}

unsigned int TriangleMesh::getVerticesNumber()
{
    return vertices.size();
}

unsigned int TriangleMesh::getEdgesNumber()
{
    return edges.size();
}

unsigned int TriangleMesh::getTrianglesNumber()
{
    return triangles.size();
}

std::shared_ptr<Vertex> TriangleMesh::addNewVertex()
{
    vertices.push_back(std::make_shared<Vertex>());
    return vertices.back();
}

std::shared_ptr<Vertex> TriangleMesh::addNewVertex(double x, double y, double z)
{
    vertices.push_back(std::make_shared<Vertex>(x, y, z));
    return vertices.back();
}

std::shared_ptr<Vertex> TriangleMesh::addNewVertex(Point p)
{
    std::shared_ptr<Vertex> v = addNewVertex(p.getX(), p.getY(), p.getZ());
    v->setInfo(p.getInfo());
    return v;
}

std::shared_ptr<Vertex> TriangleMesh::addNewVertex(std::shared_ptr<Vertex> v)
{
    auto it = std::find_if(vertices.begin(), vertices.end(), [v](std::shared_ptr<Vertex> v1){ auto p1 = *v; auto p2 = *v1; p1.setZ(0); p2.setZ(0); return p1 == p2;});
    if(it != vertices.end()) //La gestione delle strade che intersecano boundary interni dati da unione di edifici è lasciata da parte
    {
        return *it;
    }
    vertices.push_back(v);
    return vertices.back();
}

std::shared_ptr<Vertex> TriangleMesh::getVertex(unsigned int pos)
{
    if(pos >= vertices.size())
        return nullptr;
    return vertices.at(pos);
}

std::shared_ptr<Vertex> TriangleMesh::getVertex(std::string id)
{
    return getVertex(stoi(id));
}

std::shared_ptr<Edge> TriangleMesh::getEdge(unsigned int pos)
{
    if(pos >= edges.size())
        return nullptr;
    return edges.at(pos);
}

std::shared_ptr<Edge> TriangleMesh::getEdge(std::string id)
{
    return getEdge(stoi(id));
}

std::shared_ptr<Triangle> TriangleMesh::getTriangle(unsigned int pos)
{
    if(pos >= triangles.size())
        return nullptr;
    return triangles.at(pos);
}

std::shared_ptr<Triangle> TriangleMesh::getTriangle(std::string id)
{
    return getTriangle(stoi(id));
}

std::shared_ptr<Edge> TriangleMesh::addNewEdge()
{
    edges.push_back(std::make_shared<Edge>());
    return edges.back();
}

std::shared_ptr<Edge> TriangleMesh::addNewEdge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    edges.push_back(std::make_shared<Edge>(v1, v2));
    return edges.back();
}

std::shared_ptr<Edge> TriangleMesh::addNewEdge(std::shared_ptr<Edge> e)
{
    edges.push_back(std::make_shared<Edge>(e));
    return edges.back();
}

std::shared_ptr<Triangle> TriangleMesh::addNewTriangle()
{
    triangles.push_back(std::make_shared<Triangle>());
    return triangles.back();
}

std::shared_ptr<Triangle> TriangleMesh::addNewTriangle(std::shared_ptr<Edge> e1, std::shared_ptr<Edge> e2, std::shared_ptr<Edge> e3)
{
    triangles.push_back(std::make_shared<Triangle>(e1, e2, e3));
    return triangles.back();
}

std::shared_ptr<Triangle> TriangleMesh::addNewTriangle(std::shared_ptr<Triangle> t)
{
    triangles.push_back(std::make_shared<Triangle>(t));
    return triangles.back();
}

bool TriangleMesh::removeTriangle(std::string tid)
{
//    uint pos = stoi(tid);
//    if(pos > triangles.size())
//        return false;
//    auto it = triangles.erase(triangles.begin() + pos);
//    return true;
    for(auto tit = triangles.begin(); tit != triangles.end(); tit++)
        if((*tit)->getId().compare(tid) == 0)
        {
            tit = triangles.erase(tit);
            return true;
        }
    return false;
}

void TriangleMesh::removeFlaggedTriangles()
{
    for(auto it = triangles.begin(); it != triangles.end(); it++)
        if((*it)->searchFlag(FlagType::TO_BE_REMOVED) >= 0)
        {
            if((*it)->getE1()->setTriangle((*it), nullptr))
                std::cout << "";
            if((*it)->getE2()->setTriangle((*it), nullptr))
                std::cout << "";
            if((*it)->getE3()->setTriangle((*it), nullptr))
                std::cout << "";
            it = triangles.erase(it);
            it--;
        }
}


int TriangleMesh::load(std::string filename)
{
    return loadPLY(filename);
}

int TriangleMesh::save(std::string filename, unsigned int precision)
{

    std::cout << "Saving mesh on " << filename  << std::endl;
    int return_code;
    std::ofstream meshStream(filename);
    std::string format = filename.substr(filename.find(".") + 1);
    meshStream.precision(precision);
    if(meshStream.is_open())
    {
        if(format.compare("ply") == 0)
        {
            meshStream << "ply" << std::endl;
            meshStream << "format ascii 1.0" << std::endl;
            meshStream << "element vertex " << getVerticesNumber() << std::endl;
            meshStream << "property double x" << std::endl;
            meshStream << "property double y" << std::endl;
            meshStream << "property double z" << std::endl;
            meshStream << "element face " << getTrianglesNumber() << std::endl;
            meshStream << "property list uchar int vertex_indices" << std::endl;
            meshStream << "end_header" << std::endl;
            std::cout << "Writing vertices:" << std::endl;
            for(unsigned int i = 0; i < getVerticesNumber(); i++)
            {
                if(vertices.at(i)->getZ() > 5000)
                    vertices.at(i)->print(std::cout );
                meshStream << vertices.at(i)->getX() << " " << vertices.at(i)->getY() << " " << vertices.at(i)->getZ() << std::endl;
                if(i % 100 == 0)
                    std::cout << i * 100 / getVerticesNumber() << "%\r" << std::flush;
            }

            std::cout << "Ended! Written " << getVerticesNumber() << " vertices." << std::endl << "Writing triangles:" << std::endl;
            for(unsigned int i = 0; i < getTrianglesNumber(); i++)
            {
                auto v1 = triangles.at(i)->getV1();
                auto v2 = triangles.at(i)->getV2();
                auto v3 = triangles.at(i)->getV3();
                if(Point::orientation(*v1, *v2, *v3) < 0)
                {
                    meshStream << "3 " << v1->getId() << " " << v2->getId() << " " << v3->getId() << std::endl;
                } else
                    meshStream << "3 " << v1->getId() << " " << v3->getId() << " " << v2->getId() << std::endl;

                if(i % 100 == 0)
                    std::cout << i * 100 / getTrianglesNumber() << "%\r" << std::flush;
            }
            std::cout << "Ended! Written " << getTrianglesNumber() << " triangles." << std::endl;
            return_code = 0;
        } else
            return_code = 1;

        meshStream.close();
    } else
        return_code = -1;
    return return_code;
}

unsigned int TriangleMesh::removeIsolatedVertices()
{
    unsigned int removed = 0;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        if(vertices.at(i)->getE0() == nullptr)
        {
            //std::cout << "Removing vertex with id: " << vertices.at(i)->getId() << std::endl;
            vertices.erase(vertices.begin() + i);
            removed++;
            i--;
        }
    }
    return removed;
}

std::shared_ptr<Edge> TriangleMesh::searchEdgeContainingVertex(std::vector<std::shared_ptr<Edge> > list, std::shared_ptr<Vertex> v)
{
    for(unsigned int i = 0; i < list.size(); i++)
        if(list.at(i)->hasVertex(v))
            return list.at(i);

    return nullptr;
}

const std::vector<std::shared_ptr<Annotation> > &TriangleMesh::getAnnotations() const
{
    return annotations;
}

void TriangleMesh::clearAnnotations()
{
    annotations.clear();
}

void TriangleMesh::setAnnotations(const std::vector<std::shared_ptr<Annotation> > &newAnnotations)
{
    annotations = newAnnotations;
}

bool TriangleMesh::addAnnotation(const std::shared_ptr<Annotation> &newAnnotation)
{
    annotations.push_back(newAnnotation);
    return true;
}

std::shared_ptr<Annotation> TriangleMesh::getAnnotation(unsigned int id)
{
    if(id < 0 || id >= annotations.size())
        return nullptr;
    return annotations.at(id);
}

bool TriangleMesh::removeAnnotation(unsigned int id)
{
    annotations.erase(annotations.begin() + id);
    return true;
}

int TriangleMesh::triangulate(std::vector<std::vector<std::vector<std::shared_ptr<Point> > > > &boundaries, std::vector<std::vector<std::shared_ptr<Point> > > &constraints)
{
    if(boundaries.size() < 1)
        return -1;

    std::vector<double*> points;
    std::vector<std::vector<unsigned int> > polylines;
    std::map<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Edge> > > vertices_edges;
    unsigned int vertices_id = vertices.size();
    unsigned int edges_id = edges.size();
    unsigned int triangles_id = triangles.size();
    /**The first boundary is the exterior boundary**/

    std::vector<unsigned int> exterior_boundary_polyline;

    for(unsigned int i = 0; i < boundaries.at(0).at(0).size(); i++)
    {
        std::vector<std::shared_ptr<Edge> > incident;
        std::shared_ptr<Point> tmp = boundaries.at(0).at(0).at(i);
        boundaries.at(0).at(0).at(i) = addNewVertex(*tmp);
        points.push_back(boundaries.at(0).at(0).at(i)->toDoubleArray());
        tmp.reset();
        exterior_boundary_polyline.push_back(vertices_id);
        vertices.back()->setId(std::to_string(vertices_id++));
        vertices_edges.insert(std::make_pair(vertices.back(), incident));
    }
    exterior_boundary_polyline.push_back(0);
    polylines.push_back(exterior_boundary_polyline);

    /**The remaining boundaries are holes**/
    std::vector<double*> holes_seeds;


    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::shared_ptr<Point> > outline = boundaries.at(i).at(0);

        double v[2] = {(outline[1]->getX() - outline[0]->getX()) * 1E-2, (outline[1]->getY() - outline[0]->getY()) * 1E-2};
        double vec[2] = {-v[1], v[0]};
        double middle[2] = {(outline[1]->getX() + outline[0]->getX()) / 2, (outline[1]->getY() + outline[0]->getY()) / 2};
        double* innerpoint = new double(2);
        innerpoint[0] = vec[0] + middle[0];
        innerpoint[1] = vec[1] + middle[1];

        holes_seeds.push_back(innerpoint);

        for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
        {
            unsigned int counter = 0;
            std::vector<unsigned int> polyline;
            for(unsigned int k = 0; k < boundaries.at(i).at(j).size() - 1; k++)
            {
                std::vector<std::shared_ptr<Vertex> >::iterator it = std::find_if(vertices.begin(), vertices.end(),
                          [boundaries, i, j, k](std::shared_ptr<Vertex> p)
                {
                    return *boundaries.at(i).at(j).at(k) == *p;
                });

                if(it != vertices.end())
                {
                    counter++;
                    std::shared_ptr<Point> tmp = boundaries.at(i).at(j).at(k);
                    boundaries.at(i).at(j).at(k) = *it;
                    tmp.reset();
                }

                if(it == vertices.end())
                {
                    std::vector<std::shared_ptr<Edge> > incident;
                    std::shared_ptr<Point> tmp = boundaries.at(i).at(j).at(k);
                    std::shared_ptr<Vertex> v = addNewVertex(*tmp);
                    points.push_back(v->toDoubleArray());
                    boundaries.at(i).at(j).at(k) = v;
                    tmp.reset();
                    v->setId(std::to_string(vertices_id++));
                    vertices_edges.insert(std::make_pair(v, incident));
                } else if(k > 0 && *boundaries.at(i).at(j).at(k) == *boundaries.at(i).at(j).at(k - 1))
                {
                    boundaries.at(i).at(j).erase(boundaries.at(i).at(j).begin() + k);
                    k--;
                }

                polyline.push_back(static_cast<unsigned int>(std::stoi(std::static_pointer_cast<Vertex>(boundaries.at(i).at(j).at(k))->getId())));
            }

            std::shared_ptr<Point> tmp = boundaries.at(i).at(j).back();
            boundaries.at(i).at(j).back() = boundaries.at(i).at(j).at(0);
            tmp.reset();
            polyline.push_back(static_cast<unsigned int>(std::stoi(std::static_pointer_cast<Vertex>(boundaries.at(i).at(j).at(0))->getId())));
            polylines.push_back(polyline);

        }
    }

    for(unsigned int i = 0; i < constraints.size(); i++)
    {
        std::vector<unsigned int> polyline;
        for(unsigned int j = 0; j < constraints.at(i).size(); j++)
        {
            std::vector<std::shared_ptr<Vertex> >::iterator it = std::find_if(vertices.begin(), vertices.end(),
                      [constraints, i, j](std::shared_ptr<Vertex> p)
            {
                return *constraints.at(i).at(j) == *p;
            });

            if(it != vertices.end())
            {
                std::shared_ptr<Point> tmp = constraints.at(i).at(j);
                constraints.at(i).at(j) = *it;
                tmp.reset();
            }

            if(it == vertices.end())
            {
                std::vector<std::shared_ptr<Edge> > incident;
                std::shared_ptr<Point> tmp = constraints.at(i).at(j);
                std::shared_ptr<Vertex> v = addNewVertex(*tmp);
                points.push_back(v->toDoubleArray());
                constraints.at(i).at(j) = v;
                tmp.reset();
                v->setId(std::to_string(vertices_id++));
                vertices_edges.insert(std::make_pair(v, incident));
            } else if(j > 0 && *constraints.at(i).at(j) == *constraints.at(i).at(j - 1))
            {
               constraints.at(i).erase(constraints.at(i).begin() + j);
               j--;
               continue;
            }
            polyline.push_back(static_cast<unsigned int>(std::stoi(std::static_pointer_cast<Vertex>(constraints.at(i).at(j))->getId())));
        }
        polylines.push_back(polyline);
    }

    std::cout << "Starting triangulation " << std::endl;
    TriHelper::TriangleHelper helper(points, polylines, holes_seeds, true);


    std::vector<unsigned int> generated_triangles = helper.getTriangles();
    std::vector<double*> generated_points = helper.getAddedPoints();
    for(unsigned int i = 0; i < generated_points.size(); i++)
    {
        std::vector<std::shared_ptr<Edge> > incident;
        std::shared_ptr<Vertex> v = addNewVertex(generated_points.at(i)[0], generated_points.at(i)[1], 0);
        v->setId(std::to_string(vertices_id++));
        vertices_edges.insert(std::make_pair(vertices.back(), incident));
    }

    std::cout << "Ended! Created " << generated_triangles.size() / 3 << " triangles." << std::endl << "Building mesh structure:" << std::endl;
    for(unsigned int i = 0; i < generated_triangles.size() / 3; i++)
    {
        unsigned int v1 = generated_triangles.at(i * 3), v2 = generated_triangles.at(i * 3 + 1), v3 = generated_triangles.at(i * 3 + 2);
        std::shared_ptr<Edge> e1, e2, e3;

        e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v2));
        if(e1 == nullptr)
            e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v1));
        if(e1 == nullptr)
        {
            addNewEdge(vertices.at(v1), vertices.at(v2));
            e1 = edges.back();
            e1->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v1)).push_back(e1);
        }
        vertices.at(v1)->setE0(e1);
        e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v3));
        if(e2 == nullptr)
            e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v2));
        if(e2 == nullptr)
        {
            addNewEdge(vertices.at(v2), vertices.at(v3));
            e2 = edges.back();
            e2->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v2)).push_back(e2);
        }
        vertices.at(v2)->setE0(e2);
        e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v1));
        if(e3 == nullptr)
            e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v3));
        if(e3 == nullptr)
        {
            addNewEdge(vertices.at(v3), vertices.at(v1));
            e3 = edges.back();
            e3->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v3)).push_back(e3);
        }
        vertices.at(v3)->setE0(e3);

        addNewTriangle(e1, e2, e3);
        triangles.back()->setId(std::to_string(triangles_id++));
        if(e1->getT1() == nullptr)
           e1->setT1(triangles.back());
        else if(e1->getT2() == nullptr)
            e1->setT2(triangles.back());
        else
            return 9;
        if(e2->getT1() == nullptr)
            e2->setT1(triangles.back());
        else if(e2->getT2() == nullptr)
            e2->setT2(triangles.back());
        else
            return 9;
        if(e3->getT1() == nullptr)
            e3->setT1(triangles.back());
        else if(e3->getT2() == nullptr)
            e3->setT2(triangles.back());
        else
            return 9;

        if(i % 100 == 0)
            std::cout << i * 100 / generated_triangles.size() << "%\r" << std::flush;;

    }

    std::cout << "Ended!" << std::endl;
    return 0;

}

int TriangleMesh::triangulate(std::vector<std::vector<std::vector<std::shared_ptr<Vertex> > > > &boundaries, std::vector<std::vector<std::shared_ptr<Vertex> > > &constraints, std::vector<std::shared_ptr<Vertex> > constraintVertices)
{
    if(boundaries.size() < 1)
        return -1;

    std::vector<double*> points;
    std::vector<std::vector<unsigned int> > polylines;
    std::map<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Edge> > > vertices_edges;
    unsigned int vertices_id = vertices.size();
    unsigned int edges_id = edges.size();
    unsigned int triangles_id = triangles.size();
    /**The first boundary is the exterior boundary**/

    std::vector<unsigned int> exterior_boundary_polyline;

    for(unsigned int i = 0; i < boundaries.at(0).at(0).size(); i++)
    {
        std::vector<std::shared_ptr<Edge> > incident;
        addNewVertex(boundaries.at(0).at(0).at(i));
        points.push_back(boundaries.at(0).at(0).at(i)->toDoubleArray());
        exterior_boundary_polyline.push_back(vertices_id);
        vertices.back()->setId(std::to_string(vertices_id++));
        vertices_edges.insert(std::make_pair(vertices.back(), incident));
    }
    exterior_boundary_polyline.push_back(0);
    polylines.push_back(exterior_boundary_polyline);

    /**The remaining boundaries are holes**/
    std::vector<double*> holes_seeds;


    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::shared_ptr<Vertex> > outline = boundaries.at(i).at(0);

        double v[2] = {(outline[1]->getX() - outline[0]->getX()) * 1E-2, (outline[1]->getY() - outline[0]->getY()) * 1E-2};
        double vec[2] = {-v[1], v[0]};
        double middle[2] = {(outline[1]->getX() + outline[0]->getX()) / 2, (outline[1]->getY() + outline[0]->getY()) / 2};
        double* innerpoint = new double(2);
        innerpoint[0] = vec[0] + middle[0];
        innerpoint[1] = vec[1] + middle[1];

        holes_seeds.push_back(innerpoint);

        for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
        {
            unsigned int counter = 0;
            std::vector<unsigned int> polyline;
            for(unsigned int k = 0; k < boundaries.at(i).at(j).size() - 1; k++)
            {
                std::vector<std::shared_ptr<Vertex> >::iterator it = std::find_if(vertices.begin(), vertices.end(),
                          [boundaries, i, j, k](std::shared_ptr<Vertex> p)
                {
                    auto p1 = *boundaries.at(i).at(j).at(k);
                    auto p2 = *p;
                    p1.setZ(0);
                    p2.setZ(0);

                    return  p1 == p2;
                });

                if(it != vertices.end())
                {
                    counter++;
                    *boundaries.at(i).at(j).at(k) = **it;
                }

                if(it == vertices.end())
                {
                    std::vector<std::shared_ptr<Edge> > incident;
                    addNewVertex(boundaries.at(i).at(j).at(k));
                    points.push_back(boundaries.at(i).at(j).at(k)->toDoubleArray());
                    boundaries.at(i).at(j).at(k)->setId(std::to_string(vertices_id++));
                    vertices_edges.insert(std::make_pair(boundaries.at(i).at(j).at(k), incident));
                } else if(k > 0 && *boundaries.at(i).at(j).at(k) == *boundaries.at(i).at(j).at(k - 1))
                {
                    std::cerr << "Boundary must be clean at this point." << std::endl;
                    exit(88);
                }

                polyline.push_back(static_cast<unsigned int>(std::stoi(boundaries.at(i).at(j).at(k)->getId())));
            }

            *boundaries.at(i).at(j).back() = *boundaries.at(i).at(j).at(0);

            polyline.push_back(static_cast<unsigned int>(std::stoi(boundaries.at(i).at(j).at(0)->getId())));
            polylines.push_back(polyline);

        }
    }

    for(unsigned int i = 0; i < constraints.size(); i++)
    {
        std::vector<unsigned int> polyline;
        for(unsigned int j = 0; j < constraints.at(i).size(); j++)
        {
            std::vector<std::shared_ptr<Vertex> >::iterator it = std::find_if(vertices.begin(), vertices.end(),
                      [constraints, i, j](std::shared_ptr<Vertex> p)
            {
                auto p1 = *constraints.at(i).at(j);
                auto p2 = *p;
                p1.setZ(0);
                p2.setZ(0);
                return *constraints.at(i).at(j) == *p;
            });

            if(it != vertices.end())
            {
                std::shared_ptr<Vertex> tmp = constraints.at(i).at(j);
                *constraints.at(i).at(j) = **it;
                tmp.reset();
            }

            if(it == vertices.end())
            {
                std::vector<std::shared_ptr<Edge> > incident;
                addNewVertex(constraints.at(i).at(j));
                points.push_back(constraints.at(i).at(j)->toDoubleArray());
                constraints.at(i).at(j)->setId(std::to_string(vertices_id++));
                vertices_edges.insert(std::make_pair(constraints.at(i).at(j), incident));
            }
            polyline.push_back(static_cast<unsigned int>(std::stoi(constraints.at(i).at(j)->getId())));
        }
        polylines.push_back(polyline);
    }

    for(auto v : constraintVertices)
    {
        std::vector<std::shared_ptr<Edge> > incident;
        vertices.push_back(v);
        v->setId(std::to_string(vertices_id++));
        vertices_edges.insert(std::make_pair(v, incident));
        points.push_back(v->toDoubleArray());
    }
    uint noDEMPointsNumber = points.size();
    std::cout << "Starting triangulation " << std::endl;
    TriHelper::TriangleHelper helper(points, polylines, holes_seeds, true);

    for(auto p : points)
        delete p;

    std::vector<unsigned int> generated_triangles = helper.getTriangles();
    //std::vector<double*> generated_points = helper.getAddedPoints();
    std::vector<double*> generated_points(helper.getPoints().begin() + noDEMPointsNumber, helper.getPoints().end());
    this->getVertex(vertices.size() - 1)->print(std::cout);
    for(unsigned int i = 0; i < generated_points.size(); i++)
    {
        std::vector<std::shared_ptr<Edge> > incident;
        std::shared_ptr<Vertex> v = addNewVertex(generated_points.at(i)[0], generated_points.at(i)[1], 0/*generated_points.at(i)[2]*/);
        v->setId(std::to_string(vertices_id++));
        if(v->getZ() > 5000)
            std::cout << std::endl;
        vertices_edges.insert(std::make_pair(vertices.back(), incident));
    }

    std::cout << "Ended! Created " << generated_triangles.size() / 3 << " triangles." << std::endl << "Building mesh structure:" << std::endl;
    for(unsigned int i = 0; i < generated_triangles.size() / 3; i++)
    {
        unsigned int v1 = generated_triangles.at(i * 3), v2 = generated_triangles.at(i * 3 + 1), v3 = generated_triangles.at(i * 3 + 2);
        std::shared_ptr<Edge> e1, e2, e3;

        e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v2));
        if(e1 == nullptr)
            e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v1));
        if(e1 == nullptr)
        {
            addNewEdge(vertices.at(v1), vertices.at(v2));
            e1 = edges.back();
            e1->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v1)).push_back(e1);
        }
        vertices.at(v1)->setE0(e1);
        e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v3));
        if(e2 == nullptr)
            e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v2));
        if(e2 == nullptr)
        {
            addNewEdge(vertices.at(v2), vertices.at(v3));
            e2 = edges.back();
            e2->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v2)).push_back(e2);
        }
        vertices.at(v2)->setE0(e2);
        e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v1));
        if(e3 == nullptr)
            e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v3));
        if(e3 == nullptr)
        {
            addNewEdge(vertices.at(v3), vertices.at(v1));
            e3 = edges.back();
            e3->setId(std::to_string(edges_id++));
            vertices_edges.at(vertices.at(v3)).push_back(e3);
        }
        vertices.at(v3)->setE0(e3);

        addNewTriangle(e1, e2, e3);
        triangles.back()->setId(std::to_string(triangles_id++));
        if(e1->getT1() == nullptr)
           e1->setT1(triangles.back());
        else if(e1->getT2() == nullptr)
            e1->setT2(triangles.back());
        else
            return 9;
        if(e2->getT1() == nullptr)
            e2->setT1(triangles.back());
        else if(e2->getT2() == nullptr)
            e2->setT2(triangles.back());
        else
            return 9;
        if(e3->getT1() == nullptr)
            e3->setT1(triangles.back());
        else if(e3->getT2() == nullptr)
            e3->setT2(triangles.back());
        else
            return 9;

        if(i % 100 == 0)
            std::cout << i * 100 / generated_triangles.size() << "%\r" << std::flush;;

    }

    std::cout << "Ended!" << std::endl;
    return 0;
}

int TriangleMesh::loadPLY(std::string filename)
{
    std::ifstream fileStream(filename);

    if(fileStream.is_open())
    {
        unsigned int vertices_number = 0;
        unsigned int triangles_number = 0;
        unsigned int reached_vertex_id = 0;
        unsigned int reached_edge_id = 0;
        unsigned int reached_triangle_id = 0;
        std::map<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Edge> > > vertices_edges;
        std::string line;
        std::getline(fileStream, line);
        if(line.compare("ply") != 0)
            return 1;
        std::getline(fileStream, line);
        if(line.compare("format ascii 1.0") == 0)
        {
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.substr(0, 14).compare("element vertex") != 0)
                return 2;
            vertices_number = stoi(line.substr(15, line.size() - 15));
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.substr(0, 8).compare("property") != 0)
                return 3;
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.substr(0, 8).compare("property") != 0)
                return 3;
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.substr(0, 8).compare("property") != 0)
                return 3;
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.substr(0, 12).compare("element face") != 0)
                return 4;
            triangles_number = stoi(line.substr(13, line.size() - 13));
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.compare("property list uchar int vertex_indices") != 0)
                return 5;
            while(std::getline(fileStream, line) && line.substr(0, 7).compare("comment") == 0); //removing all comments
            if(line.compare("end_header") != 0)
                return 6;
            std::cout << "Loading vertices: " << std::endl;
            for(unsigned int i = 0; i < vertices_number; i++)
            {
                if(!std::getline(fileStream, line))
                    return 7;
                std::stringstream sstream(line);
                double x, y, z;
                sstream >> x;
                sstream >> y;
                sstream >> z;
                std::shared_ptr<Vertex> v = addNewVertex(x, y, z);
                v->setId(std::to_string(reached_vertex_id++));
                std::vector<std::shared_ptr<Edge> > incident;
                vertices_edges.insert(std::make_pair(v, incident));
                if(i % 100 == 0)
                    std::cout << i * 100 / vertices_number << "%\r" << std::flush;
            }
            std::cout << "Ended! Loaded " << vertices_number << " vertices." << std::endl << "Loading triangles:" << std::endl;
            for(unsigned int i = 0; i < triangles_number; i++)
            {
                if(!std::getline(fileStream, line))
                    return 8;
                std::stringstream sstream(line);
                unsigned int polyVertNum;
                unsigned int v1, v2, v3;
                std::shared_ptr<Edge> e1, e2, e3;
                sstream >> polyVertNum;
                sstream >> v1;
                sstream >> v2;
                sstream >> v3;

                e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v2));
                if(e1 == nullptr)
                    e1 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v1));
                if(e1 == nullptr)
                {
                    addNewEdge(vertices.at(v1), vertices.at(v2));
                    e1 = edges.back();
                    e1->setId(std::to_string(reached_edge_id++));
                    vertices_edges.at(vertices.at(v1)).push_back(e1);
                }
                vertices.at(v1)->setE0(e1);
                e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v2)), vertices.at(v3));
                if(e2 == nullptr)
                    e2 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v2));
                if(e2 == nullptr)
                {
                    addNewEdge(vertices.at(v2), vertices.at(v3));
                    e2 = edges.back();
                    e2->setId(std::to_string(reached_edge_id++));
                    vertices_edges.at(vertices.at(v2)).push_back(e2);
                }
                vertices.at(v2)->setE0(e2);
                e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v3)), vertices.at(v1));
                if(e3 == nullptr)
                    e3 = searchEdgeContainingVertex(vertices_edges.at(vertices.at(v1)), vertices.at(v3));
                if(e3 == nullptr)
                {
                    addNewEdge(vertices.at(v3), vertices.at(v1));
                    e3 = edges.back();
                    e3->setId(std::to_string(reached_edge_id++));
                    vertices_edges.at(vertices.at(v3)).push_back(e3);
                }
                vertices.at(v3)->setE0(e3);

                addNewTriangle(e1, e2, e3);
                triangles.back()->setId(std::to_string(reached_triangle_id++));
                if(e1->getT1() == nullptr)
                   e1->setT1(triangles.back());
                else if(e1->getT2() == nullptr)
                    e1->setT2(triangles.back());
                else
                    return 9;
                if(e2->getT1() == nullptr)
                    e2->setT1(triangles.back());
                else if(e2->getT2() == nullptr)
                    e2->setT2(triangles.back());
                else
                    return 9;
                if(e3->getT1() == nullptr)
                    e3->setT1(triangles.back());
                else if(e3->getT2() == nullptr)
                    e3->setT2(triangles.back());
                else
                    return 9;

                if(i % 100 == 0)
                    std::cout << i * 100 / triangles_number << "%\r" << std::flush;;

            }
            std::cout << "Ended! Loaded " << triangles_number << " triangles." << std::endl;

        }
        fileStream.close();
        return 0;
    }
    else
        return -1;

}
