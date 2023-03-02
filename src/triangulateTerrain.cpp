#include <iostream>
#include <liblas/liblas.hpp>
#include <shapefil.h>
#include <TriangleMesh.h>
#include <Point.h>
#include <trianglehelper1.h>
#include <utilities.h>

using namespace SemantisedTriangleMesh;
std::vector<std::vector<std::shared_ptr<Point> > >  bounds;

bool isPointOutsideBounds(const std::shared_ptr<Point> p)
{
    bool outside = true, inside = false;
    outside = !Utilities::isPointInsidePolygon(p, bounds[0]);
    for(uint i = 1; i < bounds.size(); i++)
    {
        if(Utilities::isPointInsidePolygon(p, bounds[i]))
        {
            return true;
        }
    }
    return outside;
}

int readLidar(std::string filename, std::vector<std::shared_ptr<Point> > &points)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios::in | std::ios::binary);

    if(ifs.is_open())
    {
        liblas::ReaderFactory f;
        std::string s;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header const& header = reader.GetHeader();

        std::cout << "Compressed: " << header.Compressed() << std::endl;
        std::cout << "Signature: " << header.GetFileSignature() << std::endl;
        std::cout << "Points count: " << header.GetPointRecordsCount() << std::endl;

        while (reader.ReadNextPoint())
        {
            liblas::Point const& p = reader.GetPoint();
            std::shared_ptr<Point> p_ = std::make_shared<Point>(p.GetX(), p.GetY(), p.GetZ());
            std::string s = p.GetClassification().GetClassName();
            if(s.find("Ground") != std::string::npos &&
//               s.find("High Noise") == std::string::npos &&
//               s.find("Building") == std::string::npos &&
//               s.find("Vegetation") == std::string::npos &&
               p.GetReturnNumber() == p.GetNumberOfReturns() // &&
//            if(s.find("Low Point") == std::string::npos &&
//               s.find("High Noise") == std::string::npos &&
//               s.find("Vegetation") == std::string::npos &&
//               p.GetReturnNumber() == 0 &&
//               !isPointOutsideBounds(p_)
                    )
                points.push_back(p_);
        }
        ifs.close();
        return 0;
    }
    return -1;
}

int rewriteLidar(std::string inputFilename, std::string outputFilename, std::vector<std::shared_ptr<Point> > &points)
{
    std::ifstream ifs;
    std::ofstream ofs;
    ifs.open(inputFilename, std::ios::in | std::ios::binary);
    ofs.open(outputFilename, std::ios::out | std::ios::binary);
    uint erased = 0;
    uint toBeErased = 90099898;

    for(uint i = 0; i < points.size(); i++)
    {
        auto p = points.at(i);
        if(p->getInfo() != nullptr && *static_cast<int*>(p->getInfo()) == toBeErased)
            continue;
        auto comparator = [p](std::shared_ptr<SemantisedTriangleMesh::Point> v)
        {
            auto tmp1 = *p;
            auto tmp2 = *v;
            tmp1.setZ(0.0);
            tmp2.setZ(0.0);
            return tmp1 == tmp2;
        };
        auto it = std::find_if(points.begin() + i + 1, points.end(), comparator);
        while(it != points.end())
        {
            uint pos = it - points.begin();
            (*it)->setInfo(&toBeErased);
            erased++;
            it = std::find_if(points.begin() + pos + 1, points.end(), comparator);
        }
    }
    std::cout << "We need to erase " << erased << " points" << std::endl;

    if(ifs.is_open() && ofs.is_open())
    {
        liblas::ReaderFactory f;
        std::string s;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header header = reader.GetHeader();
        uint pointsNumber = header.GetPointRecordsCount();
        liblas::Writer writer(ofs, header);
        uint i = 0;
        while(reader.ReadNextPoint())
        {
            liblas::Point lidarPoint = reader.GetPoint();
            std::string s = lidarPoint.GetClassification().GetClassName();
            if(s.find("Ground") != std::string::npos && lidarPoint.GetReturnNumber() == lidarPoint.GetNumberOfReturns())
            {
                auto p = points.at(i);
                if(p->getInfo() != nullptr && *static_cast<int*>(p->getInfo()) == toBeErased)
                    continue;
                if(!(std::isnan(p->getX()) || std::isnan(p->getY()) || std::isnan(p->getZ()) ||
                     std::isinf(p->getX()) || std::isinf(p->getY()) || std::isinf(p->getZ())))
                {
                    lidarPoint.SetCoordinates(p->getX(), p->getY(), p->getZ());
                }
                i++;
            }

            writer.WritePoint(lidarPoint);

        }

        liblas::Header wheader = writer.GetHeader();
        header.SetPointRecordsCount(pointsNumber - erased);
        ifs.close();
        ofs.close();
        return 0;
    }
    return -1;
}

std::shared_ptr<Edge> searchEdgeContainingVertex(std::vector<std::shared_ptr<Edge> > list, std::shared_ptr<Vertex> v)
{
    for(unsigned int i = 0; i < list.size(); i++)
        if(list.at(i)->hasVertex(v))
            return list.at(i);

    return nullptr;
}

typedef std::vector<std::shared_ptr<Vertex> >  VertexList;

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "Missing LAS file" << std::endl << std::flush;
        return 1;
    }
    if(argc < 3)
    {
        std::cerr << "Missing bounds shapefile" << std::endl << std::flush;
        return 2;
    }
    std::string lidarFilename = argv[1], boundsFilename = argv[2];
    std::vector<std::shared_ptr<Point> > dtmPoints;
    std::vector<double*> points;
    std::vector<std::vector<unsigned int> > polylines;
    std::map<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Edge> > > vertices_edges;
    std::vector<double *> holes;
    bool boundQuality = false;
    Utilities::load_shapefile_shp(boundsFilename, bounds);
    std::cout << "Reading LiDAR file..." << std::endl << std::flush;
    readLidar(lidarFilename, dtmPoints);

    std::cout << "Ended!" << std::endl << std::flush;

//    std::shared_ptr<Point> origin = std::make_shared<Point>(0,0,0);
//    Point min(std::numeric_limits<double>::max(),
//                          std::numeric_limits<double>::max(),
//                          0);
//    Point max(-std::numeric_limits<double>::max(),
//              -std::numeric_limits<double>::max(),
//              0);
//    unsigned int points_number = 0;

//    TriangleMesh* mesh = new TriangleMesh();

//    for(auto p : dtmPoints)
//    {
//        double* doublePoint = p->toDoubleArray();

//        points.push_back(doublePoint);
//        auto v = mesh->addNewVertex(*p);
//        v->setId(std::to_string((mesh->getVerticesNumber() - 1)));

//        std::vector<std::shared_ptr<Edge> > incident;
//        vertices_edges.insert(std::make_pair(v, incident));
//        if(p->getX() < min.getX())
//            min.setX(p->getX());
//        if(p->getY() < min.getY())
//            min.setY(p->getY());
//        if(p->getX() > max.getX())
//            max.setX(p->getX());
//        if(p->getY() > max.getY())
//            max.setY(p->getY());
//        (*origin) += (*p);
//        points_number++;
//    }

//    double city_size = (max - min).norm();
//    (*origin) /= points_number;


//    std::vector<VertexList > external_bb = {{
//        std::make_shared<Vertex>(min.getX(), min.getY(), 0.0),
//        std::make_shared<Vertex>(max.getX(), min.getY(), 0.0),
//        std::make_shared<Vertex>(max.getX(), max.getY(), 0.0),
//        std::make_shared<Vertex>(min.getX(), max.getY(), 0.0)
//    }};
//    unsigned int pointsNumber = mesh->getVerticesNumber();
//    std::vector<std::shared_ptr<Edge> > incident;
//    mesh->addNewVertex(external_bb[0][0]);
//    external_bb[0][0]->setId(std::to_string((mesh->getVerticesNumber() - 1)));
//    vertices_edges.insert(std::make_pair(external_bb[0][0], incident));
//    points.push_back(external_bb[0][0]->toDoubleArray());
//    mesh->addNewVertex(external_bb[0][1]);
//    external_bb[0][1]->setId(std::to_string((mesh->getVerticesNumber() - 1)));
//    vertices_edges.insert(std::make_pair(external_bb[0][1], incident));
//    points.push_back(external_bb[0][1]->toDoubleArray());
//    mesh->addNewVertex(external_bb[0][2]);
//    external_bb[0][2]->setId(std::to_string((mesh->getVerticesNumber() - 1)));
//    vertices_edges.insert(std::make_pair(external_bb[0][2], incident));
//    points.push_back(external_bb[0][2]->toDoubleArray());
//    mesh->addNewVertex(external_bb[0][3]);
//    external_bb[0][3]->setId(std::to_string((mesh->getVerticesNumber() - 1)));
//    vertices_edges.insert(std::make_pair(external_bb[0][3], incident));
//    points.push_back(external_bb[0][3]->toDoubleArray());

//    std::vector<unsigned int> polyline = {pointsNumber, pointsNumber + 1, pointsNumber + 2, pointsNumber + 3, pointsNumber};
//    polylines.push_back(polyline);
//    pointsNumber = points.size();

//    for(auto p : points)
//    {
//        p[0] -= origin->getX();
//        p[1] -= origin->getY();
//        p[2] -= origin->getZ();
//        p[0] /= city_size;
//        p[1] /= city_size;
//        p[2] /= city_size;
//    }

//    std::cout << "Computing triangulation..." << std::endl << std::flush;
//    TriHelper::TriangleHelper helper(points, polylines, holes, boundQuality);

//    for(auto p : points)
//        delete p;

//    std::vector<unsigned int> generated_triangles = helper.getTriangles();
//    //std::vector<double*> generated_points = helper.getAddedPoints();
//    std::vector<double*> generated_points(helper.getPoints().begin() + pointsNumber, helper.getPoints().end());
//    for(unsigned int i = 0; i < generated_points.size(); i++)
//    {
//        std::vector<std::shared_ptr<Edge> > incident;
//        std::shared_ptr<Vertex> v = mesh->addNewVertex(generated_points.at(i)[0], generated_points.at(i)[1], generated_points.at(i)[2]);
//        v->setId(std::to_string(mesh->getVerticesNumber() - 1));
//        vertices_edges.insert(std::make_pair(v, incident));
//    }

//    std::cout << "Ended! Created " << generated_triangles.size() / 3 << " triangles." << std::endl << "Building mesh structure:" << std::endl;
//    for(unsigned int i = 0; i < generated_triangles.size() / 3; i++)
//    {
//        unsigned int v1 = generated_triangles.at(i * 3), v2 = generated_triangles.at(i * 3 + 1), v3 = generated_triangles.at(i * 3 + 2);
//        std::shared_ptr<Edge> e1, e2, e3;

//        e1 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v1)), mesh->getVertex(v2));
//        if(e1 == nullptr)
//            e1 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v2)), mesh->getVertex(v1));
//        if(e1 == nullptr)
//        {
//            e1 = mesh->addNewEdge(mesh->getVertex(v1), mesh->getVertex(v2));
//            e1->setId(std::to_string(mesh->getEdgesNumber() - 1));
//            vertices_edges.at(mesh->getVertex(v1)).push_back(e1);
//        }
//        mesh->getVertex(v1)->setE0(e1);
//        e2 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v2)), mesh->getVertex(v3));
//        if(e2 == nullptr)
//            e2 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v3)), mesh->getVertex(v2));
//        if(e2 == nullptr)
//        {
//            e2 = mesh->addNewEdge(mesh->getVertex(v2), mesh->getVertex(v3));
//            e2->setId(std::to_string(mesh->getEdgesNumber() - 1));
//            vertices_edges.at(mesh->getVertex(v2)).push_back(e2);
//        }
//        mesh->getVertex(v2)->setE0(e2);
//        e3 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v3)), mesh->getVertex(v1));
//        if(e3 == nullptr)
//            e3 = searchEdgeContainingVertex(vertices_edges.at(mesh->getVertex(v1)), mesh->getVertex(v3));
//        if(e3 == nullptr)
//        {
//            e3 = mesh->addNewEdge(mesh->getVertex(v3), mesh->getVertex(v1));
//            e3->setId(std::to_string(mesh->getEdgesNumber() - 1));
//            vertices_edges.at(mesh->getVertex(v3)).push_back(e3);
//        }
//        mesh->getVertex(v3)->setE0(e3);

//        auto t = mesh->addNewTriangle(e1, e2, e3);
//        t->setId(std::to_string(mesh->getTrianglesNumber() - 1));
//        if(e1->getT1() == nullptr)
//           e1->setT1(t);
//        else if(e1->getT2() == nullptr)
//            e1->setT2(t);
//        else
//            return 9;
//        if(e2->getT1() == nullptr)
//            e2->setT1(t);
//        else if(e2->getT2() == nullptr)
//            e2->setT2(t);
//        else
//            return 9;
//        if(e3->getT1() == nullptr)
//            e3->setT1(t);
//        else if(e3->getT2() == nullptr)
//            e3->setT2(t);
//        else
//            return 9;

//        if(i % 100 == 0)
//            std::cout << i * 100 / generated_triangles.size() << "%\r" << std::flush;;

//    }

//    std::cout << "Ended!" << std::endl;

//    mesh->save("triangulated.ply", 15);
//    mesh->smooth(SemantisedTriangleMesh::WeightType::Cotangent, 10, 1);
//    mesh->save("smoothed.ply", 15);
//    for(uint i = 0; i < dtmPoints.size(); i++)
//        dtmPoints[i]->setPosition(*mesh->getVertex(i));
    rewriteLidar(argv[1], "output1.las", dtmPoints);

//    delete mesh;
    bounds.clear();
    return 0;
}
