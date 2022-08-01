
#include <document.h>
#include <filereadstream.h>
#include <coordsconverter.h>

#include "annotationfilemanager.h"
#include "lineannotation.h"
#include "geotiff.h"
#include "nanoflann.hpp"
#include "utilities.h"
#include "building.h"
#include "buildingsgroup.h"
static unsigned char BLUE[3] =   {0,0,255};
static const int BUFFER_SIZE = 65536;
int main(int argc, char *argv[])
{
//    if(argc < 3)
//    {
//        std::cerr << "Missing one or more file parameters" << std::endl;
//        return 1;
//    }

//    std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
//    mesh->load(argv[1]);

//    std::vector<std::vector<double> >  points_vector;
//    for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
//    {

//        std::vector<double> point = { mesh->getVertex(i)->getX(),
//                                      mesh->getVertex(i)->getY(),
//                                      0};
//        points_vector.push_back(point);
//    }
//    KDTree tree(points_vector);

//    std::string shortestPathJsonFile = argv[2];
//    FILE* fp = fopen(shortestPathJsonFile.c_str(),"r");

//    std::string shortestPathAnnotationFile = shortestPathJsonFile.substr(0,shortestPathJsonFile.find_last_of(".") + 1) + "ant";


//    std::shared_ptr<LineAnnotation> annotation = std::make_shared<LineAnnotation>();

//    if(fp != nullptr)
//    {
//        char buffer[BUFFER_SIZE];
//        rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

//        rapidjson::Document document;
//        if(!(document.ParseStream(frs).HasParseError())){

//            assert(document.HasMember("Attr value") && document["Attr value"].IsDouble());
//            assert(document.HasMember("Attribute") && document["Attribute"].IsString());
//            assert(document.HasMember("geometry") && document["geometry"].IsArray());

//            double value = document["Attr value"].GetDouble();
//            std::string attributeName = document["Attribute"].GetString();
//            annotation->setId(0);
//            annotation->setTag(attributeName + "-based shortest path");
//            annotation->setColor(BLUE);
//            annotation->setHierarchyLevel(-1);
//            annotation->setMesh(mesh);
//            rapidjson::Value& path = document["geometry"].GetArray();
//            std::vector<std::vector<std::shared_ptr<Vertex> > > polylines;
//            std::vector<std::shared_ptr<Vertex> > polyline;
//            CoordsConverter converter(32633);
//            std::shared_ptr<Vertex> prev;
//            for (rapidjson::SizeType i = 0; i < path.Size(); i++)
//            {
//                assert(path[i].IsArray());
//                rapidjson::Value& obj = path[i].GetArray();
//                assert(obj.Size() == 2 && obj[0].IsArray() && obj[1].IsArray());
//                rapidjson::Value& source = obj[0].GetArray();
//                rapidjson::Value& destination = obj[1].GetArray();
//                std::shared_ptr<Vertex> sourceVertex, destinationVertex;
//                assert(source.Size() == 2 && source[0].IsDouble() && source[1].IsDouble());
//                assert(destination.Size() == 2 && destination[0].IsDouble() && destination[1].IsDouble());
//                double lon = source[0].GetDouble();
//                double lat = source[1].GetDouble();
//                double x, y;
//                converter.convertToUTM(lat, lon, x, y);


//                point_t point = {mesh->getVertex(i)->getX(), mesh->getVertex(i)->getY(), 0};
//                size_t ret_index = tree.nearest_index(point);
//                std::cout << ret_index << std::endl;
//                sourceVertex = mesh->getVertex(ret_index);

//                lon = destination[0].GetDouble();
//                lat = destination[1].GetDouble();
//                converter.convertToUTM(lat, lon, x, y);

//                point[0] = x;
//                point[1] = y;
//                ret_index = tree.nearest_index(point);
//                destinationVertex = mesh->getVertex(ret_index);

//                if(prev == nullptr)
//                    polyline.push_back(sourceVertex);
//                else
//                    assert(prev->getId().compare(sourceVertex->getId()) == 0);
//                polyline.push_back(destinationVertex);
//                prev = destinationVertex;
//            }
//            polylines.push_back(polyline);
//            annotation->setPolyLines(polylines);
//            mesh->addAnnotation(annotation);
//        }
//        fclose(fp);
//    }



//    AnnotationFileManager manager;
//    manager.setMesh(mesh);
//    manager.writeAnnotations(shortestPathAnnotationFile);
//    mesh.reset();
//    std::cout << "Ended" << std::endl;

//    auto bui1 = std::make_shared<Building>();
//    auto bui2 = std::make_shared<Building>();
//    auto bui3 = std::make_shared<Building>();
//    auto bui4 = std::make_shared<Building>();
//    auto bui5 = std::make_shared<Building>();
//    auto bui6 = std::make_shared<Building>();
//    bui1->setId("0");
//    bui2->setId("1");
//    bui3->setId("2");
//    bui4->setId("3");
//    bui5->setId("4");
//    bui6->setId("5");
//    PointList b1 = {
//        std::make_shared<Point>(-5,-1,0),
//        std::make_shared<Point>(-4,-1,0),
//        std::make_shared<Point>(-3,-1,0),
//        std::make_shared<Point>(-2,-1,0),
//        std::make_shared<Point>(-1,-1,0),
//        std::make_shared<Point>(-1,0,0),
//        std::make_shared<Point>(-1,1,0),
//        std::make_shared<Point>(-1,2,0),
//        std::make_shared<Point>(-2,2,0),
//        std::make_shared<Point>(-2,1,0),
//        std::make_shared<Point>(-2,0,0),
//        std::make_shared<Point>(-3,0,0),
//        std::make_shared<Point>(-4,0,0),
//        std::make_shared<Point>(-5,0,0),
//        std::make_shared<Point>(-5,-1,0)
//    };
//    PointList b2 = {
//        std::make_shared<Point>(-1,0,0),
//        std::make_shared<Point>(-1,-1,0),
//        std::make_shared<Point>(-2,-1,0),
//        std::make_shared<Point>(-3,-1,0),
//        std::make_shared<Point>(-3,-2,0),
//        std::make_shared<Point>(-2,-2,0),
//        std::make_shared<Point>(0,-2,0),
//        std::make_shared<Point>(0,0,0),
//        std::make_shared<Point>(-1,0,0)
//    };
//    PointList b3 = {
//        std::make_shared<Point>(-1,3,0),
//        std::make_shared<Point>(-2,3,0),
//        std::make_shared<Point>(-2,2,0),
//        std::make_shared<Point>(-1,2,0),
//        std::make_shared<Point>(-1,3,0)
//    };
//    PointList b4 = {
//        std::make_shared<Point>(-3,0,0),
//        std::make_shared<Point>(-2,0,0),
//        std::make_shared<Point>(-2,1,0),
//        std::make_shared<Point>(-4,1,0),
//        std::make_shared<Point>(-4,0,0),
//        std::make_shared<Point>(-3,0,0)
//    };
//    PointList b5 = {
//        std::make_shared<Point>(-2,4,0),
//        std::make_shared<Point>(-2,3,0),
//        std::make_shared<Point>(-1,3,0),
//        std::make_shared<Point>(-1,4,0),
//        std::make_shared<Point>(-2,4,0)
//    };
//    PointList b6 = {
//        std::make_shared<Point>(-2,-2,0),
//        std::make_shared<Point>(-3,-2,0),
//        std::make_shared<Point>(-3,-3,0),
//        std::make_shared<Point>(-2,-3,0),
//        std::make_shared<Point>(-2,-2,0)
//    };
//    bui1->addOuterBoundary(b1);
//    bui2->addOuterBoundary(b2);
//    bui3->addOuterBoundary(b3);
//    bui4->addOuterBoundary(b4);
//    bui5->addOuterBoundary(b5);
//    bui6->addOuterBoundary(b6);
//    bui1->addAdjacentBuilding(bui2);
//    bui1->addAdjacentBuilding(bui3);
//    bui1->addAdjacentBuilding(bui4);
//    bui2->addAdjacentBuilding(bui1);
//    bui2->addAdjacentBuilding(bui6);
//    bui3->addAdjacentBuilding(bui1);
//    bui3->addAdjacentBuilding(bui5);
//    bui4->addAdjacentBuilding(bui1);
//    //std::vector<PointList> res = bui1->mergeWithAllAdjacents();
//    BuildingsGroup bg;
//    bg.addBuilding(bui1);
//    bg.addBuilding(bui2);
//    bg.addBuilding(bui3);
//    bg.addBuilding(bui4);
//    bg.addBuilding(bui5);
//    bg.addBuilding(bui6);
//    std::vector<PointList> res = bg.extractOverallBasePolygons();
//    for(auto b : res)
//    {
//        std::cout << "New boundary" << std::endl;
//        for(auto p : b)
//            p->print(std::cout, BracketsType::NONE, " ");
//    }
    Point p1(-4.71,1.7,0), p2(3.59,2.56,0);
    Point p3(11.89, 3.42,0);
    double angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(5.71,5.52,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(3.11,5.86,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(-1.35,3.54,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(-2.67,1.62,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(3.75,-1.96,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;
    p3 = Point(7.77,-0.18,0);
    angle = (p2 - p1).computeAngle(p3 - p2);
    std::cout << angle << std::endl;

    return 0;
}
