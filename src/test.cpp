
#include <fstream>
#include <string>
#include <cstdlib>
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <limits.h>
#include <stdlib.h>
#include <liblas/liblas.hpp>


#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <coordsconverter.h>
#include <semanticsfilemanager.hpp>
#include <lineannotation.hpp>
#include "utilities.h"

static unsigned char BLUE[3] =   {0,0,255};
static const int BUFFER_SIZE = 65536;
using namespace SemantisedTriangleMesh;


int readLiDAR(std::string filename, std::vector<liblas::Point> &points)
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
            liblas::Point p = reader.GetPoint();
            points.push_back(p);
        }

        ifs.close();

        return 0;
    }
    return -1;

}

int writeLidar(std::string inputFilename, std::string outputFilename, std::vector<liblas::Point> points, std::vector<uint> pointsIndices)
{
    std::ifstream ifs;
    std::ofstream ofs;
    ifs.open(inputFilename, std::ios::in | std::ios::binary);
    ofs.open(outputFilename, std::ios::out | std::ios::binary);

    if(ifs.is_open() && ofs.is_open())
    {
        liblas::ReaderFactory f;
        std::string s;
        liblas::Reader reader = f.CreateWithStream(ifs);
        liblas::Header header = reader.GetHeader();
        header.SetPointRecordsCount(pointsIndices.size());
        liblas::Writer writer(ofs, header);


        for(uint i = 0; i < pointsIndices.size(); i++)
        {
            liblas::Point lidarPoint = points.at(pointsIndices[i]);
            writer.WritePoint(lidarPoint);

        }

        ifs.close();
        ofs.close();
        return 0;
    }
    return -1;
}

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



//    SemanticsFileManager manager;
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
//    auto mesh = new SemantisedTriangleMesh::TriangleMesh();
//    auto new_mesh = new SemantisedTriangleMesh::TriangleMesh();
//    mesh->load(argv[1]);

//    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > >  bounds;
//    Utilities::load_shapefile_shp(argv[2], bounds);
//    std::map<std::shared_ptr<Vertex>, uint> oldToNewVertexMap;
//    std::map<std::shared_ptr<Edge>, uint> oldToNewEdgesMap;
//    std::map<std::shared_ptr<Triangle>, uint> oldToNewTrianglesMap;
//    std::queue<std::shared_ptr<Vertex> > V;
//    for(uint i = 0; i < mesh->getVerticesNumber(); i++)
//        if(!Utilities::isPointOutsideBounds(mesh->getVertex(i), bounds))
//        {
//            V.push(mesh->getVertex(i));
//            new_mesh->addNewVertex(mesh->getVertex(i));
//            oldToNewVertexMap.insert(std::make_pair(mesh->getVertex(i), i));
//            break;
//        }

//    do {
//        auto current_vertex = V.front();
//        V.pop();
//        current_vertex->addFlag(FlagType::USED);
//        if(!Utilities::isPointOutsideBounds(current_vertex, bounds))
//        {
//            auto vv = current_vertex->getVV();
//            auto ve = current_vertex->getVE();
//            for(auto e : ve)
//            {
//                auto opposite_vertex = e->getOppositeVertex(current_vertex);
//                if(opposite_vertex->searchFlag(FlagType::USED) == -1 && opposite_vertex->searchFlag(FlagType::VISITED) == -1)
//                {
//                    opposite_vertex->addFlag(FlagType::VISITED);
//                    V.push(opposite_vertex);
//                    oldToNewVertexMap.insert(std::make_pair(opposite_vertex, new_mesh->getVerticesNumber()));
//                    new_mesh->addNewVertex(opposite_vertex);
//                }
//            }

//            for(uint i = 0; i < ve.size(); i++)
//            {
//                auto current_edge = ve.at(i);
//                auto next_vertex = ve.at((i + 1) % ve.size())->getOppositeVertex(current_vertex);
//                auto opposite_vertex = current_edge->getOppositeVertex(current_vertex);

//                auto new_current_vertex = new_mesh->getVertex(oldToNewVertexMap.at(current_vertex));
//                auto new_opposite_vertex = new_mesh->getVertex(oldToNewVertexMap.at(opposite_vertex));
//                std::shared_ptr<Edge> new_current_edge, new_next_edge, new_closing_edge;
//                if(i == 0 && oldToNewEdgesMap.find(current_edge) == oldToNewEdgesMap.end())
//                {
//                    oldToNewEdgesMap.insert(std::make_pair(current_edge, new_mesh->getEdgesNumber()));
//                    new_current_edge = new_mesh->addNewEdge(new_current_vertex, new_opposite_vertex);
//                } else
//                    new_current_edge = new_mesh->getEdge(oldToNewEdgesMap.at(current_edge));

//                auto new_next_vertex = new_mesh->getVertex(oldToNewVertexMap.at(next_vertex));
//                auto next_edge = current_vertex->getCommonEdge(next_vertex);
//                if(oldToNewEdgesMap.find(next_edge) == oldToNewEdgesMap.end())
//                {
//                    oldToNewEdgesMap.insert(std::make_pair(next_edge, new_mesh->getEdgesNumber()));
//                    new_next_edge = new_mesh->addNewEdge(new_current_vertex, new_next_vertex);
//                } else
//                    new_next_edge = new_mesh->getEdge(oldToNewEdgesMap.at(next_edge));

//                auto closing_edge = opposite_vertex->getCommonEdge(next_vertex);
//                if(oldToNewEdgesMap.find(closing_edge) == oldToNewEdgesMap.end())
//                {
//                    oldToNewEdgesMap.insert(std::make_pair(closing_edge, new_mesh->getEdgesNumber()));
//                    new_closing_edge = new_mesh->getEdge(oldToNewEdgesMap.at(closing_edge));
//                }else
//                    new_closing_edge = new_mesh->getEdge(oldToNewEdgesMap.at(closing_edge));

//                auto current_triangle = current_edge->getLeftTriangle(current_vertex);
//                if(oldToNewTrianglesMap.find(current_triangle) == oldToNewTrianglesMap.end())
//                {
//                    oldToNewTrianglesMap.insert(std::make_pair(current_triangle, new_mesh->getTrianglesNumber()));
//                    auto t = new_mesh->addNewTriangle(new_current_edge, new_closing_edge, new_next_edge);
//                }



//            }

//        }



//    } while(V.size() > 0);
//    new_mesh->save("provina.ply");
//    delete mesh;
//    delete new_mesh;
    /*auto mesh = new SemantisedTriangleMesh::TriangleMesh();
    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > >  bounds;
    mesh->load(argv[1]);
    Utilities::load_shapefile_shp(argv[2], bounds);
    for(uint i = 0; i < mesh->getVerticesNumber(); i++)
        if(Utilities::isPointOutsideBounds(mesh->getVertex(i), bounds))
        {
            if(mesh->getVertex(i)->searchFlag(FlagType::TO_BE_REMOVED) == -1)
                mesh->getVertex(i)->addFlag(FlagType::TO_BE_REMOVED);
            auto ve = mesh->getVertex(i)->getVE();
            for(auto e : ve)
            {
                if(e->searchFlag(FlagType::TO_BE_REMOVED) == -1)
                    e->addFlag(FlagType::TO_BE_REMOVED);
                auto t = e->getLeftTriangle(mesh->getVertex(i));
                if(t != nullptr && t->searchFlag(FlagType::TO_BE_REMOVED) == -1)
                    t->addFlag(FlagType::TO_BE_REMOVED);
            }
        }
    mesh->removeFlaggedVertices();
    mesh->removeFlaggedEdges();
    mesh->removeFlaggedTriangles();
    double max_length = -std::numeric_limits<double>::max();
    double mean_length = 0;
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();
    for(uint i = 0; i < mesh->getVerticesNumber(); i++)
    {
        if(mesh->getVertex(i)->getZ() < min_z)
            min_z = mesh->getVertex(i)->getZ();
        if(mesh->getVertex(i)->getZ() > max_z)
            max_z = mesh->getVertex(i)->getZ();
    }

    for(uint i = 0; i < mesh->getEdgesNumber(); i++)
    {
        auto l = mesh->getEdge(i)->computeLength();
        if(l > max_length)
            max_length = l;
        mean_length += l;
    }
    mean_length /= mesh->getEdgesNumber();


    for(uint i = 0; i < mesh->getEdgesNumber(); i++)
    {
        auto e = mesh->getEdge(i);
        if(e->computeLength() < 0.1 && e->searchFlag(FlagType::TO_BE_REMOVED) == -1)
        {
            e->collapse();
        }
    }

    mesh->removeFlaggedVertices();
    mesh->removeFlaggedEdges();
    mesh->removeFlaggedTriangles();
    std::cout << "Max length: " << max_length << " min z: " << min_z << " max z: " << max_z << std::endl;

    mesh->resetIds();
    mesh->save("provina.ply", 15);
    delete mesh;*/

//    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > points;
//    std::string filename = argv[1];
//    auto retValue = readLiDAR(filename, points);
//    if(retValue != 0)
//        exit(retValue);

//    GDALAllRegister();
//    CPLPushErrorHandler(CPLQuietErrorHandler);
//    GDALDataset *geotiffDataset;
//    GDALDriver *driverGeotiff;
//    GDALRasterBand *geotiffBand;
//    double minX = std::numeric_limits<double>::max();
//    double minY = std::numeric_limits<double>::max();
//    double maxX = -std::numeric_limits<double>::max();
//    double maxY = -std::numeric_limits<double>::max();
//    for(auto p : points)
//    {
//        if(p->getX() < minX)
//            minX = p->getX();
//        if(p->getX() > maxX)
//            maxX = p->getX();
//        if(p->getY() < minY)
//            minY = p->getY();
//        if(p->getY() > maxY)
//            maxY = p->getY();
//    }
//    uint imageXResolution = std::ceil(maxX - minX);
//    uint imageYResolution = std::ceil(maxY - minY);
//    double transform[6] = {minX, 1, 0, maxY, 0, 1};
//    double** heights = static_cast<double**>(malloc(imageYResolution * sizeof (double*)));
//    uint** heights_count = static_cast<uint**>(malloc(imageYResolution * sizeof (uint*)));
//    for(uint i = 0; i < imageYResolution; i++)
//    {
//        heights[i] = static_cast<double*>(malloc(imageXResolution * sizeof(double)));
//        heights_count[i] = static_cast<uint*>(malloc(imageXResolution * sizeof(uint)));
//    }

//    for(auto p : points)
//    {
//        uint i = imageYResolution - (std::floor(p->getY() - minY) + 1);
//        uint j = std::floor(p->getX() - minX);
//        heights[i][j] += p->getZ();
//        heights_count[i][j]++;
//    }

//    for(uint i = 0; i < imageYResolution; i++)
//        for(uint j = 0; j < imageXResolution; j++)
//            if(heights_count[i][j] > 0)
//                heights[i][j] /= heights_count[i][j];
//            else
//            {
//                double height = 0;
//                uint count = 0;
//                if(i > 0 && heights_count[i - 1][j] > 0)
//                {
//                    height += heights[i - 1][j];
//                    count++;
//                }
//                if(i < imageYResolution && heights_count[i + 1][j] > 0)
//                {
//                    height += heights[i + 1][j];
//                    count++;
//                }
//                if(j > 0 && heights_count[i][j - 1] > 0)
//                {
//                    height += heights[i][j - 1];
//                    count++;
//                }
//                if(j < imageXResolution && heights_count[i][j + 1] > 0)
//                {
//                    height += heights[i][j+1];
//                    count++;
//                }
//                if(count > 0)
//                    height /= count;
//                else
//                    exit(8);
//                heights[i][j] = height;
//            }


//    std::string geoTiffFilename = "Catania_DSM.tif";
//    driverGeotiff = GetGDALDriverManager()->GetDriverByName("GTiff");
//    geotiffDataset = driverGeotiff->Create(geoTiffFilename.c_str(),imageXResolution, imageYResolution, 1, GDT_Float32,NULL);
//    geotiffDataset->SetGeoTransform(transform);

//    geotiffDataset->SetProjection("EPSG:32633");
//    float *rowBuff = (float*) CPLMalloc(sizeof(float) * imageXResolution);

//      /* iterate through all pixels in 2D dBZ
//       * reflectivity array and assign values into rowBuff
//       * array. Then write each row into Geotiff object.
//       */

//    for(int row = 0; row < imageYResolution; row++) {
//        for(int col = 0; col < imageXResolution; col++) {
//            rowBuff[col] = (float)heights[row][col];
//        }
//        geotiffDataset->GetRasterBand(1)->RasterIO(GF_Write, 0, row, imageXResolution, 1, rowBuff, imageXResolution, 1, GDT_Float32, 0, 0);
//    }

//    GDALClose(geotiffDataset) ;
//    GDALDestroyDriverManager();
//    for(uint i = 0; i < imageYResolution; i++)
//    {
//        free(heights[i]);
//        free(heights_count[i]);
//    }
//    free(heights);
//    free(heights_count);

    std::string filename = argv[1];
    std::vector<std::vector<uint> > pointClouds;
    std::vector<liblas::Point> points;
    auto retValue = readLiDAR(filename, points);
    if(retValue != 0)
        exit(retValue);

    std::string json_directory = argv[2];
    if(json_directory.at(json_directory.size() - 1) != '/')
        json_directory.append("/");
    std::vector<std::string> json_files = Utilities::globVector(json_directory+"*.json");
    uint total_points = 0;
    for(unsigned int i = 0; i < json_files.size(); i++)
    {
        FILE* fp = fopen(json_files[i].c_str(),"r");
        if(fp != nullptr)
        {
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            std::string id, description, file;
            uint startRecord = 0, numRecords = 0;
            if((document.ParseStream(frs).HasParseError()))
                exit(1);

            if(!(document.HasMember("ID") && document["ID"].IsUint()))
                exit(2);

            id = std::to_string(document["ID"].GetUint());
            if(!(document.HasMember("description") && document["description"].IsString()))
                exit(3);

            description = document["description"].GetString();

            if(!(document.HasMember("las_blocks") && document["las_blocks"].IsArray()))
                exit(4);
            rapidjson::Value& blocksList = document["las_blocks"];
            for (rapidjson::SizeType j = 0; j < blocksList.Size(); j++) // rapidjson uses SizeType instead of size_t.
            {
                std::vector<uint> pointCloud;
                pointCloud.clear();
                rapidjson::Value& jsonBlock = blocksList[j];
                if(!jsonBlock.IsObject())
                    exit(5);
                if(!(jsonBlock.HasMember("file") && jsonBlock["file"].IsString()))
                    exit(6);
                file = jsonBlock["file"].GetString();
                if(!(jsonBlock.HasMember("start_record") && jsonBlock["start_record"].IsUint()))
                    exit(7);
                startRecord = jsonBlock["start_record"].GetUint();
                if(!(jsonBlock.HasMember("num_records") && jsonBlock["num_records"].IsUint()))
                    exit(8);
                numRecords = jsonBlock["num_records"].GetUint();
                for(uint k = 0; k < numRecords; k++)
                    pointCloud.push_back(startRecord + k);
                total_points += numRecords;

                std::string s = "./point_clouds/output";
                s.append(id);
                s.append(".las");
                writeLidar(filename, s, points, pointCloud);

            }
            delete fp;
        }
    }
    std::cout << total_points;


    return 0;
}
