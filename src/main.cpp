#include <utilities.h>
#include <trianglehelper.h>
#include <Point.h>
#include <TriangleMesh.h>
#include <citygmlcore.h>

#include <geotiff.h>
#include <rapidjson/prettywriter.h>
#include <tinyxml2.h>

#include <annotationfilemanager.h>
#include <iostream>
#include <map>
#include <string>

using namespace std;
const double OFFSET = 5e-3;


int main(int argc, char *argv[])
{

    if(argc < 2)
    {
        std::cerr << "Missing OSM map parameter" << std::endl;
        return 1;
    }
    if(argc < 3)
    {
        std::cerr << "Missing DTM file parameter" << std::endl;
        return 2;
    }
    if(argc < 4)
    {
        std::cerr << "Missing DSM file parameter" << std::endl;
        return 3;
    }
    if(argc < 5)
    {
        std::cerr << "Missing LiDAR file parameter" << std::endl;
        return 4;
    }

    CityGMLCore manager(argv[1], argv[2], argv[3], argv[4]);

 //   manager.setLevel(0, "level0.ply", "annotations.ant");
    manager.buildLevel(0);
    manager.buildLevel(1);
//    std::cout << "Loading DTM" << std::endl;
//    GeoTiff* dtm_tiff = new GeoTiff(argv[3]);
//    std::cout << "Loading DSM" << std::endl;
//    GeoTiff* dsm_tiff = new GeoTiff(argv[4]);


//    double* geoTransform = dtm_tiff->GetGeoTransform();
//    Point min(geoTransform[0],
//              geoTransform[3] + dtm_tiff->GetDimensions()[1] * geoTransform[5],
//              0);
//    Point max(geoTransform[0] + dtm_tiff->GetDimensions()[0] * geoTransform[1],
//              geoTransform[3],
//              0);

//    std::vector<std::vector<std::shared_ptr<Point> > > boundaries;
//    Utilities::load_shapefile_shp(argv[1], boundaries);
//    std::cout << "Loaded " << boundaries.size() << " buildings' bases" << std::endl;
//    /*Utilities::load_shapefile_shp(argv[2], streets);
//    std::cout << "Loaded " << streets.size() << " streets' segments" << std::endl;*/
//    std::vector<std::shared_ptr<OSMNode> > nodes;
//    std::vector<std::shared_ptr<OSMWay> > arcs;
//    std::vector<std::shared_ptr<OSMRelation> > relations;
//    int out = Utilities::loadOSM("/home/andreas/Scrivania/Matera/map.osm", nodes, arcs, relations);

//    std::vector<std::shared_ptr<OSMWay> > buildings_arcs;
//    std::vector<std::shared_ptr<OSMWay> > streets_arcs;
//    /****Loading and preprocessing base polygons of buildings****/

//    if(out < 0)
//        return out;

//    for(unsigned int i = 0; i < arcs.size(); i++)
//    {
//        bool should_delete = false;
//        for(unsigned int j = 0; j < arcs.at(i)->getNodes().size(); j++)
//        {
//            if(arcs.at(i)->getNodes().at(j)->getCoordinates()->getX() < min.getX() ||
//               arcs.at(i)->getNodes().at(j)->getCoordinates()->getY() < min.getY() ||
//               arcs.at(i)->getNodes().at(j)->getCoordinates()->getZ() < min.getZ() ||
//               arcs.at(i)->getNodes().at(j)->getCoordinates()->getX() > max.getX() ||
//               arcs.at(i)->getNodes().at(j)->getCoordinates()->getY() > max.getY() ||
//               arcs.at(i)->getNodes().at(j)->getCoordinates()->getZ() > max.getZ() )
//            {
//                should_delete = true;
//                break;
//            }
//        }
//        if(should_delete)
//        {
//            for(unsigned int j = 0; j < arcs.at(i)->getNodes().size(); j++)
//            {
//                arcs.at(i)->getNodes().at(j)->getCoordinates().reset();
//                arcs.at(i)->getNodes().at(j).reset();
//            }
//            arcs.erase(arcs.begin() + i);
//            i--;
//        } else
//        {
//            if(arcs.at(i)->checkTag(std::make_pair("building","")))
//                buildings_arcs.push_back(arcs.at(i));

//            if(arcs.at(i)->checkTag(std::make_pair("highway","")) && (!arcs.at(i)->checkTag(std::make_pair("area","")) || arcs.at(i)->checkTag(std::make_pair("area","no"))))
//                streets_arcs.push_back(arcs.at(i));
//        }
//    }

//    Utilities::fix_lines(streets_arcs);


////    for(unsigned int i = 0; i < buildings_arcs.size(); i++)
////    {

////        std::vector<std::shared_ptr<Point> > boundary;
////        std::vector<std::shared_ptr<OSMNode> > nodes = buildings_arcs.at(i)->getNodes();
////        for(unsigned int j = 0; j < nodes.size() - 1; j++)
////            boundary.push_back(nodes.at(j)->getCoordinates());
////        boundaries.push_back(boundary);

////    }

////    boundaries.clear();

//    for(unsigned int i = 0; i < boundaries.size(); i++)
//    {

//        bool should_delete = false;
//        for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
//        {
//            if(boundaries.at(i).at(j)->getX() < min.getX() ||
//               boundaries.at(i).at(j)->getY() < min.getY() ||
//               boundaries.at(i).at(j)->getZ() < min.getZ() ||
//               boundaries.at(i).at(j)->getX() > max.getX() ||
//               boundaries.at(i).at(j)->getY() > max.getY() ||
//               boundaries.at(i).at(j)->getZ() > max.getZ() )
//            {
//                should_delete = true;
//                break;
//            }
//        }
//        if(should_delete)
//        {
//            for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
//            {
//                boundaries.at(i).at(j).reset();
//            }
//            boundaries.erase(boundaries.begin() + i);
//            i--;
//        }
//    }


//    std::shared_ptr<Point> origin = std::make_shared<Point>(0,0,0);
//    unsigned int points_number = 0;

//    std::cout << "Cleaning polygons/lines and normalizing points" << std::endl;
//    for(unsigned int i = 0; i < boundaries.size(); i++)
//    {
//        for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
//        {
//            (*origin) += (*boundaries.at(i).at(j));
//            points_number++;
//        }
//    }

//    for(unsigned int j = 0; j < dtm_tiff->GetDimensions()[0]; j++)
//    {
//        for(unsigned int k = 0; k < dtm_tiff->GetDimensions()[1]; k++)
//        {

//            Point p(geoTransform[0] + (k + 0.5) * geoTransform[1] + j * geoTransform[2],
//                      geoTransform[3] + k * geoTransform[4] + (j + 0.5) * geoTransform[5], 0);

//            (*origin) += p;
//            points_number++;
//        }
//    }
//    double city_size = (max - min).norm();
//    (*origin) /= points_number;

//    for(unsigned int i = 0; i < boundaries.size(); i++)
//        for(unsigned int j = 0; j < boundaries.at(i).size(); j++)
//        {
//            *boundaries.at(i).at(j) -= (*origin);
//            *boundaries.at(i).at(j) /= city_size;
//        }

//    for(unsigned int i = 0; i < nodes.size(); i++)
//    {
//        std::shared_ptr<OSMNode> n = nodes.at(i);
//        std::shared_ptr<Point> pos = n->getCoordinates();
//        *pos -= *origin;
//        *pos /= city_size;
//    }

//    Utilities::fix_polygons(boundaries);

//    for(unsigned int i = 0; i < boundaries.size(); i++)
//    {
//        for(unsigned int j = 0; j < boundaries.size(); j++)
//        {
//            if(i==j)
//                continue;
//            if(Utilities::isPolygonInsidePolygon(boundaries.at(i), boundaries.at(j)))
//            {
//                for(unsigned int k = 0; k < boundaries.at(i).size(); k++)
//                    boundaries.at(i).at(k).reset();
//                boundaries.erase(boundaries.begin() + i);
//                i--;
//                if(j > i)
//                    j--;
//                break;
//            }
//        }
//    }

//    std::cout << "Removing nodes into buildings"<< std::endl;
//    Utilities::removeOSMWayPointsInPolygons(streets_arcs, boundaries);
//    for(unsigned int i = 0; i < streets_arcs.size(); i++)
//        if(streets_arcs.at(i)->getNodes().size() < 2)
//        {
//            streets_arcs.erase(streets_arcs.begin() + i);
//            i--;
//        }

//    std::vector<std::vector<std::shared_ptr<Point> > > arcs_points;
//    for(unsigned int i = 0; i < streets_arcs.size(); i++)
//    {
//        std::vector<std::shared_ptr<Point> > arc_points;
//        std::vector<std::shared_ptr<OSMNode> > arc_nodes = streets_arcs.at(i)->getNodes();
//        for(unsigned int j = 0; j < arc_nodes.size(); j++)
//            arc_points.push_back(arc_nodes.at(j)->getCoordinates());
//        arcs_points.push_back(arc_points);
//    }

//    std::cout << "Refining lines:" << std::endl;
//    Utilities::refineLines(arcs_points, dtm_tiff, city_size, *origin);
//    for(unsigned int i = 0; i < arcs_points.size(); i++)
//        if(arcs_points.at(i).size() == 1)
//        {
//            std::cout <<"Found singlet" << std::endl;
//            exit(1312);
//        }
//    std::cout << "Removing inserted critical vertices:" << std::endl;

//    Utilities::removeLinePointsInPolygons(arcs_points, boundaries);

//    for(unsigned int i = 0; i < arcs_points.size(); i++)
//        if(arcs_points.at(i).size() < 2)
//        {
//            arcs_points.erase(arcs_points.begin() + i);
//            streets_arcs.erase(streets_arcs.begin() + i);
//            i--;
////            std::cout <<"Found singlet" << std::endl;
////            exit(1312);
//        }
//    std::vector<std::pair<unsigned int, unsigned int> > keptClippings = Utilities::removeSegmentsIntersectingPolygons(arcs_points, boundaries);

//    std::vector<std::vector<std::shared_ptr<Point> > > main_triangulation_boundaries;
//    std::vector<std::shared_ptr<Point> > external_bb = {
//        std::make_shared<Point>(min.getX(), min.getY(), 0),
//        std::make_shared<Point>(max.getX(), min.getY(), 0),
//        std::make_shared<Point>(max.getX(), max.getY(), 0),
//        std::make_shared<Point>(min.getX(), max.getY(), 0)};
//    *external_bb[0] = ((*external_bb[0]) - (*origin)) / city_size;
//    *external_bb[1] = ((*external_bb[1]) - (*origin)) / city_size;
//    *external_bb[2] = ((*external_bb[2]) - (*origin)) / city_size;
//    *external_bb[3] = ((*external_bb[3]) - (*origin)) / city_size;

//    main_triangulation_boundaries.push_back(external_bb);
//    for(unsigned int i = 0; i < boundaries.size(); i++)
//    {
//        std::vector<std::shared_ptr<Point> > hole;
//        unsigned int size = boundaries.at(i).size();
//        if(boundaries.at(i)[0] == boundaries.at(i).back())
//            size--;
//        for(unsigned int j = 0; j < size; j++)
//            hole.push_back(boundaries.at(i).at(j));

//        main_triangulation_boundaries.push_back(hole);
//    }
//    std::cout << "Ended" << std::endl << "Creating triangulation:" << std::endl;

//    TriangleMesh* mesh = new TriangleMesh();
//    mesh->triangulate(main_triangulation_boundaries, arcs_points);
//    mesh->save("blublu.ply", 15);


//    for(unsigned int i = 0; i < keptClippings.size(); i++)
//    {
//        unsigned int numberOfNewLines = keptClippings.at(i).second - keptClippings.at(i).first;
//        for(unsigned int j = 0; j < numberOfNewLines; j++)
//        {
//            std::vector<std::shared_ptr<Point> > clipping = arcs_points.at(keptClippings.at(i).first + j + 1);
//            arcs_points.at(keptClippings.at(i).first).insert(arcs_points.at(keptClippings.at(i).first).end(), clipping.begin(), clipping.end());
//        }
//        arcs_points.erase(arcs_points.begin() + keptClippings.at(i).first + 1, arcs_points.begin() + keptClippings.at(i).second + 1);
//        for(unsigned int j = i + 1; j < keptClippings.size(); j++)
//        {
//            keptClippings.at(j).first -= numberOfNewLines;
//            keptClippings.at(j).second -= numberOfNewLines;
//        }
//    }


//    std::cout << "Associating height to buildings" << std::endl;
//    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > bases_elevations = Utilities::extractPolygonsHeights(main_triangulation_boundaries, dtm_tiff, city_size, *origin);

//    unsigned int reachedEdgeId = mesh->getEdgesNumber();
//    unsigned int reachedTriangleId = mesh->getTrianglesNumber();


//    for(unsigned int i = 0; i < main_triangulation_boundaries.size(); i++)
//    {

//        std::reverse(main_triangulation_boundaries.at(i).begin(), main_triangulation_boundaries.at(i).end());
//        std::vector<unsigned int> shell_triangles;
//        unsigned int size = main_triangulation_boundaries.at(i).size();
//        std::map<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Edge> > > vertices_edges;

//        for(unsigned int j = 0; j < size; j++)
//        {

//            std::shared_ptr<Vertex> v1 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(j));
//            std::shared_ptr<Vertex> v2 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at((j + 1) % size));
//            std::vector<std::shared_ptr<Vertex> > shortestPath = Utilities::dijkstra(v1, v2);
//            if(shortestPath.size() > 1)
//            {
////                std::cout << "Non-existent vertices catched: adding to polygon " << i << " " << j<< std::endl;
////                std::cout << "Original edge: " << std::endl;
////                v1->print(std::cout);
////                v2->print(std::cout);
////                for(unsigned int k = 0; k < shortestPath.size(); k++)
////                    shortestPath.at(k)->print(std::cout);
//                main_triangulation_boundaries.at(i).insert(main_triangulation_boundaries.at(i).begin() + (j + 1) % size,
//                                                           shortestPath.begin(),
//                                                           shortestPath.begin() + shortestPath.size() - 1);
//                size += shortestPath.size() - 1;
//            } else if(shortestPath.size() < 1){
//                std::cerr << "Impossible case" << std::endl;

//                for(unsigned int j = 0; j < size; j++)
//                    main_triangulation_boundaries.at(i).at(j)->print(std::cerr,BracketsType::NONE, " ");
//                v1->print(std::cerr);
//                v2->print(std::cerr);
//                for(unsigned int k = 0; k < main_triangulation_boundaries.at(i).size(); k++)
//                {
//                    std::cerr << "Vertex " << k << " on boundary" << std::endl;
//                    std::shared_ptr<Vertex> v =  static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(k));
//                    if(v!= nullptr)
//                    {
//                        v->print(std::cerr);
//                        std::vector<std::shared_ptr<Edge> > ve = v->getVE();
//                        for(unsigned int l = 0; l < ve.size(); l++)
//                        {
//                            ve[l]->print(std::cerr);
//                            std::shared_ptr<Vertex> v_ = ve[l]->getOppositeVertex(v);
//                            if(v_ != nullptr)
//                                v_->print(std::cerr);
//                            else
//                                std::cerr << "Opposite vertex not available!!!" << std::endl;
//                        }
//                        std::cerr << std::endl;
//                    } else
//                    {
//                        std::cerr << "Vertex not available!!!" << std::endl;
//                    }

//                }
//                std::cerr << std::flush;
//                delete mesh;
//                for(unsigned int i = 0; i < main_triangulation_boundaries.size(); i++)
//                    for(unsigned int j = 0; j < main_triangulation_boundaries[i].size(); j++)
//                        if(main_triangulation_boundaries.at(i).at(j) != nullptr)
//                            main_triangulation_boundaries.at(i).at(j).reset();

//                for(unsigned int i = 0; i < arcs_points.size(); i++)
//                    for(unsigned int j = 0; j < arcs_points[i].size(); j++)
//                        if(arcs_points.at(i).at(j) != nullptr)
//                            if(static_pointer_cast<Vertex>(arcs_points.at(i).at(j)) != nullptr)
//                                arcs_points.at(i).at(j).reset();
//                return(5);
//            }
//        }

//        for(unsigned int j = 0; j < size; j++)
//        {
//            std::vector<std::shared_ptr<Edge> > connected, ve = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(j))->getVE();

//            vertices_edges.insert(std::make_pair(static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(j)), ve));
//            main_triangulation_boundaries.at(i).push_back(mesh->addNewVertex(main_triangulation_boundaries.at(i).at(j)->getX(),
//                                                                          main_triangulation_boundaries.at(i).at(j)->getY(),
//                                                                          main_triangulation_boundaries.at(i).at(j)->getZ() + OFFSET));
//            vertices_edges.insert(std::make_pair(static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).back()), connected));

//            static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).back())->setId(std::to_string(mesh->getVerticesNumber() - 1));

//        }

//        for(int j = 0; j < size; j++)
//        {
//            unsigned int pos1 = Utilities::mod(j - 1, size);
//            unsigned int pos2 = size + j;
//            unsigned int pos3 = size + Utilities::mod(j - 1, size);
//            unsigned int pos4 = j;
//            std::shared_ptr<Vertex> v1 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(pos1));
//            std::shared_ptr<Vertex> v2 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(pos2));
//            std::shared_ptr<Vertex> v3 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(pos3));
//            std::shared_ptr<Vertex> v4 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(pos4));

//            std::shared_ptr<Edge> e1 = mesh->addNewEdge(v1, v2);
//            vertices_edges.at(v1).push_back(e1);
//            vertices_edges.at(v2).push_back(e1);
//            e1->setId(std::to_string(reachedEdgeId++));
//            std::shared_ptr<Edge> e2 = mesh->addNewEdge(v2, v3);
//            vertices_edges.at(v2).push_back(e2);
//            vertices_edges.at(v3).push_back(e2);
//            e2->setId(std::to_string(reachedEdgeId++));
//            std::shared_ptr<Edge> e3 = mesh->addNewEdge(v3, v1);
//            vertices_edges.at(v3).push_back(e3);
//            vertices_edges.at(v1).push_back(e3);
//            e3->setId(std::to_string(reachedEdgeId++));
//            std::shared_ptr<Triangle> t1 = mesh->addNewTriangle(e1, e2, e3);
//            t1->setId(std::to_string(reachedTriangleId++));
//            e1->setT1(t1);
//            e2->setT1(t1);
//            if(e3->getT1() == nullptr)
//                e3->setT1(t1);
//            else if(e3->getT2() == nullptr)
//                e3->setT2(t1);
//            else
//                std::cout << "Found" << std::endl;
//            std::shared_ptr<Edge> e4 = mesh->searchEdgeContainingVertex(vertices_edges.at(v1), v4);
//            if(e4 == nullptr)
//                e4 = mesh->searchEdgeContainingVertex(vertices_edges.at(v4), v1);
//            std::shared_ptr<Edge> e5 = mesh->searchEdgeContainingVertex(vertices_edges.at(v4), v2);
//            if(e5 == nullptr)
//                e5 = mesh->searchEdgeContainingVertex(vertices_edges.at(v2), v4);
//            if(e5 == nullptr)
//            {
//                e5 = mesh->addNewEdge(v4, v2);
//                vertices_edges.at(v4).push_back(e5);
//                vertices_edges.at(v2).push_back(e5);
//                e5->setId(std::to_string(reachedEdgeId++));
//            }
//            std::shared_ptr<Triangle> t2 = mesh->addNewTriangle(e4, e5, e1);
//            t2->setId(std::to_string(reachedTriangleId++));

//            if(e4->getT1() == nullptr)
//                e4->setT1(t2);
//            else if(e4->getT2() == nullptr)
//                e4->setT2(t2);
//            else
//                std::cout << "Found" << std::endl;
//            if(e5->getT1() == nullptr)
//                e5->setT1(t2);
//            else if(e5->getT2() == nullptr)
//                e5->setT2(t2);
//            else
//                std::cout << "Found" << std::endl;
//            e1->setT2(t2);

//        }

//        std::vector<double*> boundary;
//        std::vector<std::vector<double*> > holes;
//        std::vector<double*> constraints;
//        std::vector<std::pair<unsigned int, unsigned int> > constraint_segments;
//        for(unsigned int j = 0; j < size; j++)
//        {
//            boundary.push_back(main_triangulation_boundaries.at(i)[size + j]->toDoubleArray());
//        }
//        TriHelper::TriangleHelper helper(boundary, holes, constraints, constraint_segments);
//        std::vector<unsigned int> enclosing_triangles = helper.getTriangles();

//        for(unsigned int j = 0; j < enclosing_triangles.size() / 3; j++)
//        {
//            std::shared_ptr<Vertex> v1 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(size + enclosing_triangles[j * 3]));
//            std::shared_ptr<Vertex> v2 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(size + enclosing_triangles[j * 3 + 1]));
//            std::shared_ptr<Vertex> v3 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(i).at(size + enclosing_triangles[j * 3 + 2]));
//            std::shared_ptr<Edge> e1 = mesh->searchEdgeContainingVertex(vertices_edges.at(v1), v2);
//            if(e1 == nullptr)
//            {
//                e1 = mesh->addNewEdge(v1, v2);
//                e1->setId(std::to_string(reachedEdgeId++));
//                vertices_edges.at(v1).push_back(e1);
//                vertices_edges.at(v2).push_back(e1);
//            }
//            std::shared_ptr<Edge> e2 = mesh->searchEdgeContainingVertex(vertices_edges.at(v2), v3);
//            if(e2 == nullptr)
//            {
//                e2 = mesh->addNewEdge(v2, v3);
//                e2->setId(std::to_string(reachedEdgeId++));
//                vertices_edges.at(v2).push_back(e2);
//                vertices_edges.at(v3).push_back(e2);
//            }
//            std::shared_ptr<Edge> e3 = mesh->searchEdgeContainingVertex(vertices_edges.at(v3), v1);
//            if(e3 == nullptr)
//            {
//                e3 = mesh->addNewEdge(v3, v1);
//                e3->setId(std::to_string(reachedEdgeId++));
//                vertices_edges.at(v3).push_back(e3);
//                vertices_edges.at(v1).push_back(e3);
//            }
//            std::shared_ptr<Triangle> t = mesh->addNewTriangle(e1, e2, e3);
//            t->setId(std::to_string(reachedTriangleId++));
//            if(e1->getT1() == nullptr)
//                e1->setT1(t);
//            else if(e1->getT2() == nullptr)
//                e1->setT2(t);
//            else
//                std::cerr << "Non manifold configuration!" << std::endl;

//            if(e2->getT1() == nullptr)
//                e2->setT1(t);
//            else if(e2->getT2() == nullptr)
//                e2->setT2(t);
//            else
//                std::cerr << "Non manifold configuration!" << std::endl;

//            if(e3->getT1() == nullptr)
//                e3->setT1(t);
//            else if(e3->getT2() == nullptr)
//                e3->setT2(t);
//            else
//                std::cerr << "Non manifold configuration!" << std::endl;
//        }

//    }


//    std::cout << "Associating heights to buildings." << std::endl;
//    float** elevations = dtm_tiff->GetRasterBand(1);
//    float** heights = dsm_tiff->GetRasterBand(1);

//    std::cout << "Ended" << std::endl << "Adapting heights to DTM" << std::endl;
//    Utilities::associateHeights(mesh, dtm_tiff, city_size, *origin);
//    std::cout << "Ended" << std::endl;



////        std::shared_ptr<Vertex> v0 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(0).at(0));
////        std::shared_ptr<Vertex> v1 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(0).at(1));
////        std::shared_ptr<Vertex> v2 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(0).at(2));
////        std::shared_ptr<Vertex> v3 = static_pointer_cast<Vertex>(main_triangulation_boundaries.at(0).at(3));
////        std::shared_ptr<Vertex> v0_ = mesh->addNewVertex(v0->getX(), v0->getY(), 0);
////        std::shared_ptr<Vertex> v1_ = mesh->addNewVertex(v1->getX(), v1->getY(), 0);
////        std::shared_ptr<Vertex> v2_ = mesh->addNewVertex(v2->getX(), v2->getY(), 0);
////        std::shared_ptr<Vertex> v3_ = mesh->addNewVertex(v3->getX(), v3->getY(), 0);


////    delete dtm_tiff;
////    delete dsm_tiff;
//    //De-normalise boundaries points
//    for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
//    {
//        std::shared_ptr<Vertex> v = mesh->getVertex(i);
//        v->setX(v->getX() * city_size + origin->getX());
//        v->setY(v->getY() * city_size + origin->getY());
//    }

//    origin.reset();
//    for(unsigned int i = 0; i < main_triangulation_boundaries.size(); i++)
//    {

//        unsigned int row_min = -1, col_min = -1, row_max = 0, col_max = 0;
//        double elevation_min = std::numeric_limits<double>::max(), elevation_max = -std::numeric_limits<double>::max(), elevation_mean = 0;
//        double height_min = std::numeric_limits<double>::max(), height_max = -std::numeric_limits<double>::max(), height_mean = 0;
//        //Computing min-max-mean height values
//        for(unsigned int k = 0; k < bases_elevations.at(i).size(); k++)
//        {
//            if(row_min > bases_elevations.at(i)[k].first)
//                row_min = bases_elevations.at(i)[k].first;
//            if(row_max < bases_elevations.at(i)[k].first)
//                row_max = bases_elevations.at(i)[k].first;
//            if(col_min > bases_elevations.at(i)[k].second)
//                col_min = bases_elevations.at(i)[k].second;
//            if(col_max < bases_elevations.at(i)[k].second)
//                col_max = bases_elevations.at(i)[k].second;

//            elevation_mean += elevations[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];
//            height_mean += heights[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];
//            if(elevation_min > elevations[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second])
//                elevation_min = elevations[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];

//            if(elevation_max < elevations[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second])
//                elevation_max = elevations[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];

//            if(height_min > heights[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second])
//                height_min = heights[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];

//            if(height_max < heights[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second])
//                height_max = heights[bases_elevations.at(i)[k].first][bases_elevations.at(i)[k].second];

//        }
//        elevation_mean /= bases_elevations.at(i).size();
//        height_mean /= bases_elevations.at(i).size();
//        for(unsigned int j = 0; j < main_triangulation_boundaries.at(i).size(); j++)
//        {
//            if(j < main_triangulation_boundaries.at(i).size() / 2)
//            {
//                main_triangulation_boundaries.at(i).at(j)->setZ(elevation_mean);
//            } else
//                if(i == 0)
//                    main_triangulation_boundaries.at(i).at(j)->setZ(0);
//                else
//                    main_triangulation_boundaries.at(i).at(j)->setZ(height_mean);
//        }
//    }
//    mesh->save("base_triangulated.ply", 15);
//    for(unsigned int i = 0; i < dtm_tiff->GetDimensions()[0]; i++)
//        if(heights[i] != nullptr)
//            delete[] elevations[i];
//    for(unsigned int i = 0; i < dtm_tiff->GetDimensions()[0]; i++)
//        if(heights[i] != nullptr)
//            delete[] heights[i];



//    std::cout <<"Ended!" << std::endl;

//    std::cout <<"Writing annotations." << std::endl;
//    unsigned char building_color[] = {255, 215, 0};
//    ofstream annotationFile("buildings.ant");
//    rapidjson::StringBuffer s_base;
//    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s_base);
//    writer.Reset(s_base);
//    writer.StartObject();
//    writer.Key("annotations");
//    writer.StartArray();

//    for(unsigned int i = 1; i < main_triangulation_boundaries.size(); i++)
//    {
//        std::vector<std::shared_ptr<Point> > boundary(main_triangulation_boundaries.at(i).begin(),
//                                      main_triangulation_boundaries.at(i).begin() + main_triangulation_boundaries.at(i).size() / 2);
//        writer.StartObject();
//        writer.Key("id");
//        writer.Uint(i - 1);
//        writer.Key("tag");
//        writer.String(("building n° " + std::to_string(i - 1)).c_str());
//        writer.Key("father");
//        writer.Int(-1);
//        writer.Key("level");
//        writer.Uint(0);
//        writer.Key("color");
//        writer.StartArray();
//        writer.Int(building_color[0]);
//        writer.Int(building_color[1]);
//        writer.Int(building_color[2]);
//        writer.EndArray();
//        writer.Key("type");
//        writer.String("Area");
//        writer.Key("boundaries");
//        writer.StartArray();

//        writer.StartArray();
//        for(unsigned int j = 0; j < boundary.size(); j++)
//        {
//            std::shared_ptr<Vertex> v = static_pointer_cast<Vertex>(boundary.at(j));
//            if(stoi(v->getId()) < 0 || stoi(v->getId()) > mesh->getVerticesNumber())
//                std::cout << "Found deleted vertex in building: " << i << " " << j << std::endl;
//            else
//            {
//                writer.Int(stoi(v->getId()));
//            }
//        }
//        writer.Int(stoi(static_pointer_cast<Vertex>(boundary.at(0))->getId()));
//        writer.EndArray();
//        writer.EndArray();
//        writer.Key("attributes");
//        writer.StartArray();
//        //Add here attributes
//        writer.EndArray();
//        writer.EndObject();
//    }

//    writer.EndArray();
//    writer.EndObject();
//    if(annotationFile.is_open())
//    {
//        annotationFile << s_base.GetString();
//        annotationFile.close();
//    }

//    annotationFile.open("streets.ant");
//    s_base.Clear();
//    writer.Reset(s_base);
//    writer.StartObject();
//    writer.Key("annotations");
//    writer.StartArray();
//    unsigned char street_color[] = {0, 0, 0};
//    unsigned char green[] = {0, 255, 0};
//    unsigned char yellowgreen[] = {210,255,105};
//    unsigned char yellow[] = {255,255,0};
//    unsigned char orange[] = {255,131,0};
//    unsigned char salmon[] = {255,160,122};
//    unsigned char red[] = {255, 0, 0};
//    unsigned char violet[] = {155,38,182};
//    std::map<OSMNode*, std::shared_ptr<Vertex> > traversed_nodes;

//    for(unsigned int i = 0; i < arcs_points.size(); i++)
//    {

//        writer.StartObject();
//        writer.Key("id");
//        writer.Uint(main_triangulation_boundaries.size() - 1 + i);
//        writer.Key("tag");
//        writer.String(("street n° " + std::to_string(i)).c_str());
//        writer.Key("father");
//        writer.Int(-1);
//        writer.Key("level");
//        writer.Uint(0);
//        writer.Key("color");
//        writer.StartArray();
//        writer.Int(street_color[0]);
//        writer.Int(street_color[1]);
//        writer.Int(street_color[2]);
//        writer.EndArray();
//        writer.Key("type");
//        writer.String("Line");
//        writer.Key("polylines");

//        writer.StartArray();
//        writer.StartArray();
//        std::cout.precision(15);
//        for(unsigned int j = 0; j < arcs_points.at(i).size(); j++)
//        {
//            std::shared_ptr<Vertex> v = static_pointer_cast<Vertex>(arcs_points.at(i).at(j));
//            unsigned int id = stoi(v->getId());
//            if(arcs_points.at(i).at(j)->getInfo() != nullptr)
//            {
//                OSMNode* n = static_cast<OSMNode*>(arcs_points.at(i).at(j)->getInfo());
//                if(traversed_nodes.find(n) == traversed_nodes.end())
//                    traversed_nodes.insert(std::make_pair(n, v));
//            }
//            if(id > mesh->getVerticesNumber())
//                std::cout << "Found deleted vertex in street: " << i << " " << j << std::endl;
//            else
//            {
//                writer.Int(id);
//            }
//        }
//        writer.EndArray();
//        writer.EndArray();
//        writer.Key("attributes");
//        writer.StartArray();
//        writer.StartObject();
//        writer.Key("id");
//        writer.Int(0);
//        writer.Key("name");
//        writer.String("osmid");
//        writer.Key("type");
//        writer.String("Semantic");
//        writer.Key("value");
//        writer.String(streets_arcs.at(i)->getId().c_str());
//        writer.EndObject();
//        writer.EndArray();
//        writer.EndObject();
//    }
//    unsigned int i = 0;
//    for(std::map<OSMNode*, std::shared_ptr<Vertex> >::iterator it = traversed_nodes.begin(); it != traversed_nodes.end(); it++)
//    {
//        writer.StartObject();
//        writer.Key("id");
//        writer.Uint(main_triangulation_boundaries.size() + streets_arcs.size() - 1 + i);
//        writer.Key("tag");
//        writer.String(("node n° " + std::to_string(i++)).c_str());
//        writer.Key("father");
//        writer.Int(-1);
//        writer.Key("level");
//        writer.Uint(0);
//        writer.Key("color");
//        writer.StartArray();
//        writer.Int(street_color[0]);
//        writer.Int(street_color[1]);
//        writer.Int(street_color[2]);
//        writer.EndArray();
//        writer.Key("type");
//        writer.String("Point");
//        writer.Key("points");

//        writer.StartArray();
//        writer.Int(stoi(it->second->getId()));
//        writer.EndArray();
//        writer.Key("attributes");
//        writer.StartArray();
//        writer.StartObject();
//        writer.Key("id");
//        writer.Int(0);
//        writer.Key("name");
//        writer.String("osmid");
//        writer.Key("type");
//        writer.String("Semantic");
//        writer.Key("value");
//        writer.String(it->first->getId().c_str());
//        writer.EndObject();
//        writer.EndArray();
//        writer.EndObject();
//    }
//    writer.EndArray();
//    writer.EndObject();

//    if(annotationFile.is_open())
//    {
//        annotationFile << s_base.GetString();
//        annotationFile.close();
//    }

////    std::cout <<"Computing slopes." << std::endl;

////    double overall_min_slope = 0;
////    double overall_max_slope = 0.261799;
////    ofstream streetsSlopesFile("slopes.ant");
////    rapidjson::StringBuffer s_slopes;
////    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer1(s_slopes);
////    unsigned int counter = 0;
////    s_slopes.Clear();
////    writer1.Reset(s_slopes);
////    writer1.StartObject();
////    writer1.Key("annotations");
////    writer1.StartArray();
////    for(unsigned int i = 0; i < arcs_points.size(); i++)
////    {
////        for(unsigned int j = 1; j < arcs_points.at(i).size(); j++)
////        {
////            /************SLOPES*****************/
////            writer1.StartObject();
////            writer1.Key("id");
////            writer1.Uint(counter);
////            writer1.Key("tag");
////            writer1.String(("street edge n° " + std::to_string(counter++)).c_str());
////            writer1.Key("father");
////            writer1.Int(-1);
////            writer1.Key("level");
////            writer1.Uint(0);
////            writer1.Key("color");
////            writer1.StartArray();
////            unsigned char color[3] = {0,0,0};
////            Point v = *arcs_points.at(i).at(j) - *arcs_points.at(i).at(j - 1);
////            Point projected(v.getX(), v.getY(), 0);
////            v /= v.norm();
////            projected /= projected.norm();
////            double angle = v.computeAngle(projected);
////            if(angle < 0)
////                angle *= -1;

////           // Utilities::interpolate(green, red, (angle - overall_min_slope) / (overall_max_slope - overall_min_slope), color);

////            if(angle >= 0 && angle < 0.01745330748016422)
////            {
////                color[0] = green[0];
////                color[1] = green[1];
////                color[2] = green[2];
////            }else if(angle >= 0.01745330748016422 && angle < 0.034906615)
////            {
////                color[0] = yellowgreen[0];
////                color[1] = yellowgreen[1];
////                color[2] = yellowgreen[2];
////            }else if(angle >= 0.034906615 && angle < 0.06981323)
////            {
////                color[0] = yellow[0];
////                color[1] = yellow[1];
////                color[2] = yellow[2];
////            }else if(angle >= 0.06981323 && angle < 0.13962646)
////            {
////                color[0] = orange[0];
////                color[1] = orange[1];
////                color[2] = orange[2];
////            }else if(angle >= 0.13962646 && angle < 0.27925292)
////            {
////                color[0] = salmon[0];
////                color[1] = salmon[1];
////                color[2] = salmon[2];
////            } else
////            {
////                color[0] = violet[0];
////                color[1] = violet[1];
////                color[2] = violet[2];
////            }
////            writer1.Uint(color[0]);
////            writer1.Uint(color[1]);
////            writer1.Uint(color[2]);
////            writer1.EndArray();
////            writer1.Key("type");
////            writer1.String("Line");
////            writer1.Key("polylines");

////            writer1.StartArray();
////            writer1.StartArray();

////            for(unsigned int j = 0; j < arcs_points.at(i).size(); j++)
////                if(stoi(static_pointer_cast<Vertex>(arcs_points.at(i).at(j))->getId()) > mesh->getVerticesNumber())
////                    std::cout << "Found deleted vertex in street: " << i << " " << j << std::endl;
////                else
////                {
////                    writer1.Int(stoi(static_pointer_cast<Vertex>(arcs_points.at(i).at(j))->getId()));
////                }

////            writer1.EndArray();
////            writer1.EndArray();
////            writer1.EndObject();
////        }
////    }
////    writer1.EndArray();
////    writer1.EndObject();
////    if(streetsSlopesFile.is_open())
////    {
////        streetsSlopesFile << s_slopes.GetString();
////        streetsSlopesFile.close();
////    }

////    std::map<std::string, std::shared_ptr<Point> > osmid_point_association;
////    for(unsigned int i = 0; i < arcs_points.size(); i++)
////        for(unsigned int j = 0; j < arcs_points.at(i).size(); j++)
////            if(arcs_points.at(i).at(j)->getInfo() != nullptr)
////            {
////                OSMNode* n = static_cast<OSMNode*>(arcs_points.at(i).at(j)->getInfo());
////                if(n != nullptr)
////                    osmid_point_association.insert(std::make_pair(n->getId(), arcs_points.at(i).at(j)));
////                else
////                    std::cout << "nullptr: " << i << " " << j << std::endl;
////            }
////    std::cout << "Beginning slopes computation" << std::endl;
////    ofstream slopesFile("attributes.json");
////    s_base.Clear();
////    writer.Reset(s_base);
////    writer.StartObject();
////    writer.Key("Arcs_attributes");
////    writer.StartArray();
////    counter = 0;
////    for(unsigned int i = 0; i < arcs_points.size(); i++)
////    {
////        std::vector<std::shared_ptr<OSMNode> > arc_nodes = streets_arcs.at(i)->getNodes();
////        std::shared_ptr<Vertex> v1 = static_pointer_cast<Vertex>(arcs_points.at(i).at(0));
////        OSMNode* n1 = static_cast<OSMNode*>(v1->getInfo());

////        for(unsigned int j = 1; j < arcs_points.at(i).size(); j++)
////        {
////            double min_slope = std::numeric_limits<double>::max(), mean_slope = 0.0, max_slope = -std::numeric_limits<double>::max();

////            std::shared_ptr<Vertex> v2 = static_pointer_cast<Vertex>(arcs_points.at(i).at(j));
////            OSMNode* n2 = static_cast<OSMNode*>(v2->getInfo());
////            if(n2 == nullptr)
////                continue;

////            if(v1 == nullptr || v2 == nullptr){
////                std::cerr << "Arc id " << streets_arcs.at(i)->getId() << std::endl;
////                std::cerr << "Pair (" << n1->getId() << "," << n2->getId() << "); ("  << n1->getId() << "," << n2->getId() << std::endl;
////                std::cerr << "No id vertex association" << std::endl;
////                std::cerr << "Position: " << i << " " << j << std::endl;
////                counter++;
////                continue;
////            }
////            writer.StartObject();
////            writer.Key("Node1");
////            writer.String(n1->getId().c_str());
////            writer.Key("Node2");
////            writer.String(n2->getId().c_str());
////            std::vector<std::shared_ptr<Vertex> > path = Utilities::dijkstra(v1, v2);
////            path.insert(path.begin(), v1);
////            for(unsigned int k = 1; k < path.size(); k++)
////            {
////                Point v = *path.at(k) - *path.at(k - 1);
////                Point projected(v.getX(), v.getY(), 0);
////                v /= v.norm();
////                projected /= projected.norm();
////                double sign = path.at(k - 1)->getZ() > path.at(k)->getZ() ? -1 : 1;
////                double angle = sign * v.computeAngle(projected);
////                mean_slope += angle;
////                if(angle < min_slope)
////                    min_slope = angle;
////                if(angle > max_slope)
////                    max_slope = angle;

////            }
////            mean_slope /= path.size() - 1;

////            writer.Key("Attributes");
////            writer.StartArray();
////            writer.StartObject();
////            writer.Key("type");
////            writer.String("min_slope");
////            writer.Key("value");
////            writer.Double(min_slope);
////            writer.EndObject();
////            writer.StartObject();
////            writer.Key("type");
////            writer.String("max_slope");
////            writer.Key("value");
////            writer.Double(max_slope);
////            writer.EndObject();
////            writer.StartObject();
////            writer.Key("type");
////            writer.String("mean_slope");
////            writer.Key("value");
////            writer.Double(mean_slope);
////            writer.EndObject();
////            writer.EndArray();
////            writer.EndObject();

////            v1 = v2;
////            n1 = n2;
////        }
////    }
////    std::cout << std::endl;
////    writer.EndArray();
////    writer.EndObject();
////    if(slopesFile.is_open())
////    {
////        slopesFile << s_base.GetString();
////        slopesFile.close();
////    }

//    delete mesh;
//    for(unsigned int i = 0; i < main_triangulation_boundaries.size(); i++)
//        for(unsigned int j = 0; j < main_triangulation_boundaries[i].size(); j++)
//            if(main_triangulation_boundaries.at(i).at(j).unique())
//                main_triangulation_boundaries.at(i).at(j).reset();
//    for(unsigned int i = 0; i < arcs_points.size(); i++)
//        for(unsigned int j = 0; j < arcs_points[i].size(); j++)
//            if(arcs_points.at(i).at(j).unique())
//                arcs_points.at(i).at(j).reset();
//    std::cout << "Process completed" << std::endl;


    return 0;

}
