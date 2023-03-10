#include "TriangleMesh.hpp"
#include "annotationfilemanager.h"
#include "pointannotation.hpp"
#include "lineannotation.hpp"
#include "semanticattribute.hpp"
#include "utilities.h"
#include <fstream>
#include <chrono>
#include <thread>
#include <map>
#include <root.h>
using namespace SemantisedTriangleMesh;
using namespace OpenStreetMap;

int main(int argc, char* argv[])
{
    if(argc < 4)
    {
        std::cerr << "Missing some parameters" << std::endl;
        return 1;
    }
    std::cout << "Reading mesh file." << std::endl << std::flush;
    std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
    int out1 = mesh->load(argv[1]);

    if(out1 != 0)
    {
        std::cerr << "Error loading mesh file. Error code: " << out1 << std::endl;
        return out1;
    }


    unsigned char street_color[] = {0, 0, 0};
    unsigned char green[] = {0, 255, 0};
    unsigned char yellowgreen[] = {210,255,105};
    unsigned char yellow[] = {255,255,0};
    unsigned char orange[] = {255,131,0};
    unsigned char salmon[] = {255,160,122};
    unsigned char red[] = {255, 0, 0};
    unsigned char violet[] = {155,38,182};
    double overall_min_slope = 0;
    double overall_max_slope = 0.261799;

    AnnotationFileManager manager;
    manager.setMesh(mesh);
    std::cout << "Reading annotations." << std::endl << std::flush;
    std::vector<std::shared_ptr<Annotation> > all_annotations = manager.readAndStoreAnnotations(argv[2]);
    std::cout << "Ended!" << std::endl << std::flush;

    std::cout << "Reading OSM file." << std::endl << std::flush;
    OpenStreetMap::Root osm;
    int out2 = osm.load(argv[3]);
    std::cout << "Ended!" << std::endl << std::flush;
    std::map<std::string, std::shared_ptr<Vertex> > osmid_to_vertex;

    if(out2 != 0)
    {
        std::cerr << "Error loading OSM file. Error code: " << out2 << std::endl;
        return out2;
    }

    uint counter = 0;

    std::cout << "Processing annotations: 0%" << std::flush;
    #pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < all_annotations.size(); i++)
    {
        if(all_annotations.at(i)->getType() == AnnotationType::Point)
        {
            std::shared_ptr<PointAnnotation> a = std::dynamic_pointer_cast<PointAnnotation>(all_annotations.at(i));
            std::vector<std::shared_ptr<Attribute> > attributes = a->getAttributes();
            if(a->getTag().find("node") != std::string::npos && a->getInvolvedVertices().size() == 1)
            {
                std::shared_ptr<Attribute>  att = *std::find_if(attributes.begin(), attributes.end(),
                        [](std::shared_ptr<Attribute> a){
                            return a->getKey().compare("osmid") == 0;
                        }
                );
                std::string osmid = *static_cast<std::string*>(att->getValue());
                osmid_to_vertex.insert(std::make_pair(osmid, a->getInvolvedVertices().at(0)));
            }
        }

        if(all_annotations.at(i)->getType() == AnnotationType::Line)
        {
            #pragma omp critical
            {
                mesh->addAnnotation(all_annotations.at(i));
            }
        }

        #pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / all_annotations.size() << "%\r";
        }
    }
    std::cout << std::endl << "Ended!" << std::endl << std::flush;
    all_annotations.clear();

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::shared_ptr<Annotation> > streets_annotations = mesh->getAnnotations();

    /*********************************Beginning JSON part*************************************/
    std::ofstream slopesFile("attributes.json");
    rapidjson::StringBuffer s_base;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s_base);
    s_base.Clear();
    writer.Reset(s_base);
    writer.StartObject();
    writer.Key("Arcs_attributes");
    writer.StartArray();
    /**********************************Ending JSON part**************************************/

    std::vector<std::shared_ptr<Annotation> > segments_slopes_annotations;
    std::vector<std::shared_ptr<Annotation> > streets_slopes_annotations;
    unsigned int segments_counter = 0, streets_counter = 0;
    std::cout << "Starting slopes computation: 0%" << std::flush;
    for(unsigned int i = 0; i < streets_annotations.size(); i++)
    {
        std::shared_ptr<LineAnnotation> street = std::dynamic_pointer_cast<LineAnnotation>(streets_annotations.at(i));
        std::vector<std::shared_ptr<Vertex> > full_path;
        double full_path_slope = 0;
        std::vector<std::shared_ptr<Attribute> > attributes = street->getAttributes();
        std::vector<std::shared_ptr<Attribute> >::iterator it = std::find_if(attributes.begin(), attributes.end(),
                [](std::shared_ptr<Attribute> a){
                    return a->getKey().compare("osmid") == 0;
                });

        if(it == attributes.end())
        {
            std::cerr << "Cannot find the OSMID of the street in the list of attributes. Annotation id: " << street->getId() << std::endl;
            return 49;
        }

        std::string osmid = *static_cast<std::string*>((*it)->getValue());
        std::vector<std::vector<std::shared_ptr<Vertex> > > polylines = street->getPolyLines();
        auto arc = std::find_if(osm.getWays().begin(), osm.getWays().end(),
                [osmid](std::pair<std::string, Way*> a){
                    return a.second->getId().compare(osmid) == 0;
                });

        if(arc == osm.getWays().end())
        {
            std::cerr << "One street annotation is not present in OSM." << std::endl;
            return 47;
        }
        if(polylines.size() > 1)
        {
            std::cerr << "One street can't have more than one polyline" << std::endl;
            return 48;
        }


        auto nodes = arc->second->getNodes();
        std::map<std::string, std::shared_ptr<Vertex> >::iterator vit1;
        unsigned int firstPos = 0;
        do
        {
            vit1 = osmid_to_vertex.find(nodes.at(firstPos++)->getId());
        }while(vit1 == osmid_to_vertex.end());
        firstPos--;

        full_path.push_back(vit1->second);
        for(unsigned int j = firstPos + 1; j < nodes.size(); j++)
        {
            std::map<std::string, std::shared_ptr<Vertex> >::iterator vit2 = osmid_to_vertex.find(nodes.at(j)->getId());
            if(vit2 == osmid_to_vertex.end())
                continue;
            std::shared_ptr<Vertex> v1 = vit1->second;
            std::shared_ptr<Vertex> v2 = vit2->second;
            std::vector<std::shared_ptr<Vertex> > path =
                    Utilities::dijkstra(v1, v2);
            double mean_slope = 0,
                   max_slope = -std::numeric_limits<double>::max(),
                   min_slope = std::numeric_limits<double>::max();

            /*********************************Beginning JSON part*************************************/
            writer.StartObject();
            writer.Key("Node1");
            writer.String(nodes.at(firstPos)->getId().c_str());
            writer.Key("Node2");
            writer.String(nodes.at(j)->getId().c_str());
            /**********************************Ending JSON part**************************************/

            std::shared_ptr<Vertex> p1 = v1;
            for(unsigned int l = 0; l < path.size(); l++)
            {
                unsigned char color[3] = {0,0,0};
                std::shared_ptr<Vertex> p2 = path.at(l);
                SemantisedTriangleMesh::Point v = (*p2) - (*p1);
                SemantisedTriangleMesh::Point projected(v.getX(), v.getY(), 0);
                v /= v.norm();

                double angle = M_PI / 2;
                if(v != SemantisedTriangleMesh::Point(0,0,1) && v != SemantisedTriangleMesh::Point(0,0,-1))
                {
                    projected /= projected.norm();
                    angle = v.computeAngle(projected);
                    if(p2->getZ() < p1->getZ())
                        angle *= -1;
                    if(angle < min_slope)
                        min_slope = angle;
                    if(angle > max_slope)
                        max_slope = angle;
                    mean_slope += angle;
                }

                angle = abs(angle);
                if(angle >= 0 && angle < 0.01745330748016422)
                {
                    color[0] = green[0];
                    color[1] = green[1];
                    color[2] = green[2];
                }else if(angle >= 0.01745330748016422 && angle < 0.034906615)
                {
                    color[0] = yellowgreen[0];
                    color[1] = yellowgreen[1];
                    color[2] = yellowgreen[2];
                }else if(angle >= 0.034906615 && angle < 0.06981323)
                {
                    color[0] = yellow[0];
                    color[1] = yellow[1];
                    color[2] = yellow[2];
                }else if(angle >= 0.06981323 && angle < 0.13962646)
                {
                    color[0] = orange[0];
                    color[1] = orange[1];
                    color[2] = orange[2];
                }else if(angle >= 0.13962646 && angle < 0.27925292)
                {
                    color[0] = salmon[0];
                    color[1] = salmon[1];
                    color[2] = salmon[2];
                } else
                {
                    color[0] = violet[0];
                    color[1] = violet[1];
                    color[2] = violet[2];
                }

                std::shared_ptr<LineAnnotation> segment = std::make_shared<LineAnnotation>();
                segment->setId(segments_counter);
                segment->setTag("street edge n° " + std::to_string(segments_counter++));
                segment->setColor(color);
                std::vector<std::shared_ptr<Vertex> > polyline;
                polyline.push_back(p1);
                polyline.push_back(p2);
                segment->addPolyLine(polyline);
                segment->setMesh(mesh);

                segments_slopes_annotations.push_back(segment);
                p1 = p2;

            }

            mean_slope /= path.size();
            if(path.size() == 0)
                std::cout << nodes.at(firstPos)->getId() << " " << nodes.at(j)->getId() << std::endl;

            vit1 = vit2;
            firstPos = j;
            writer.Key("Attributes");
            writer.StartArray();
            writer.StartObject();
            writer.Key("type");
            writer.String("min_slope");
            writer.Key("value");
            writer.Double(min_slope);
            writer.EndObject();
            writer.StartObject();
            writer.Key("type");
            writer.String("max_slope");
            writer.Key("value");
            writer.Double(max_slope);
            writer.EndObject();
            writer.StartObject();
            writer.Key("type");
            writer.String("mean_slope");
            writer.Key("value");
            writer.Double(mean_slope);
            writer.EndObject();
            writer.EndArray();
            writer.EndObject();

            full_path.insert(full_path.end(), path.begin(), path.end());
            full_path_slope += mean_slope;

        }

        unsigned char color[3] = {0,0,0};
        full_path_slope = abs(full_path_slope / (nodes.size() - 1));

        if(full_path_slope >= 0 && full_path_slope < 0.01745330748016422)
        {
            color[0] = green[0];
            color[1] = green[1];
            color[2] = green[2];
        }else if(full_path_slope >= 0.01745330748016422 && full_path_slope < 0.034906615)
        {
            color[0] = yellowgreen[0];
            color[1] = yellowgreen[1];
            color[2] = yellowgreen[2];
        }else if(full_path_slope >= 0.034906615 && full_path_slope < 0.06981323)
        {
            color[0] = yellow[0];
            color[1] = yellow[1];
            color[2] = yellow[2];
        }else if(full_path_slope >= 0.06981323 && full_path_slope < 0.13962646)
        {
            color[0] = orange[0];
            color[1] = orange[1];
            color[2] = orange[2];
        }else if(full_path_slope >= 0.13962646 && full_path_slope < 0.27925292)
        {
            color[0] = salmon[0];
            color[1] = salmon[1];
            color[2] = salmon[2];
        } else
        {
            color[0] = violet[0];
            color[1] = violet[1];
            color[2] = violet[2];
        }
        std::shared_ptr<LineAnnotation> street_annotation = std::make_shared<LineAnnotation>();
        street_annotation->setId(streets_counter);
        street_annotation->setTag("street edge n° " + std::to_string(streets_counter++));
        street_annotation->setColor(color);
        street_annotation->addPolyLine(polylines.at(0));
        street_annotation->setMesh(mesh);
        streets_slopes_annotations.push_back(street_annotation);

        std::cout << i * 100 / streets_annotations.size() << "%\r" <<std::flush;
    }
    /*********************************Beginning JSON part*************************************/
    writer.EndArray();
    writer.EndObject();
    if(slopesFile.is_open())
    {
        slopesFile << s_base.GetString();
        slopesFile.close();
    }
    /**********************************Ending JSON part**************************************/


    mesh->clearAnnotations();
    mesh->setAnnotations(segments_slopes_annotations);
    manager.writeAnnotations("slopes.ant");
    mesh->clearAnnotations();
    mesh->setAnnotations(streets_slopes_annotations);
    manager.writeAnnotations("averaged_slopes.ant");

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "The computation took " << duration.count() << "seconds" << std::endl << std::flush;
    std::cout << "Ended!" << std::endl;
    return 0;
}
