#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <semanticsfilemanager.hpp>
#include <utilities.h>
#include <graph.h>
#include <lineannotation.hpp>
#include <pointannotation.hpp>
#include <semanticattribute.hpp>
#include <map>

using namespace SemantisedTriangleMesh;
static const int BUFFER_SIZE = 65536;

std::vector<std::vector<ulong> > readItineraries(std::string filename);
std::vector<ulong> readBestPath(std::string filename);

int main(int argc, char* argv[])
{
    if(argc < 5)
    {
        std::cout << "Missing some parameters" << std::endl;
    }

    bool orienteering = strcmp(argv[1], "orienteering") == 0;
    auto mesh = std::make_shared<TriangleMesh>();
    mesh->load(argv[2]);
    SemanticsFileManager manager;
    manager.setMesh(mesh);
    manager.readAnnotations(argv[3]);
    std::vector<std::vector<ulong> >  itineraries;
    if(orienteering)
        itineraries = readItineraries(argv[4]);
    else
    {
        auto itinerary = readBestPath(argv[4]);
        itineraries.push_back(itinerary);
    }


    GraphTemplate::Graph<std::shared_ptr<Annotation> > graph;
    std::map<std::shared_ptr<Vertex>, GraphTemplate::Node<std::shared_ptr<Annotation> > *> vertexToNode;
    std::map<ulong, ulong> osmidToAnnotation;
    auto annotations = mesh->getAnnotations();
    std::vector<std::shared_ptr<LineAnnotation> > streets;
    uint counter = 0;
    for(auto annotation : annotations)
    {
        if(annotation->getType() == AnnotationType::Point)
        {
            auto pointAnnotation = std::dynamic_pointer_cast<PointAnnotation>(annotation);
            if(pointAnnotation->getPoints().size() == 1)
            {
                auto attributes = pointAnnotation->getAttributes();
                for(auto attribute : attributes)
                {
                    if(attribute->getKey().compare("osmid") == 0)
                    {
                        auto osmid = *static_cast<std::string*>(attribute->getValue());
                        std::stringstream stream(osmid);
                        ulong osmidLong;
                        stream >> osmidLong;
                        osmidToAnnotation.insert(std::make_pair(osmidLong, counter));
                        auto n = new GraphTemplate::Node<std::shared_ptr<Annotation> >();
                        n->setData(pointAnnotation);
                        n->clearArcs();
                        graph.addNode(n);
                        vertexToNode.insert(std::make_pair(pointAnnotation->getPoints()[0], n));
                    }
                }
            }
        } else if(annotation->getType() == AnnotationType::Line)
        {
            auto tag = annotation->getTag();
            std::transform(tag.begin(), tag.end(), tag.begin(), [](unsigned char c) { return std::tolower(c); });
            if(tag.find("street") != std::string::npos)
                streets.push_back(std::dynamic_pointer_cast<LineAnnotation>(annotation));
        }
        counter++;
    }

    uint reachedId = 0;
    for(auto street : streets)
    {
        auto polylines = street->getPolyLines();
        for(auto polyline : polylines)
        {
            for(uint i = 1; i < polyline.size(); i++)
            {
                std::map<std::shared_ptr<Vertex>, GraphTemplate::Node<std::shared_ptr<Annotation> >* >::iterator it = vertexToNode.find(polyline.at(i - 1));
                while (it == vertexToNode.end())
                {
                    i++;
                    it = vertexToNode.find(polyline.at(i - 1));
                }

                auto n1 = it->second;
                it = vertexToNode.find(polyline.at(i));
                while (it == vertexToNode.end())
                {
                    i++;
                    it = vertexToNode.find(polyline.at(i));
                }
                auto n2 = it->second;
                auto a = new GraphTemplate::Arc<std::shared_ptr<Annotation> >();
                a->setId(reachedId++);
                a->setDirected(false);
                a->setWeight(1);
                a->setN1(n1);
                a->setN2(n2);
                std::stringstream stream;
                stream << street->getTag() << "_segment" << i;
                a->setLabel(stream.str());
                graph.addArc(a);
            }
        }
    }

    mesh->clearAnnotations();
    std::vector<std::shared_ptr<LineAnnotation> > routes;
    for(uint i = 0; i < itineraries.size(); i++)
    {
        std::shared_ptr<LineAnnotation> annotation = std::make_shared<LineAnnotation>();
        annotation->setId(std::to_string(i));
        annotation->setTag("itinerary_" + std::to_string(i));
        auto startingID = itineraries.at(i).at(0);
        auto closingID = itineraries.at(i).back();
        auto attr1 = std::make_shared<SemanticAttribute>();
        attr1->setId(0);
        attr1->setKey("start_osmid");
        attr1->setValue(std::to_string(startingID));
        annotation->addAttribute(attr1);
        auto attr2 = std::make_shared<SemanticAttribute>();
        attr2->setId(1);
        attr2->setKey("end_osmid");
        attr2->setValue(std::to_string(closingID));
        annotation->addAttribute(attr2);
        std::vector<std::shared_ptr<Vertex> >  polyline;
        std::shared_ptr<Vertex> prev = nullptr;
        for(uint j = 0; j < itineraries.at(i).size(); j++)
        {

            auto osmid = itineraries.at(i).at(j);
            if(osmidToAnnotation.find(osmid) != osmidToAnnotation.end())
            {
                ulong pos = osmidToAnnotation.at(osmid);
                auto pointAnnotation = annotations.at(pos);
                auto vid = std::dynamic_pointer_cast<PointAnnotation>(pointAnnotation)->getPoints()[0]->getId();
                auto v = mesh->getVertex(vid);
                if(prev == nullptr)
                {
                    polyline.push_back(v);
                    auto n2 = vertexToNode.at(v);
                    mesh->addAnnotation(n2->getData());
                }
                else
                {

                    auto n1 = vertexToNode.at(prev);
                    auto n2 = vertexToNode.at(v);
                    mesh->addAnnotation(n2->getData());

                    auto graphPath = graph.shortestPathSearch(n1, n2);
                    for(auto a : graphPath)
                    {
                        auto pa1 = std::dynamic_pointer_cast<PointAnnotation>(a->getN1()->getData());
                        auto pa2 = std::dynamic_pointer_cast<PointAnnotation>(a->getN2()->getData());
                        auto v1 = pa1->getPoints()[0];
                        auto v2 = pa2->getPoints()[0];
//                        auto path = Utilities::dijkstra(v1, v2);
//                        polyline.insert(polyline.end(), path.begin(), path.end());
                        polyline.push_back(v2);
                    }
                }
                prev = v;
            } else
            {
                std::cout << "Itinerary passes through node which is not present in the 3D model" << std::endl;
                std::cout << "OSMID of the not found node: " << osmid << std::endl;
                exit(18);
            }
        }
        annotation->addPolyLine(polyline);
        unsigned char color[3] = {0, 255, 0};
        annotation->setColor(color);
        annotation->setTag("best_route");
        mesh->addAnnotation(annotation);

    }
    for(auto a : graph.getArcs())
    {
        graph.removeArc(a);
        delete a;
    }
    for(auto n : graph.getNodes())
    {
        graph.removeNode(n);
        delete n;
    }

    manager.writeAnnotations("routes.ant");
    mesh.reset();

}

std::vector<std::vector<ulong> > readItineraries(std::string filename)
{
    std::vector<ulong> pois;
    std::vector<std::vector<ulong> > itineraries;

    std::string extension = filename.substr(filename.find_last_of(".") + 1);
    if(extension.compare("json") == 0){
        FILE* fp = fopen(filename.c_str(),"r");
        if(fp != nullptr)
        {
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                if(document.HasMember("OSMID") && document["OSMID"].IsArray())
                {
                    rapidjson::Value& poiList = document["OSMID"];
                    for (rapidjson::SizeType i = 0; i < poiList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& osmid_object = poiList[i];

                        ulong osmid = osmid_object.GetDouble();
                        pois.push_back(osmid);


                    }
                } else
                {
                    itineraries.clear();
                    return itineraries;
                }

                if(document.HasMember("OSMIDPATH") && document["OSMIDPATH"].IsArray())
                {

                    rapidjson::Value& itineraryList = document["OSMIDPATH"];
                    for (rapidjson::SizeType i = 0; i < itineraryList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& itinerary_object = itineraryList[i];
                        std::vector<ulong> itinerary;
                        if(itinerary_object.IsArray()){
                            for (rapidjson::SizeType j = 0; j < itinerary_object.Size(); j++) // rapidjson uses SizeType instead of size_t.
                            {

                                rapidjson::Value& osmid_object = itinerary_object[j];
                                ulong osmid = osmid_object.GetDouble();
                                itinerary.push_back(osmid);
                            }
                        }
                        itineraries.push_back(itinerary);

                    }

                } else
                {
                    itineraries.clear();
                    return itineraries;
                }
            } else
            {
                itineraries.clear();
                return itineraries;
            }
        } else
        {
            itineraries.clear();
            return itineraries;
        }
    } else
    {
        itineraries.clear();
        return itineraries;
    }

    return itineraries;
}


std::vector<ulong> readBestPath(std::string filename)
{
    std::vector<ulong> itinerary;

    std::string extension = filename.substr(filename.find_last_of(".") + 1);
    if(extension.compare("json") == 0){
        FILE* fp = fopen(filename.c_str(),"r");
        if(fp != nullptr)
        {
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                auto list = document.GetArray();
                for (rapidjson::SizeType i = 0; i < list.Size(); i++) // rapidjson uses SizeType instead of size_t.
                {
                    rapidjson::Value& osmid_object = list[i];
                    ulong osmid = osmid_object.GetDouble();
                    itinerary.push_back(osmid);
                }
            } else
            {
                itinerary.clear();
                return itinerary;
            }
        } else
        {
            itinerary.clear();
            return itinerary;
        }
    }
    return itinerary;
}
