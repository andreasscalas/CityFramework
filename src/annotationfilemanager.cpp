#include "annotationfilemanager.h"
#include "facet.h"
#include "geometricattribute.h"
#include "semanticattribute.h"
#include "surfaceannotation.h"
#include "lineannotation.h"
#include "pointannotation.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <rapidjson/writer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

using namespace std;
using namespace SemantisedTriangleMesh;

AnnotationFileManager::AnnotationFileManager()
{
    mesh = nullptr;
}

bool AnnotationFileManager::writeAnnotations(std::string fileName)
{
    if(mesh != nullptr){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("ant") == 0){
            vector<std::shared_ptr<Annotation> > annotations = mesh->getAnnotations();
            ofstream annotationsFile;
            rapidjson::StringBuffer s;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
            writer.StartObject();
            writer.Key("annotations");
            writer.StartArray();
            for(unsigned int i = 0; i < annotations.size(); i++)
                annotations[i]->printJson(writer);
            writer.EndArray();
            writer.EndObject();
            annotationsFile.open(fileName);
            if(annotationsFile.is_open())
            {
                annotationsFile << s.GetString();
                annotationsFile.close();
            }
        } else if(extension.compare("triant") == 0){
            vector<std::shared_ptr<Annotation>> annotations = mesh->getAnnotations();
            ofstream annotationsFile;
            rapidjson::StringBuffer s;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
            writer.StartObject();
            writer.Key("annotations");
            writer.StartArray();
            vector<std::shared_ptr<Vertex>> lst;
            for(unsigned int i = 0; i < annotations.size(); i++){
                writer.StartObject();
                std::shared_ptr<SurfaceAnnotation> a = dynamic_pointer_cast<SurfaceAnnotation>(annotations[i]);
                writer.Key("id");
                writer.Uint(a->getId());
                writer.Key("tag");
                writer.String(a->getTag().c_str());
//                writer.Key("level");
//                writer.Uint(a->getHierarchyLevel());
                writer.Key("color");
                writer.StartArray();
                writer.Int(a->getColor()[0]);
                writer.Int(a->getColor()[1]);
                writer.Int(a->getColor()[2]);
                writer.EndArray();
                writer.Key("triangles");

                std::vector<std::shared_ptr<Triangle>> triangles = a->getTriangles();
                writer.StartArray();
                for(unsigned int i = 0; i < triangles.size(); i++){
                    writer.String(triangles[i]->getId().c_str());
                }
                writer.EndArray();
                writer.EndObject();
            }
            writer.EndArray();
            writer.EndObject();
            annotationsFile.open(fileName);
            annotationsFile << s.GetString();
            annotationsFile.close();
        } else if(extension.compare("m") == 0){
            ofstream matrixFile;
            matrixFile.open(fileName);
            for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
            {
                std::shared_ptr<Annotation> a = mesh->getAnnotations()[i];
                std::vector<std::shared_ptr<Vertex>> involvedVertices;
                std::vector<int> isBoundary;
                switch (a->getType()) {
                    case AnnotationType::Point:
                    {
                        std::shared_ptr<PointAnnotation> p = dynamic_pointer_cast<PointAnnotation>(a);
                        involvedVertices = p->getInvolvedVertices();
                        isBoundary.resize(involvedVertices.size());
                        fill(isBoundary.begin(), isBoundary.end(), 1);
                        break;
                    }
                    case AnnotationType::Line:
                    {
                        std::shared_ptr<LineAnnotation> l = dynamic_pointer_cast<LineAnnotation>(a);
                        involvedVertices = l->getInvolvedVertices();
                        isBoundary.resize(involvedVertices.size());
                        fill(isBoundary.begin(), isBoundary.end(), 1);
                        break;
                    }
                    case AnnotationType::Surface:
                    {
                        std::shared_ptr<SurfaceAnnotation> s = dynamic_pointer_cast<SurfaceAnnotation>(a);
                        involvedVertices = s->getInvolvedVertices();
                        break;
                    }
                    default:
                        exit(11);

                }
                matrixFile << "Mesh: [";
                for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
                {
                    std::shared_ptr<Vertex> v = mesh->getVertex(i);
                    matrixFile << "\t" << v->getX() << " " << v->getY() << " " << v->getZ() << std::endl;
                }
                matrixFile << "];" << std::endl << std::endl;
                matrixFile << "A" << i << ":" << std::endl << "[" << std::endl << "\t";
                for(unsigned int j = 0; j < involvedVertices.size(); j++)
                {
                    matrixFile << "\t" << involvedVertices[j]->getId() << std::endl;
                }
                matrixFile << "]" << std::endl << "B" << i << ":" << std::endl << "[" << std::endl;
                for(unsigned int j = 0; j < isBoundary.size(); j++)
                {
                    matrixFile <<  "\t" << isBoundary[j] << std::endl;
                }
                matrixFile << "]" << std::endl << std::endl;
            }
        } else
            return false;
        return true;
    }else
        return false;
}

bool AnnotationFileManager::readAnnotations(string fileName)
{

    if(mesh != nullptr && mesh->getVerticesNumber()){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("ant") == 0){
            FILE* fp = fopen(fileName.c_str(),"r");
            if(fp != nullptr)
            {
                char buffer[BUFFER_SIZE];
                rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

                rapidjson::Document document;
                if(!(document.ParseStream(frs).HasParseError())){
                    if(document.HasMember("annotations") && document["annotations"].IsArray()){
                        rapidjson::Value& annotationsList = document["annotations"];
                        for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                        {
                            rapidjson::Value& jsonAnnotation = annotationsList[i];
                            if(jsonAnnotation.IsObject()){
                                assert(jsonAnnotation.HasMember("type"));
                                assert(jsonAnnotation["type"].IsString());
                                string type = jsonAnnotation["type"].GetString();

                                std::shared_ptr<Annotation> annotation;

                                if(type.compare("Area") == 0)
                                    annotation = std::make_shared<SurfaceAnnotation>();
                                else if(type.compare("Line") == 0)
                                    annotation = std::make_shared<LineAnnotation>();
                                else if(type.compare("Point") == 0)
                                    annotation = std::make_shared<PointAnnotation>();
                                else
                                    return false;

                                annotation->setMesh(mesh);
                                assert(jsonAnnotation.HasMember("id"));
                                assert(jsonAnnotation["id"].IsUint());
                                annotation->setId(jsonAnnotation["id"].GetUint());

                                assert(jsonAnnotation.HasMember("tag"));
                                assert(jsonAnnotation["tag"].IsString());
                                annotation->setTag(jsonAnnotation["tag"].GetString());

    //                            assert(jsonAnnotation.HasMember("level"));
    //                            assert(jsonAnnotation["level"].IsUint());
    //                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                                unsigned char color[3];
                                if(jsonAnnotation.HasMember("color")){
                                    rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                    assert(jsonColor.IsArray());
                                    assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                    color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                    color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                    color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                                }else {
                                    color[0] = 0;
                                    color[1] = 0;
                                    color[2] = 0;
                                }

                                annotation->setColor(color);

                                if(type.compare("Area") == 0){
                                    assert(jsonAnnotation.HasMember("boundaries"));
                                    assert(jsonAnnotation["boundaries"].IsArray());
                                    rapidjson::Value& boundaries = jsonAnnotation["boundaries"];
                                    for(rapidjson::SizeType j = 0; j < boundaries.Size(); j++){
                                        rapidjson::Value& boundary = boundaries[j];
                                        vector<std::shared_ptr<Vertex>> outline;
                                        assert(boundary.IsArray());
                                        for(rapidjson::SizeType k = 0; k < boundary.Size(); k++){
                                            rapidjson::Value& pointID = boundary[k];
                                            assert(pointID.IsInt());
                                            std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                            outline.push_back(v);
                                        }
                                        dynamic_pointer_cast<SurfaceAnnotation>(annotation)->addOutline(outline);
                                    }

                                } else if(type.compare("Line") == 0){
                                    assert(jsonAnnotation.HasMember("polylines"));
                                    assert(jsonAnnotation["polylines"].IsArray());
                                    rapidjson::Value& polyLines = jsonAnnotation["polylines"];
                                    for(rapidjson::SizeType j = 0; j < polyLines.Size(); j++){
                                        rapidjson::Value& jsonPolyLine = polyLines[j];
                                        vector<std::shared_ptr<Vertex>> polyLine;
                                        assert(jsonPolyLine.IsArray());
                                        for(rapidjson::SizeType k = 0; k < jsonPolyLine.Size(); k++){
                                            rapidjson::Value& pointID = jsonPolyLine[k];
                                            assert(pointID.IsInt());
                                            std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                            polyLine.push_back(v);
                                        }
                                        dynamic_pointer_cast<LineAnnotation>(annotation)->addPolyLine(polyLine);
                                    }
                                } else if(type.compare("Point") == 0){
                                    assert(jsonAnnotation.HasMember("points"));
                                    assert(jsonAnnotation["points"].IsArray());
                                    rapidjson::Value& points = jsonAnnotation["points"];
                                    for(rapidjson::SizeType j = 0; j < points.Size(); j++){
                                        rapidjson::Value& pointID = points[j];
                                        assert(pointID.IsInt());
                                        std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                        dynamic_pointer_cast<PointAnnotation>(annotation)->addPoint(v);
                                    }

                                }

                                assert(jsonAnnotation.HasMember("attributes"));
                                assert(jsonAnnotation["attributes"].IsArray());
                                rapidjson::Value& attributes = jsonAnnotation["attributes"];

                                for(rapidjson::SizeType j = 0; j < attributes.Size(); j++){
                                    rapidjson::Value& jsonAttribute = attributes[j];
                                    assert(jsonAttribute.IsObject());
                                    assert(jsonAttribute.HasMember("id"));
                                    assert(jsonAttribute.HasMember("type"));
                                    assert(jsonAttribute.HasMember("name"));
                                    assert(jsonAttribute.HasMember("value"));
                                    assert(jsonAttribute["type"].IsString());
                                    assert(jsonAttribute["name"].IsString());
                                    unsigned int attributeID = jsonAttribute["id"].GetUint();
                                    string attributeType = jsonAttribute["type"].GetString();
                                    string attributeName = jsonAttribute["name"].GetString();
                                    std::shared_ptr<Attribute> attribute;

                                    if(attributeType.compare("Geometric") == 0){
                                        assert(false);
                                    } else if(attributeType.compare("Semantic") == 0){
                                        attribute = std::make_shared<SemanticAttribute>();
                                        string value = jsonAttribute["value"].GetString();
                                        attribute->setValue(value);

                                    } else {
                                        assert(false);
                                    }

                                    attribute->setId(attributeID);
                                    attribute->setKey(attributeName);
                                    annotation->addAttribute(attribute);

                                }

                                annotation->setMesh(mesh);
                                mesh->addAnnotation(annotation);
                            }else
                                return false;
                        }
                    }else
                        return false;
                }else
                    return false;
            } else return false;
        } else if (extension.compare("triant") == 0){

            FILE* fp = fopen(fileName.c_str(),"r");
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                if(document.HasMember("annotations") && document["annotations"].IsArray()){
                    rapidjson::Value& annotationsList = document["annotations"];
                    for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& jsonAnnotation = annotationsList[i];
                        if(jsonAnnotation.IsObject()){

                            std::shared_ptr<SurfaceAnnotation> annotation = std::make_shared<SurfaceAnnotation>();
                            assert(jsonAnnotation.HasMember("tag"));
                            assert(jsonAnnotation["tag"].IsString());
                            annotation->setTag(jsonAnnotation["tag"].GetString());

//                            assert(jsonAnnotation.HasMember("level"));
//                            assert(jsonAnnotation["level"].IsUint());
//                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                            unsigned char color[3];
                            if(jsonAnnotation.HasMember("color")){
                                rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                assert(jsonColor.IsArray());
                                assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                            }else {
                                color[0] = 0;
                                color[1] = 0;
                                color[2] = 0;
                            }

                            annotation->setColor(color);

                            assert(jsonAnnotation.HasMember("triangles"));
                            assert(jsonAnnotation["triangles"].IsArray());
                            rapidjson::Value& jsonTriangles = jsonAnnotation["triangles"];
                            std::vector<::std::shared_ptr<Triangle>> triangles;
                            for(rapidjson::SizeType j = 0; j < jsonTriangles.Size(); j++){
                                ::std::shared_ptr<Vertex> v = mesh->getTriangle(jsonTriangles[j].GetInt())->getV1();
                                for(unsigned int i = 0; i < 3; i++)
                                    v = mesh->getTriangle(jsonTriangles[j].GetInt())->getNextVertex(v);
                                triangles.push_back(mesh->getTriangle(jsonTriangles[j].GetInt()));
                            }

                            annotation->setOutlines(SurfaceAnnotation::getOutlines(triangles));

                            annotation->setMesh(mesh);
                            mesh->addAnnotation(annotation);
                        }else
                            return false;
                    }
                }else
                    return false;
            }else
                return false;
        }
    }else
        return false;

    return true;
}

std::vector<std::shared_ptr<Annotation> > AnnotationFileManager::readAndStoreAnnotations(string fileName)
{
    std::vector<std::shared_ptr<Annotation> > annotations;
    if(mesh != nullptr && mesh->getVerticesNumber()){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("ant") == 0){
            FILE* fp = fopen(fileName.c_str(),"r");
            if(fp != nullptr)
            {
                char buffer[BUFFER_SIZE];
                rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

                rapidjson::Document document;
                if(!(document.ParseStream(frs).HasParseError())){
                    if(document.HasMember("annotations") && document["annotations"].IsArray()){
                        rapidjson::Value& annotationsList = document["annotations"];
                        for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                        {
                            rapidjson::Value& jsonAnnotation = annotationsList[i];
                            if(jsonAnnotation.IsObject()){
                                assert(jsonAnnotation.HasMember("type"));
                                assert(jsonAnnotation["type"].IsString());
                                string type = jsonAnnotation["type"].GetString();

                                std::shared_ptr<Annotation> annotation;

                                if(type.compare("Area") == 0)
                                    annotation = std::make_shared<SurfaceAnnotation>();
                                else if(type.compare("Line") == 0)
                                    annotation = std::make_shared<LineAnnotation>();
                                else if(type.compare("Point") == 0)
                                    annotation = std::make_shared<PointAnnotation>();
                                else
                                {
                                    annotations.clear();
                                    return annotations;
                                }

                                annotation->setMesh(mesh);
                                assert(jsonAnnotation.HasMember("id"));
                                assert(jsonAnnotation["id"].IsUint());
                                annotation->setId(jsonAnnotation["id"].GetUint());

                                assert(jsonAnnotation.HasMember("tag"));
                                assert(jsonAnnotation["tag"].IsString());
                                annotation->setTag(jsonAnnotation["tag"].GetString());

    //                            assert(jsonAnnotation.HasMember("level"));
    //                            assert(jsonAnnotation["level"].IsUint());
    //                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                                unsigned char color[3];
                                if(jsonAnnotation.HasMember("color")){
                                    rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                    assert(jsonColor.IsArray());
                                    assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                    color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                    color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                    color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                                }else {
                                    color[0] = 0;
                                    color[1] = 0;
                                    color[2] = 0;
                                }

                                annotation->setColor(color);

                                if(type.compare("Area") == 0){
                                    assert(jsonAnnotation.HasMember("boundaries"));
                                    assert(jsonAnnotation["boundaries"].IsArray());
                                    rapidjson::Value& boundaries = jsonAnnotation["boundaries"];
                                    for(rapidjson::SizeType j = 0; j < boundaries.Size(); j++){
                                        rapidjson::Value& boundary = boundaries[j];
                                        vector<std::shared_ptr<Vertex>> outline;
                                        assert(boundary.IsArray());
                                        for(rapidjson::SizeType k = 0; k < boundary.Size(); k++){
                                            rapidjson::Value& pointID = boundary[k];
                                            assert(pointID.IsInt());
                                            std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                            outline.push_back(v);
                                        }
                                        dynamic_pointer_cast<SurfaceAnnotation>(annotation)->addOutline(outline);
                                    }

                                } else if(type.compare("Line") == 0){
                                    assert(jsonAnnotation.HasMember("polylines"));
                                    assert(jsonAnnotation["polylines"].IsArray());
                                    rapidjson::Value& polyLines = jsonAnnotation["polylines"];
                                    for(rapidjson::SizeType j = 0; j < polyLines.Size(); j++){
                                        rapidjson::Value& jsonPolyLine = polyLines[j];
                                        vector<std::shared_ptr<Vertex>> polyLine;
                                        assert(jsonPolyLine.IsArray());
                                        for(rapidjson::SizeType k = 0; k < jsonPolyLine.Size(); k++){
                                            rapidjson::Value& pointID = jsonPolyLine[k];
                                            assert(pointID.IsInt());
                                            std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                            polyLine.push_back(v);
                                        }
                                        dynamic_pointer_cast<LineAnnotation>(annotation)->addPolyLine(polyLine);
                                    }
                                } else if(type.compare("Point") == 0){
                                    assert(jsonAnnotation.HasMember("points"));
                                    assert(jsonAnnotation["points"].IsArray());
                                    rapidjson::Value& points = jsonAnnotation["points"];
                                    for(rapidjson::SizeType j = 0; j < points.Size(); j++){
                                        rapidjson::Value& pointID = points[j];
                                        assert(pointID.IsInt());
                                        std::shared_ptr<Vertex> v = mesh->getVertex(static_cast<unsigned long>(pointID.GetInt()));
                                        dynamic_pointer_cast<PointAnnotation>(annotation)->addPoint(v);
                                    }

                                }

                                assert(jsonAnnotation.HasMember("attributes"));
                                assert(jsonAnnotation["attributes"].IsArray());
                                rapidjson::Value& attributes = jsonAnnotation["attributes"];

                                for(rapidjson::SizeType j = 0; j < attributes.Size(); j++){
                                    rapidjson::Value& jsonAttribute = attributes[j];
                                    assert(jsonAttribute.IsObject());
                                    assert(jsonAttribute.HasMember("id"));
                                    assert(jsonAttribute.HasMember("type"));
                                    assert(jsonAttribute.HasMember("name"));
                                    assert(jsonAttribute.HasMember("value"));
                                    assert(jsonAttribute["type"].IsString());
                                    assert(jsonAttribute["name"].IsString());
                                    unsigned int attributeID = jsonAttribute["id"].GetUint();
                                    string attributeType = jsonAttribute["type"].GetString();
                                    string attributeName = jsonAttribute["name"].GetString();
                                    std::shared_ptr<Attribute> attribute;

                                    if(attributeType.compare("Geometric") == 0){
                                        assert(false);
                                    } else if(attributeType.compare("Semantic") == 0){
                                        attribute = std::make_shared<SemanticAttribute>();
                                        string value = jsonAttribute["value"].GetString();
                                        attribute->setValue(value);

                                    } else {
                                        assert(false);
                                    }

                                    attribute->setId(attributeID);
                                    attribute->setKey(attributeName);
                                    annotation->addAttribute(attribute);

                                }
                                annotation->setMesh(mesh);
                                annotations.push_back(annotation);
                            }else
                            {
                                annotations.clear();
                                return annotations;
                            }
                        }
                    }
                }
            }
        } else if (extension.compare("triant") == 0){

            FILE* fp = fopen(fileName.c_str(),"r");
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                if(document.HasMember("annotations") && document["annotations"].IsArray()){
                    rapidjson::Value& annotationsList = document["annotations"];
                    for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& jsonAnnotation = annotationsList[i];
                        if(jsonAnnotation.IsObject()){

                            std::shared_ptr<SurfaceAnnotation> annotation = std::make_shared<SurfaceAnnotation>();
                            assert(jsonAnnotation.HasMember("tag"));
                            assert(jsonAnnotation["tag"].IsString());
                            annotation->setTag(jsonAnnotation["tag"].GetString());

//                            assert(jsonAnnotation.HasMember("level"));
//                            assert(jsonAnnotation["level"].IsUint());
//                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                            unsigned char color[3];
                            if(jsonAnnotation.HasMember("color")){
                                rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                assert(jsonColor.IsArray());
                                assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                            }else {
                                color[0] = 0;
                                color[1] = 0;
                                color[2] = 0;
                            }

                            annotation->setColor(color);

                            assert(jsonAnnotation.HasMember("triangles"));
                            assert(jsonAnnotation["triangles"].IsArray());
                            rapidjson::Value& jsonTriangles = jsonAnnotation["triangles"];
                            std::vector<::std::shared_ptr<Triangle>> triangles;
                            for(rapidjson::SizeType j = 0; j < jsonTriangles.Size(); j++){
                                ::std::shared_ptr<Vertex> v = mesh->getTriangle(jsonTriangles[j].GetInt())->getV1();
                                for(unsigned int i = 0; i < 3; i++)
                                    v = mesh->getTriangle(jsonTriangles[j].GetInt())->getNextVertex(v);
                                triangles.push_back(mesh->getTriangle(jsonTriangles[j].GetInt()));
                            }

                            annotation->setOutlines(SurfaceAnnotation::getOutlines(triangles));

                            annotation->setMesh(mesh);
                            annotations.push_back(annotation);
                        }else
                        {
                            annotations.clear();
                            return annotations;
                        }
                    }
                }
            }
        }
    }

    return annotations;
}

std::shared_ptr<TriangleMesh> AnnotationFileManager::getMesh() const
{
    return mesh;
}

void AnnotationFileManager::setMesh(std::shared_ptr<TriangleMesh> value)
{
    mesh = value;
}

