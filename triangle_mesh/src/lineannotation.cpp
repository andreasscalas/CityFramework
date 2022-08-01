#include "lineannotation.h"
#include <eigen3/Eigen/Dense>

using namespace std;

LineAnnotation::LineAnnotation()
{
    type = AnnotationType::Line;
}

LineAnnotation::LineAnnotation(TriangleMesh *mesh, std::shared_ptr<LineAnnotation> other) : Annotation(other)
{
    auto otherPolylines = other->getPolyLines();
    for(auto otherPolyline : otherPolylines)
    {
        std::vector<std::shared_ptr<Vertex> > polyline;
        for(auto v : otherPolyline)
            polyline.push_back(mesh->getVertex(v->getId()));
        polyLines.push_back(polyline);
    }
}

LineAnnotation::~LineAnnotation()
{
    polyLines.clear();
}

void LineAnnotation::print(ostream &os)
{
    Annotation::print(os);
    os << "type: Line" << std::endl << "PolyLines: [" << std::endl;
    for(unsigned int i = 0; i < polyLines.size(); i++){
        os << "[ " ;
        for(unsigned j = 0; j < polyLines[i].size(); j++){
            os << polyLines[i][j]->getId();
            if(j < polyLines[i].size() - 1)
                os << ", ";
        }
        os << " ]," << std::endl;
    }
    os << "]" << std::endl;
}

void LineAnnotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
    writer.StartObject();
    Annotation::printJson(writer);
    writer.Key("type");
    writer.String("Line");
    writer.Key("polylines");
    writer.StartArray();
    for(unsigned int i = 0; i < polyLines.size(); i++){
        writer.StartArray();
        for(unsigned j = 0; j < polyLines[i].size(); j++)
            writer.Int(stoi(polyLines[i][j]->getId()));
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
}

void LineAnnotation::addPolyLine(std::vector<std::shared_ptr<Vertex>> &value)
{
    this->polyLines.push_back(value);
}

std::vector<std::vector<std::shared_ptr<Vertex>> > LineAnnotation::getPolyLines() const
{
    return polyLines;
}

void LineAnnotation::setPolyLines(const std::vector<std::vector<std::shared_ptr<Vertex>> > &value)
{
    polyLines = value;
}

void LineAnnotation::clearPolylines()
{
    polyLines.clear();
}

std::vector<std::shared_ptr<Vertex>> LineAnnotation::getInvolvedVertices()
{
    vector<std::shared_ptr<Vertex>> vertices;
    for (unsigned int i = 0; i < polyLines.size(); i++)
        vertices.insert(vertices.end(), polyLines[i].begin(), polyLines[i].end());

    return vertices;
}

bool LineAnnotation::isPointInAnnotation(std::shared_ptr<Vertex>p)
{
    vector<std::shared_ptr<Vertex>> involved = getInvolvedVertices();
    for(unsigned int i = 0; i < involved.size(); i++){
        if(involved[i] == p)
            return true;
    }

    return false;
}


Point *LineAnnotation::getCenter()
{
    std::vector<std::shared_ptr<Vertex>> involvedVertices = getInvolvedVertices();
    Point* center = new Point(0,0,0);
    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        *center += *(involvedVertices[i]);
    }

    *center /= involvedVertices.size();

    return center;
}

Point *LineAnnotation::getOrientation()
{
    std::vector<std::shared_ptr<Vertex> > involvedVertices = getInvolvedVertices();
    Eigen::MatrixXd eigenPoints;
    eigenPoints.resize(3, involvedVertices.size());

    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        Eigen::Vector3d p = {involvedVertices[i]->getX(), involvedVertices[i]->getY(), involvedVertices[i]->getZ()};
        eigenPoints.col(i) = p;
    }

    Eigen::Vector3d mean_vector = eigenPoints.rowwise().mean();
    eigenPoints.colwise() -= mean_vector;
    Eigen::Matrix3d U = eigenPoints.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    Point* orientation = new Point();
    orientation->setX(U(0,2));
    orientation->setY(U(1,2));
    orientation->setZ(U(2,2));
    return orientation;
}
