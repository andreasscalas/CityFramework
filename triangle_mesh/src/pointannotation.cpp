#include "pointannotation.h"
#include <eigen3/Eigen/Dense>

PointAnnotation::PointAnnotation()
{
    type = AnnotationType::Point;
}

PointAnnotation::PointAnnotation(TriangleMesh *mesh, std::shared_ptr<PointAnnotation> other) : Annotation(other)
{
    auto otherPoints = other->getPoints();
    for(auto v : otherPoints)
        points.push_back(mesh->getVertex(v->getId()));
}

PointAnnotation::~PointAnnotation()
{
    this->points.clear();
}


void PointAnnotation::print(std::ostream &os)
{
    Annotation::print(os);
    os << "Annotation on points: [" << std::endl;
    for(unsigned int i = 0; i < points.size(); i++){
        os << "(" << points[i]->getX() << "," << points[i]->getY() << "," << points[i]->getZ() << ")";
        if(i != points.size() - 1)
            os <<",";
        os << std::endl;
    }
}

void PointAnnotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    writer.StartObject();
    Annotation::printJson(writer);
    writer.Key("type");
    writer.String("Point");
    writer.Key("points");
    writer.StartArray();
    for(unsigned int i = 0; i < points.size(); i++)
        writer.Int(stoi(points[i]->getId()));
    writer.EndArray();
    writer.EndObject();
}

std::vector<std::shared_ptr<Vertex>> PointAnnotation::getInvolvedVertices()
{
    return points;
}

bool PointAnnotation::isPointInAnnotation(std::shared_ptr<Vertex>v)
{
    for(unsigned int i = 0; i < points.size(); i++)
        if(points[i] == v)
            return true;
    return false;
}

std::vector<std::shared_ptr<Vertex>> PointAnnotation::getPoints() const
{
    return points;
}

void PointAnnotation::setPoints(const std::vector<std::shared_ptr<Vertex>> &value)
{
    points = value;
}

void PointAnnotation::addPoint(std::shared_ptr<Vertex>value)
{
    this->points.push_back(value);
}

Point *PointAnnotation::getCenter()
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

Point *PointAnnotation::getOrientation()
{
    std::vector<std::shared_ptr<Vertex>> involvedVertices = getInvolvedVertices();
    Eigen::MatrixXd eigenPoints;
    eigenPoints.resize(3, static_cast<Eigen::Index>(involvedVertices.size()));

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

