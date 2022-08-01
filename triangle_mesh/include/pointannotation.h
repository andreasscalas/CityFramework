#ifndef POINTANNOTATION_H
#define POINTANNOTATION_H

#include <annotation.h>
#include <TriangleMesh.h>


class PointAnnotation : virtual public Annotation
{
public:
    PointAnnotation();
    PointAnnotation(TriangleMesh *mesh, std::shared_ptr<PointAnnotation> other);

    virtual ~PointAnnotation() override;

    virtual void print(std::ostream&) override;

    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    virtual std::vector<std::shared_ptr<Vertex>> getInvolvedVertices() override;

    virtual bool isPointInAnnotation(std::shared_ptr<Vertex> annotatedPoint) override;

    std::vector<std::shared_ptr<Vertex>> getPoints() const;
    void setPoints(const std::vector<std::shared_ptr<Vertex>> &value);
    void addPoint(std::shared_ptr<Vertex> value);

    virtual Point* getCenter() override;
    virtual Point* getOrientation() override;


protected:
    std::vector<std::shared_ptr<Vertex> > points;

};

#endif // POINTANNOTATION_H
