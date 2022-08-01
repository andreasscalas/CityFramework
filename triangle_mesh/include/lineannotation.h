#ifndef LINEANNOTATION_H
#define LINEANNOTATION_H

#include <vector>
#include <annotation.h>
#include <Vertex.h>

class LineAnnotation : virtual public Annotation
{
public:
    LineAnnotation();
    LineAnnotation(TriangleMesh*, std::shared_ptr<LineAnnotation> other);

    virtual ~LineAnnotation() override;


    virtual void print (std::ostream&) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    virtual std::vector<std::shared_ptr<Vertex> > getInvolvedVertices() override;
    virtual bool isPointInAnnotation(std::shared_ptr<Vertex> p) override;

    void addPolyLine(std::vector<std::shared_ptr<Vertex> > &value);
    std::vector<std::vector<std::shared_ptr<Vertex> > > getPolyLines() const;
    void setPolyLines(const std::vector<std::vector<std::shared_ptr<Vertex> > > &value);
    void clearPolylines();

    virtual Point* getCenter() override;
    virtual Point* getOrientation() override;

protected:
    std::vector<std::vector<std::shared_ptr<Vertex> > > polyLines;
    const short BBOX_SPHERE_RATIO = 1000;                   //Divisive coefficient between the BBox longest diagonal and neighborhood sphere radius
    const bool POST_PROCESSING = true;                     //Spikes can be removed in postprocessing or avoided by inserting infinite weights in the shortest path computation.
};

#endif // LINEANNOTATION_H
