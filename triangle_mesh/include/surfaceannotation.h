#ifndef AREAANNOTATION_H
#define AREAANNOTATION_H

#include <annotation.h>

class SurfaceAnnotation : virtual public Annotation
{
public:
    SurfaceAnnotation();
    SurfaceAnnotation(TriangleMesh *mesh, std::shared_ptr<SurfaceAnnotation>);

    virtual ~SurfaceAnnotation() override;

    virtual void print(std::ostream&) override;

    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    virtual std::vector<std::shared_ptr<Vertex> > getInvolvedVertices() override;
    virtual std::vector<std::shared_ptr<Vertex> > getInnerVertices();

    virtual bool isPointInAnnotation(std::shared_ptr<Vertex> p) override;
    virtual bool isPointInsideAnnotation(std::shared_ptr<Vertex> p);
    bool isPointOnBorder(std::shared_ptr<Vertex> p);
    bool isPointOnBorder(std::shared_ptr<Vertex> p, unsigned int &boundaryIndex);

    virtual bool isTriangleInAnnotation(std::shared_ptr<Triangle> p);

    std::pair<unsigned int, unsigned int> getAdjacentBoundary(SurfaceAnnotation* a);
    bool checkAdjacency(SurfaceAnnotation* a);


    //Getter and setter methods
    std::vector<std::vector<std::shared_ptr<Vertex>> > getOutlines() const;
    void setOutlines(const std::vector<std::vector<std::shared_ptr<Vertex>> > value);
    void addOutline(const std::vector<std::shared_ptr<Vertex>> value);
    std::vector<std::shared_ptr<Triangle>> getTriangles();


    virtual Point* getCenter() override;
    virtual Point* getOrientation() override;

    /**
         * @brief regionGrowing method for obtaining the set of triangles composing a region enclosed by some contours. Inside/outside ambiguity is solved imposing that the
         * contours must be ordered coherently (here counterclockwisely).
         * @param contours the contours enclosing the region.
         * @return the set of triangles composing the region.
         */
    static std::vector<std::shared_ptr<Triangle> > regionGrowing(std::vector<std::vector<std::shared_ptr<Vertex> > > contours);
    static std::vector<std::vector<std::shared_ptr<Vertex> > > getOutlines(std::vector<std::shared_ptr<Triangle>> set);


protected:
    std::vector<std::vector<std::shared_ptr<Vertex>> > outlines;    //The outline of the annotated region
    const short BBOX_SPHERE_RATIO = 100;                            //Divisive coefficient between the BBox longest diagonal and neighborhood sphere radius
    const bool ORDER = true;                                        //Order of the outline: if TRUE then it is counterclockwise, otherwise is clockwise
    const bool POST_PROCESSING = false;                             //Spikes can be removed in postprocessing or avoided by inserting infinite weights in the shortest path computation.

};

#endif // AREAANNOTATION_H
