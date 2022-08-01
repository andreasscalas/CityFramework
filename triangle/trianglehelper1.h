#ifndef TRIANGLEHELPER_H
#define TRIANGLEHELPER_H

#include <vector>
#include <string>


namespace TriHelper {
    class TriangleHelper
    {
    public:
        TriangleHelper(std::vector<double*> points, std::vector<std::vector<unsigned int> > polylines, std::vector<double*> holes, bool boundQuality = false);

        ~TriangleHelper();
        const std::vector<double*> &getPoints() const;
        const std::vector<unsigned int> &getTriangles() const;


        const std::vector<double *> &getAddedPoints() const;


    private:
        std::vector<double*> points;
        std::vector<std::vector<unsigned int> > polylines;
        std::vector<double*> holes;
        std::vector<double*> addedPoints;
        std::vector<unsigned int> triangles;
        bool boundQuality;
        const std::string filename = "tmp";
        void launchTriangle();
    };

}

#endif // TRIANGLEHELPER_H
