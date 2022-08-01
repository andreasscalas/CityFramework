#ifndef TRIANGLEHELPER_H
#define TRIANGLEHELPER_H

#include <vector>
#include <string>


namespace TriHelper {
    class TriangleHelper
    {
    public:
        TriangleHelper(std::vector<double*> boundary, std::vector<std::vector<double*> > holes, std::vector<double*> constraints_vertices, std::vector<std::pair<unsigned int, unsigned int> > segments, bool boundQuality = false);

        ~TriangleHelper();
        const std::vector<unsigned int> &getTriangles() const;


        const std::vector<double *> &getAddedPoints() const;


    private:
        std::vector<double*> boundary;
        std::vector<std::vector<double*> > holes;
        std::vector<double*>  constraints_vertices;
        std::vector<std::pair<unsigned int, unsigned int> > constraints_segments;
        std::vector<double*> addedPoints;
        std::vector<unsigned int> triangles;
        bool boundQuality;
        const std::string filename = "tmp";
        void launchTriangle();
    };

}

#endif // TRIANGLEHELPER_H
