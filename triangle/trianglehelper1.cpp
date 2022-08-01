#include "trianglehelper1.h"

#ifndef ANSI_DECLARATORS
#define ANSI_DECLARATORS
#endif
#ifndef TRILIBRARY
#define TRILIBRARY
#endif
#ifndef REAL
#define REAL double
#endif
#ifndef VOID
#define VOID void
#endif
#include "shewchuk_triangle.h"

#include <map>
using namespace std;
using namespace TriHelper;
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include <string>

TriangleHelper::TriangleHelper(std::vector<double *> points, std::vector<std::vector<unsigned int> > polylines, std::vector<double *> holes, bool boundQuality){

    this->points.insert(this->points.end(), points.begin(), points.end());
    this->polylines.insert(this->polylines.end(), polylines.begin(), polylines.end());
    this->holes.insert(this->holes.end(), holes.begin(), holes.end());
    this->boundQuality = boundQuality;
    launchTriangle();
}

TriangleHelper::~TriangleHelper()
{
    for(unsigned int i = 0; i < points.size(); i++)
        if(points[i] != nullptr)
            delete points[i];
}

const std::vector<double*> &TriangleHelper::getPoints() const
{
    return points;
}

const std::vector<unsigned int> &TriangleHelper::getTriangles() const
{
    return triangles;
}

const std::vector<double *> &TriangleHelper::getAddedPoints() const
{
    return addedPoints;
}


std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void TriangleHelper::launchTriangle(){

    std::ofstream outStream("polyin.poly");
    unsigned int vertices_number = 0;
    if(outStream.is_open())
    {
        outStream.precision(15);
        unsigned int vertices_number = 0;
        unsigned int segments_number = 0;
        unsigned int vert_id = 0;
        unsigned int seg_id = 0;
        vertices_number = points.size();

        outStream << vertices_number << " 2 1 1" << std::endl;
        for(unsigned int i = 0; i < points.size(); i++)
            outStream << vert_id++ << " " << points[i][0] << " " << points[i][1] << " " << points[i][2] << " 1" << std::endl;
        for(unsigned int i = 0; i < polylines.size(); i++)
            for(unsigned int j = 1; j < polylines.at(i).size(); j++)
                segments_number++;
        outStream << segments_number << " 1" << std::endl;
        for(unsigned int i = 0; i < polylines.size(); i++)
            for(unsigned int j = 1; j < polylines.at(i).size(); j++)
                outStream << seg_id++ << " " << polylines.at(i).at(j - 1) << " " << polylines.at(i).at(j)<< " 1" << std::endl;

        outStream << holes.size() << std::endl;
        for(unsigned int hid = 0; hid < holes.size(); ++hid)
            outStream << hid << " " << holes.at(hid)[0] << " " << holes.at(hid)[1] << std::endl;

        outStream.close();

    }


    std::string command = "/home/andreas/Documenti/Progetti/triangle/triangle ";
    std::string flags = "-pzBQ";
    if(boundQuality)
        flags.append("q");
    command.append(flags);
    command.append(" ./polyin.poly");
    exec(command.c_str());

    std::ifstream nodes_file("polyin.1.node");

    std::string line;
    unsigned int out_nodes_number;
    std::getline(nodes_file, line);
    std::stringstream ss;
    ss.str(line);
    ss >> out_nodes_number;
    ss.clear();

    points.clear();
    while(std::getline(nodes_file, line))
    {
        if(line[0] == '#')
            continue;
        unsigned int point_id;
        size_t start = line.find_first_not_of(' ');
        if(start != std::string::npos) line = line.substr(start);
        std::string::iterator new_end = std::unique(line.begin(), line.end(), [](char l, char r){ return l == r && l == ' ';});
        line.erase(new_end, line.end());
        std::stringstream ss;
        ss.str(line);
        ss >> point_id;
        points.push_back(new double(3));
        ss >> points.back()[0];
        ss >> points.back()[1];
        ss >> points.back()[2];
        if(points.size() > vertices_number)
            addedPoints.push_back(points.back());
    }

//    if(out_nodes_number > points.size())
//    {
//        for(unsigned int i = 0; i < points.size() && std::getline(nodes_file, line); i++); //Just ignoring already known nodes
//        while(std::getline(nodes_file, line))
//        {
//            if(line[0] == '#')
//                continue;
//            long point_id;
//            size_t start = line.find_first_not_of(' ');
//            if(start != std::string::npos) line = line.substr(start);
//            std::string::iterator new_end = std::unique(line.begin(), line.end(), [](char l, char r){ return l == r && l == ' ';});
//            line.erase(new_end, line.end());
//            std::stringstream ss;
//            ss.str(line);
//            ss >> point_id;
//            addedPoints.push_back(new double(2));
//            ss >> addedPoints.back()[0];
//            ss >> addedPoints.back()[1];
//        }
//    }

    nodes_file.close();
    exec("rm -r ./polyin.1.node");
    std::ifstream triangulation_file("polyin.1.ele");
    if(triangulation_file.is_open())
    {
        std::getline(triangulation_file, line);
        ss.str(line);
        unsigned int out_triangles_number, triangle_id = 0;
        ss >> out_triangles_number;
        while(std::getline(triangulation_file, line) && triangle_id < out_triangles_number)
        {
            if(line[0] == '#')
                continue;
            size_t start = line.find_first_not_of(' ');
            if(start != std::string::npos) line = line.substr(start);
            std::string::iterator new_end = std::unique(line.begin(), line.end(), [](char l, char r){ return l == r && l == ' ';});
            line.erase(new_end, line.end());
            std::stringstream ss;
            ss.str(line);
            ss >> triangle_id;
            long v1, v2, v3;
            ss >> v1;
            ss >> v2;
            ss >> v3;
            triangles.push_back(v1);
            triangles.push_back(v2);
            triangles.push_back(v3);
            ss.clear();
        }
    }
    triangulation_file.close();
    exec("rm -r ./polyin.1.ele");
    exec("rm -r ./polyin.1.poly");

    return;
//    triangulateio in, out;

//    vector<pair<unsigned int, unsigned int> > segments;
//    vector<bool> onBoundary;

//    in.numberofpoints = static_cast<int>(points.size());
//    in.pointlist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofpoints), sizeof(double)));
//    in.numberofpointattributes = 0;
//    in.pointmarkerlist  = static_cast<int*>(calloc(static_cast<size_t>(in.numberofpoints), sizeof(int)));
//    for(int vid = 0; vid < in.numberofpoints; vid++)
//       in.pointmarkerlist[vid] = 1;

//    for(unsigned int i = 0; i < static_cast<unsigned int>(points.size()); i++)
//    {
//        in.pointlist[i * 2  ]     = points.at(i)[0];
//        in.pointlist[i * 2 + 1]   = points.at(i)[1];
//    }
//    for(unsigned int i = 0; i < static_cast<unsigned int>(polylines.size()); i++)
//        for(unsigned int j = 1; j < polylines.at(i).size(); j++)
//        {
//            segments.push_back(make_pair(polylines.at(i).at(j - 1), polylines.at(i).at(j)));
//            onBoundary.push_back(true);

//        }

//    in.numberofsegments = static_cast<int>(segments.size());
//    in.segmentlist      = static_cast<int*>(calloc(static_cast<size_t>(2 * in.numberofsegments), sizeof(int)));

//    for(unsigned int i = 0; i < segments.size(); i++)
//    {
//        in.segmentlist[i * 2] = static_cast<int>(segments[i].first);
//        in.segmentlist[i * 2 + 1] = static_cast<int>(segments[i].second);
//        onBoundary.push_back(true);
//    }

//    in.segmentmarkerlist = nullptr;/*static_cast<int*>(calloc(static_cast<size_t>(onBoundary.size()), sizeof(int)));
//    for(unsigned int i = 0; i < onBoundary.size(); i++)
//        in.segmentmarkerlist[i] = onBoundary[i];*/

//    in.numberofholes = static_cast<int>(holes.size());
//    in.holelist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofholes), sizeof(double)));

//    for(unsigned int i = 0; i < static_cast<unsigned int>(in.numberofholes); ++i)
//    {

//        in.holelist[i * 2  ]      = holes.at(i)[0];
//        in.holelist[i * 2 + 1]    = holes.at(i)[1];
//    }

//    in.numberoftriangles          = 0;
//    in.numberofcorners            = 0;
//    in.numberoftriangleattributes = 0;
//    in.trianglelist               = nullptr;
//    in.triangleattributelist      = nullptr;
//    in.numberofregions = 0;

//    out.pointlist      = nullptr;
//    out.trianglelist   = nullptr;
//    out.segmentlist    = nullptr;

//    std::string s = "pzB";
//    if(boundQuality)
//        s = "pzqBQ";

//    triangulate(const_cast<char*>(s.c_str()), &in, &out, nullptr);

//    for(int tid = 0; tid < out.numberoftriangles; tid++){
//        triangles.push_back(static_cast<unsigned int >(out.trianglelist[tid * 3]));
//        triangles.push_back(static_cast<unsigned int >(out.trianglelist[tid * 3 + 1]));
//        triangles.push_back(static_cast<unsigned int >(out.trianglelist[tid * 3 + 2]));
//    }

//    if(out.numberofpoints > in.numberofpoints)
//        for(int vid = 0; vid < out.numberofpoints - in.numberofpoints; vid++){
//            addedPoints.push_back(new double(2));
//            addedPoints.at(static_cast<unsigned int >(vid))[0] = out.pointlist[(in.numberofpoints + vid) * 2];
//            addedPoints.at(static_cast<unsigned int >(vid))[1] = out.pointlist[(in.numberofpoints + vid) * 2 + 1];
//        }
//    free(in.pointlist);
//    free(in.pointmarkerlist);
//    free(in.segmentlist);
//    free(in.holelist);
//    free(out.pointlist);
//    free(out.trianglelist);
//    free(out.segmentlist);

}

#undef ANSI_DECLARATORS
#undef REAL
#undef VOID
#undef TRILIBRARY
