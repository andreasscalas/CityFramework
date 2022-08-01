#include "trianglehelper.h"

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
#include <fstream>
#include <iostream>

TriangleHelper::TriangleHelper(std::vector<double*> boundary, std::vector<std::vector<double*> > holes, std::vector<double *> constraints_vertices, std::vector<std::pair<unsigned int, unsigned int> > constraints_segments, bool boundQuality){

    this->boundary.insert(this->boundary.end(), boundary.begin(), boundary.end());
    this->holes.insert(this->holes.end(), holes.begin(), holes.end());
    this->constraints_vertices.insert(this->constraints_vertices.end(), constraints_vertices.begin(), constraints_vertices.end());
    this->constraints_segments.insert(this->constraints_segments.end(), constraints_segments.begin(), constraints_segments.end());
    this->boundQuality = boundQuality;
    launchTriangle();
}

TriangleHelper::~TriangleHelper()
{
    for(unsigned int i = 0; i < boundary.size(); i++)
        if(boundary[i] != nullptr)
            delete boundary[i];
    for(unsigned int i = 0; i < holes.size(); i++)
        for(unsigned int j = 0; j < holes[i].size(); j++)
            if(holes[i][j] != nullptr)
                delete holes[i][j];
    for(unsigned int i = 0; i < constraints_vertices.size(); i++)
        if(constraints_vertices[i] != nullptr)
            delete constraints_vertices[i];
    for(unsigned int i = 0; i < addedPoints.size(); i++)
        if(addedPoints[i] != nullptr)
            delete addedPoints[i];
}

const std::vector<unsigned int> &TriangleHelper::getTriangles() const
{
    return triangles;
}

const std::vector<double *> &TriangleHelper::getAddedPoints() const
{
    return addedPoints;
}

void TriangleHelper::launchTriangle(){

//    std::ofstream outStream("polyin.poly");
//    if(outStream.is_open())
//    {
//        outStream.precision(15);
//        unsigned int vertices_number = 0;
//        unsigned int segments_number = 0;
//        unsigned int vert_id = 1;
//        unsigned int seg_id = 1;
//        for(unsigned int i = 0; i < boundary.size(); i++)
//        {
//            vertices_number++;
//            segments_number++;
//        }
//        for(unsigned int i = 0; i < holes.size(); i++)
//            for(unsigned int j = 0; j < holes[i].size(); j++)
//            {
//                vertices_number++;
//                segments_number++;
//            }
//        vertices_number += constraints_vertices.size();
//        segments_number += constraints_segments.size();

//        outStream << vertices_number << " 2 1 0" << std::endl;
//        for(unsigned int i = 0; i < boundary.size(); i++)
//            outStream << vert_id++ << " " << boundary[i][0] << " " << boundary[i][1] << std::endl;
//        for(unsigned int i = 0; i < holes.size(); i++)
//            for(unsigned int j = 0; j < holes[i].size(); j++)
//                outStream << vert_id++  << " " << holes[i][j][0] << " " << holes[i][j][1] << std::endl;
//        for(unsigned int i = 0; i < constraints_vertices.size(); i++)
//                outStream << vert_id++ << " "  << constraints_vertices[i][0] << " " << constraints_vertices[i][1] << std::endl;

//        outStream << segments_number << " 1" << std::endl;
//        unsigned int reached_id = 1;
//        for(unsigned int i = 2; i < boundary.size() + 1; i++)
//            outStream <<  seg_id++ << " " << i - 1 << " " << i << " 1" << std::endl;
//        outStream <<  seg_id++ << " " << boundary.size() << " 1 1" << std::endl;
//        reached_id = boundary.size() + 1;
//        for(unsigned int i = 0; i < holes.size(); i++)
//        {
//            for(unsigned int j = 1; j < holes[i].size(); j++)
//                outStream <<  seg_id++ << " " << reached_id + j - 1 << " " << reached_id + j << " 1" << std::endl;
//            outStream <<  seg_id++ << " " << reached_id + holes[i].size() - 1 << " " << reached_id << " 1" << std::endl;
//            reached_id += holes[i].size();

//        }
//        for(unsigned int i = 0; i < constraints_segments.size(); i++)
//        {
//            outStream <<  seg_id++ << " " << constraints_segments[i].first + 1 << " " << constraints_segments[i].second + 1 << " 0" << std::endl;
//        }

//        outStream << holes.size() << std::endl;
//        for(unsigned int hid = 0; hid < holes.size(); ++hid){

//            vector<double*> outline = holes[hid];

//            double v[2] = {(outline[1][0] - outline[0][0]) * 1E-2, (outline[1][1] - outline[0][1]) * 1E-2};
//            double vec[2] = {-v[1], v[0]};
//            double middle[2] = {(outline[1][0] + outline[0][0]) / 2, (outline[1][1] + outline[0][1]) / 2};
//            double innerpoint[2] = {vec[0] + middle[0], vec[1] + middle[1]};
//            double turningSign =    (outline[1][1] - outline[0][1]) * (innerpoint[0] - outline[0][0]) -
//                                    (outline[1][0] - outline[0][0]) * (innerpoint[1] - outline[0][1]);
//            if(turningSign < 0){
//                innerpoint[0] = -vec[0] + middle[0];
//                innerpoint[1] = -vec[1] + middle[1];
//            }

//            outStream << hid + 1 << " " << innerpoint[0] << " " << innerpoint[1] << std::endl;
//        }

//        outStream.close();

//    }

//    return;
    triangulateio in, out;

    vector<pair<unsigned int, unsigned int> > segments;
    vector<bool> onBoundary;

    int numberOfPoints = static_cast<int>(boundary.size());

    numberOfPoints += constraints_vertices.size();

    for(unsigned int i = 0; i < holes.size(); i++)
        numberOfPoints += holes[i].size();

    in.numberofpoints = numberOfPoints;
    in.pointlist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofpoints), sizeof(double)));
    in.numberofpointattributes = 0;
    in.pointmarkerlist  = static_cast<int*>(calloc(static_cast<size_t>(in.numberofpoints), sizeof(int)));
    for(int vid = 0; vid < in.numberofpoints; vid++)
       in.pointmarkerlist[vid] = 1;

    unsigned int reachedID;
    for(unsigned int vid = 0; vid < static_cast<unsigned int>(boundary.size()); vid++){

        in.pointlist[vid * 2  ]     = boundary[vid][0];
        in.pointlist[vid * 2 + 1]   = boundary[vid][1];
        if(vid > 0)
        {
            segments.push_back(make_pair(vid - 1, vid));
            onBoundary.push_back(true);
        }
        reachedID = vid;
    }

    segments.push_back(make_pair(reachedID, 0));
    onBoundary.push_back(true);
    reachedID++;

    for(vector<vector<double*> >::iterator bit = holes.begin(); bit != holes.end(); bit++){
        unsigned int vid = reachedID;
        vector<double*> outline = (*bit);
        for(vector<double*>::iterator vit = outline.begin(); vit != outline.end(); vit++){
            in.pointlist[vid * 2  ]     = (*vit)[0];
            in.pointlist[vid * 2 + 1]   = (*vit)[1];
            if(vid > reachedID)
            {
                segments.push_back(make_pair(vid - 1, vid));
                onBoundary.push_back(true);
            }

            vid++;
        }
        segments.push_back(make_pair(vid - 1, reachedID));
        onBoundary.push_back(true);
        reachedID = vid;
    }


    in.numberofsegments = static_cast<int>(segments.size() + constraints_segments.size());
    in.segmentlist      = static_cast<int*>(calloc(static_cast<size_t>(2 * in.numberofsegments), sizeof(int)));

    for(unsigned int i = 0; i < segments.size(); i ++)
    {
        in.segmentlist[i * 2] = static_cast<int>(segments[i].first);
        in.segmentlist[i * 2 + 1] = static_cast<int>(segments[i].second);
        onBoundary.push_back(true);
    }

    for(vector<double*> ::iterator cit = constraints_vertices.begin(); cit != constraints_vertices.end(); cit++){
        in.pointlist[reachedID * 2  ]     = (*cit)[0];
        in.pointlist[reachedID * 2 + 1]   = (*cit)[1];
        reachedID++;
    }
    for(unsigned int i = 0; i < constraints_segments.size(); i++)
    {
        in.segmentlist[(segments.size() + i) * 2] = static_cast<int>(constraints_segments[i].first);
        in.segmentlist[(segments.size() + i) * 2 + 1] = static_cast<int>(constraints_segments[i].second);
        onBoundary.push_back(false);
    }

    in.segmentmarkerlist = static_cast<int*>(calloc(static_cast<size_t>(onBoundary.size()), sizeof(int)));
    for(unsigned int i = 0; i < onBoundary.size(); i++)
        in.segmentmarkerlist[i] = onBoundary[i];

    in.numberofholes = static_cast<int>(holes.size());
    in.holelist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofholes), sizeof(double)));

    for(unsigned int hid = 0; hid < static_cast<unsigned int>(in.numberofholes); ++hid){

        vector<double*> outline = holes[hid];

        double v[2] = {(outline[1][0] - outline[0][0]) * 1E-2, (outline[1][1] - outline[0][1]) * 1E-2};
        double vec[2] = {-v[1], v[0]};
        double middle[2] = {(outline[1][0] + outline[0][0]) / 2, (outline[1][1] + outline[0][1]) / 2};
        double innerpoint[2] = {vec[0] + middle[0], vec[1] + middle[1]};
        double turningSign =    (outline[1][1] - outline[0][1]) * (innerpoint[0] - outline[0][0]) -
                                (outline[1][0] - outline[0][0]) * (innerpoint[1] - outline[0][1]);
        if(turningSign < 0){
            innerpoint[0] = -vec[0] + middle[0];
            innerpoint[1] = -vec[1] + middle[1];
        }

        in.holelist[hid * 2  ]      = innerpoint[0];
        in.holelist[hid * 2 + 1]    = innerpoint[1];
    }

    in.numberoftriangles          = 0;
    in.numberofcorners            = 0;
    in.numberoftriangleattributes = 0;
    in.trianglelist               = nullptr;
    in.triangleattributelist      = nullptr;
    in.numberofregions = 0;

    out.pointlist      = nullptr;
    out.trianglelist   = nullptr;
    out.segmentlist    = nullptr;

    std::string s = "pzBQ";
    if(boundQuality)
        s = "pzqBQ";


    triangulate(const_cast<char*>(s.c_str()), &in, &out, nullptr);



    for(int tid = 0; tid < out.numberoftriangles; tid++){
        triangles.push_back(out.trianglelist[tid * 3]);
        triangles.push_back(out.trianglelist[tid * 3 + 1]);
        triangles.push_back(out.trianglelist[tid * 3 + 2]);
    }

    if(out.numberofpoints > in.numberofpoints)
        for(int vid = 0; vid < out.numberofpoints - in.numberofpoints; vid++){
            addedPoints.push_back(new double(2));
            addedPoints.at(vid)[0] = out.pointlist[(in.numberofpoints + vid) * 2];
            addedPoints.at(vid)[1] = out.pointlist[(in.numberofpoints + vid) * 2 + 1];
        }
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.holelist);
    free(out.pointlist);
    free(out.trianglelist);
    free(out.segmentlist);

}

#undef ANSI_DECLARATORS
#undef REAL
#undef VOID
#undef TRILIBRARY
