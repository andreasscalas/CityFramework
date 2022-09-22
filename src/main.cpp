#include <utilities.h>
#include <trianglehelper.h>
#include <Point.h>
#include <TriangleMesh.h>
#include <citygmlcore.h>

#include <geotiff.h>
#include <rapidjson/prettywriter.h>
#include <tinyxml2.h>

#include <annotationfilemanager.h>
#include <iostream>
#include <map>
#include <string>

using namespace std;
const double OFFSET = 5e-3;


int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        std::cerr << "Missing OSM map parameter" << std::endl;
        return 1;
    }
    if(argc < 3)
    {
        std::cerr << "Missing DTM file parameter" << std::endl;
        return 2;
    }
    if(argc < 4)
    {
        std::cerr << "Missing DSM file parameter" << std::endl;
        return 3;
    }
    if(argc < 5)
    {
        std::cerr << "Missing LiDAR file parameter" << std::endl;
        return 4;
    }
    if(argc < 5)
    {
        std::cerr << "Missing interest area file parameter" << std::endl;
        return 5;
    }

    CityGMLCore manager(argv[1], argv[2], argv[3], argv[4], argv[5]);

    manager.setLevel(0,
                     "/home/andreas/Documenti/Progetti/Extruder/build-minSizeRelease/level0.ply",
                     "/home/andreas/Documenti/Progetti/Extruder/build-minSizeRelease/annotations.ant");
//    manager.buildLevel(0);
    manager.buildLevel(1);


    return 0;

}
