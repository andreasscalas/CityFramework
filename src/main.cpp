#include "citygmlcore.h"
#include <iostream>
#include <chrono>
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
                     "/home/andreas/Documenti/Progetti/Extruder/build-minSizeRelease/Catania/simplified/level0.ply",
                     "/home/andreas/Documenti/Progetti/Extruder/build-minSizeRelease/Catania/simplified/annotations.ant");
    auto start = std::chrono::high_resolution_clock::now();
    //manager.buildLevel(0);
    manager.buildLevel(1);
    auto end = std::chrono::high_resolution_clock::now();


    return 0;

}
