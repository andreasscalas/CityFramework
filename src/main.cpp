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
        std::cerr << "Missing LiDAR file parameter" << std::endl;
        return 2;
    }
    if(argc < 4)
    {
        std::cerr << "Missing interest area file parameter" << std::endl;
        return 3;
    }

    CityGMLCore manager(argv[1], argv[2], argv[3]);


    auto start = std::chrono::high_resolution_clock::now();
//    manager.setLevel(0,
//                     "/home/andreas/Documenti/Progetti/Extruder/build-debug/level0.ply",
//                     "/home/andreas/Documenti/Progetti/Extruder/build-debug/annotations.ant");
    manager.buildLevel(0);
//    manager.buildLevel(1);
    auto end = std::chrono::high_resolution_clock::now();


    return 0;

}
