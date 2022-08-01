#ifndef COORDSCONVERTER_H
#define COORDSCONVERTER_H

#include "proj.h"
#include <memory>

class CoordsConverter
{
public:
    CoordsConverter( unsigned int epsg_utm);

    void convertToWgs84(double x_utm, double y_utm, double &lat, double &lon);
    void convertToUTM(double lat, double lon, double &x_utm, double &y_utm);

private:

    PJ* P1;
    PJ* P2;


};

#endif // COORDSCONVERTER_H
