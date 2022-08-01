#include <geotiff.h>


#include <string>
#include <cpl_conv.h>
#include <gdalwarper.h>
#include <stdlib.h>


GeoTiff::GeoTiff(const char *tiffname) {
    filename = tiffname ;
    GDALAllRegister();

    // set pointer to Geotiff dataset as class member.
    geotiffDataset = (GDALDataset*) GDALOpen(filename, GA_ReadOnly);

    // set the dimensions of the Geotiff
    NROWS   = GDALGetRasterYSize( geotiffDataset );
    NCOLS   = GDALGetRasterXSize( geotiffDataset );
    NLEVELS = GDALGetRasterCount( geotiffDataset );

}

GeoTiff::~GeoTiff() {
    // close the Geotiff dataset, free memory for array.
    GDALClose(geotiffDataset);
    GDALDestroyDriverManager();
}

const char *GeoTiff::GetFileName() {
    /*
       * function GetFileName()
       * This function returns the filename of the Geotiff.
       */
    return filename;
}

const char *GeoTiff::GetProjection() {
    /* function const char* GetProjection():
       *  This function returns a character array (string)
       *  for the projection of the geotiff file. Note that
       *  the "->" notation is used. This is because the
       *  "geotiffDataset" class variable is a pointer
       *  to an object or structure, and not the object
       *  itself, so the "." dot notation is not used.
       */
    return geotiffDataset->GetProjectionRef();
}

double *GeoTiff::GetGeoTransform() {
    /*
       * function double *GetGeoTransform()
       *  This function returns a pointer to a double that
       *  is the first element of a 6 element array that holds
       *  the geotransform of the geotiff.
       */
    geotiffDataset->GetGeoTransform(geotransform);
    return geotransform;
}

double GeoTiff::GetNoDataValue() {
    /*
       * function GetNoDataValue():
       *  This function returns the NoDataValue for the Geotiff dataset.
       *  Returns the NoData as a double.
       */
    return (double)geotiffDataset->GetRasterBand(1)->GetNoDataValue();
}

int *GeoTiff::GetDimensions() {
    /*
       * int *GetDimensions():
       *
       *  This function returns a pointer to an array of 3 integers
       *  holding the dimensions of the Geotiff. The array holds the
       *  dimensions in the following order:
       *   (1) number of columns (x size)
       *   (2) number of rows (y size)
       *   (3) number of bands (number of bands, z dimension)
       */
    dimensions[0] = NROWS;
    dimensions[1] = NCOLS;
    dimensions[2] = NLEVELS;
    return dimensions;
}

float **GeoTiff::GetRasterBand(int z) {

    /*
       * function float** GetRasterBand(int z):
       * This function reads a band from a geotiff at a
       * specified vertical level (z value, 1 ...
       * n bands). To this end, the Geotiff's GDAL
       * data type is passed to a switch statement,
       * and the template function GetArray2D (see below)
       * is called with the appropriate C++ data type.
       * The GetArray2D function uses the passed-in C++
       * data type to properly read the band data from
       * the Geotiff, cast the data to float**, and return
       * it to this function. This function returns that
       * float** pointer.
       */

    float** bandLayer = new float*[NROWS];
    switch( GDALGetRasterDataType(geotiffDataset->GetRasterBand(z)) ) {
    case 0:
        return NULL; // GDT_Unknown, or unknown data type.
    case 1:
        // GDAL GDT_Byte (-128 to 127) - unsigned  char
        return GetArray2D<unsigned char>(z,bandLayer);
    case 2:
        // GDAL GDT_UInt16 - short
        return GetArray2D<unsigned short>(z,bandLayer);
    case 3:
        // GDT_Int16
        return GetArray2D<short>(z,bandLayer);
    case 4:
        // GDT_UInt32
        return GetArray2D<unsigned int>(z,bandLayer);
    case 5:
        // GDT_Int32
        return GetArray2D<int>(z,bandLayer);
    case 6:
        // GDT_Float32
        return GetArray2D<float>(z,bandLayer);
    case 7:
        // GDT_Float64
        return GetArray2D<double>(z,bandLayer);
    default:
        break;
    }
    return NULL;
}
