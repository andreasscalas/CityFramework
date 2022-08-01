#ifndef GEOTIFF_H
#define GEOTIFF_H


#include <iostream>
#include <gdal_priv.h>
class GeoTiff {

  private: // NOTE: "private" keyword is redundant here.
           // we place it here for emphasis. Because these
           // variables are declared outside of "public",
           // they are private.

    const char* filename;        // name of Geotiff
    GDALDataset *geotiffDataset; // Geotiff GDAL datset object.
    double geotransform[6];      // 6-element geotranform array.
    int dimensions[3];           // X,Y, and Z dimensions.
    int NROWS,NCOLS,NLEVELS;     // dimensions of data in Geotiff.

  public:

    // define constructor function to instantiate object
    // of this Geotiff class.
    GeoTiff( const char* tiffname );

    // define destructor function to close dataset,
    // for when object goes out of scope or is removed
    // from memory.
    ~GeoTiff();

    const char *GetFileName();

    const char *GetProjection();

    double *GetGeoTransform();

    double GetNoDataValue();

    int *GetDimensions();

   float** GetRasterBand(int z);

    template<typename T>
    float** GetArray2D(int layerIndex,float** bandLayer) {

       /*
        * function float** GetArray2D(int layerIndex):
        * This function returns a pointer (to a pointer)
        * for a float array that holds the band (array)
        * data from the geotiff, for a specified layer
        * index layerIndex (1,2,3... for GDAL, for Geotiffs
        * with more than one band or data layer, 3D that is).
        *
        * Note this is a template function that is meant
        * to take in a valid C++ data type (i.e. char,
        * short, int, float), for the Geotiff in question
        * such that the Geotiff band data may be properly
        * read-in as numbers. Then, this function casts
        * the data to a float data type automatically.
        */

       // get the raster data type (ENUM integer 1-12,
       // see GDAL C/C++ documentation for more details)
       GDALDataType bandType = GDALGetRasterDataType(
         geotiffDataset->GetRasterBand(layerIndex));

       // get number of bytes per pixel in Geotiff
       int nbytes = GDALGetDataTypeSizeBytes(bandType);

       // allocate pointer to memory block for one row (scanline)
       // in 2D Geotiff array.
       T *rowBuff = (T*) CPLMalloc(nbytes*NCOLS);

       for(int row=0; row<NROWS; row++) {     // iterate through rows

         // read the scanline into the dynamically allocated row-buffer
         CPLErr e = geotiffDataset->GetRasterBand(layerIndex)->RasterIO(
           GF_Read,0,row,NCOLS,1,rowBuff,NCOLS,1,bandType,0,0);
         if(!(e == 0)) {
           std::cout << "Warning: Unable to read scanline in Geotiff!" << std::endl;
           exit(1);
         }

         bandLayer[row] = new float[NCOLS];
         for( int col=0; col<NCOLS; col++ ) { // iterate through columns
           bandLayer[row][col] = (float)rowBuff[col];
         }
       }
       CPLFree( rowBuff );
       return bandLayer;
    }

};
#endif // GEOTIFF_H
