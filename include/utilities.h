#ifndef UTILITIES_H
#define UTILITIES_H

#include "TriangleMesh.hpp"
#include "Point.hpp"
#include "relation.h"
#include "way.h"
#include "node.h"
#include "geotiff.h"


#include <vector>
#include <iterator>
#include <map>


namespace Utilities {

    const double EPSILON_DETERMINANT = 1e-12;
    const double EPSILON_CONTAINMENT = 1e-8;

    enum ERRORS {
        IOERROR
    };

    int mod(int val, int m);

    bool isPolygonClockwise(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon);


    /**
     * @brief load_shapefile_shp reader di shapefile (per gentile concessione di Daniela)
     * @param filename il nome (path + nome) dello shapefile
     * @param boundaries vettore che fa da contenitore ai poligoni nello shapefile
     * @return codice d'errore
     */
    int load_shapefile_shp(const std::string filename, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >> &boundaries);


    unsigned int findNodeInList(std::vector<OpenStreetMap::Node*> list, unsigned int begin, unsigned int end, OpenStreetMap::Node* p);

    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator findPointInList(
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator begin,
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator end,
            std::shared_ptr<SemantisedTriangleMesh::Point> p);


    /**
     * @brief isPointInSegment check di contenimento di un punto in un segmento
     * @param a punto di inizio del segmento
     * @param b punto di fine del segmento
     * @param p punto query
     * @return true se p è contenuto in ab, false altrimenti
     */
    bool isPointInSegment(SemantisedTriangleMesh::Point* a, SemantisedTriangleMesh::Point* b, SemantisedTriangleMesh::Point* p);


    inline double det(double a, double b, double c, double d)
    {
        return a * d - b * c;
    }

    inline double det3x3(double a11, double a12, double a13, double a21, double a22, double a23, double a31, double a32, double a33)
    {
        return (a11)*((a22)*(a33) - (a23)*(a32)) - (a12)*((a21)*(a33) - (a23)*(a31)) + (a13)*((a21)*(a32) - (a22)*(a31));
    }

    /**
     * @brief segmentsIntersection metodo per il check di intersezione tra segmenti (al momento funziona solo in 2D, deov ancora estenderlo).
     * @param p1 punto di inizio del segmento numero 1
     * @param p2 punto di fine del segmento numero 1
     * @param p3 punto di inizio del segmento numero 2
     * @param p4 punto di fine del segmento numero 2
     * @param pout il punto di intersezione (per riferimento)
     * @return true se i segmenti si intersecano, false altrimenti
     */
    bool segmentsIntersection(std::shared_ptr<SemantisedTriangleMesh::Point> p1,
                                     std::shared_ptr<SemantisedTriangleMesh::Point> p2,
                                     std::shared_ptr<SemantisedTriangleMesh::Point> p3,
                                     std::shared_ptr<SemantisedTriangleMesh::Point> p4,
                                     SemantisedTriangleMesh::Point &pout);

    /**
     * @brief polygonsIntersect metodo per il check di intersezione tra poligoni
     * @param polygon1 poligono numero 1
     * @param polygon2 poligono numero 2
     * @return true se i poligoni si intersecano, false altrimenti
     */
    bool polygonsIntersect(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon1,
                            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon2);

    SemantisedTriangleMesh::Point linePlaneIntersection(const SemantisedTriangleMesh::Point& p,
                                                               const SemantisedTriangleMesh::Point& q,
                                                               const SemantisedTriangleMesh::Point& r,
                                                               const SemantisedTriangleMesh::Point& s,
                                                               const SemantisedTriangleMesh::Point& t);

    /**
     * @brief bbExtraction metodo per estrarre la AABB dai poligoni (riduzione costo computazionale del calcolo di intersezione)
     * @param points il poligono
     * @return array di 4 elementi (5 in realtà, il primo e l'ultimo sono uguali) codificante la AABB. Seguo questa codifica per compatibilità con gli altri poligoni.
     */
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bbExtraction(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > points);

    /**
     * @brief bbExtraction metodo per estrarre la AABB dai poligoni (riduzione costo computazionale del calcolo di intersezione)
     * @param points il poligono
     * @return array di 4 elementi (5 in realtà, il primo e l'ultimo sono uguali) codificante la AABB. Seguo questa codifica per compatibilità con gli altri poligoni.
     */
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bbExtraction(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > points);


    void fix_polygons(std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > &boundaries);

    void fix_polygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &boundaries);

    void fix_lines(std::vector<std::shared_ptr<OpenStreetMap::Way> > &lines);

    void fix_lines(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines);

    bool isPointInsidePolygon(std::shared_ptr<SemantisedTriangleMesh::Point> v,
                              std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > boundary);

    bool isPointInsidePolygon(std::shared_ptr<SemantisedTriangleMesh::Point> v, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary);

    bool isPointOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> v, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary);

    bool isPolygonInsidePolygon(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary1, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary2);

    /**
     * @brief extractPolygonsHeights assolutamente da ottimizzare
     * @param boundaries
     * @param dhm_tiff
     * @return
     */
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > extractPolygonsHeights(std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > boundaries,
                                                                                             GeoTiff* dhm_tiff, double scale_factor = 1.0,
                                                                                             SemantisedTriangleMesh::Point origin = SemantisedTriangleMesh::Point(0,0,0));


    /**
     * @brief extractPolygonsHeights assolutamente da ottimizzare
     * @param boundaries
     * @param dhm_tiff
     * @return
     */
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > extractPolygonsHeights(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > boundaries,
                                                                                             GeoTiff* dhm_tiff, double scale_factor = 1.0,
                                                                                             SemantisedTriangleMesh::Point origin = SemantisedTriangleMesh::Point(0,0,0));


    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > extractPolygonsHeights(std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > > boundaries,
                                                                                             GeoTiff* dhm_tiff, double scale_factor = 1.0,
                                                                                             SemantisedTriangleMesh::Point origin = SemantisedTriangleMesh::Point(0,0,0));

    void associateHeights(SemantisedTriangleMesh::TriangleMesh* mesh, GeoTiff* dhm_tiff, double scale_factor, SemantisedTriangleMesh::Point origin);

    void refineLines(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > &lines,
                     GeoTiff* dhm_tiff, double scale_factor, SemantisedTriangleMesh::Point origin);

    void refineLines(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                     GeoTiff* dhm_tiff, double scale_factor, SemantisedTriangleMesh::Point origin);

    std::shared_ptr<SemantisedTriangleMesh::Vertex> extractNearestVertex(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > &frontier,
                                                                         std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, double> distances);

    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> dijkstra(std::shared_ptr<SemantisedTriangleMesh::Vertex> v1,
                                                                          std::shared_ptr<SemantisedTriangleMesh::Vertex> v2);

    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> polylineDijkstra(std::shared_ptr<SemantisedTriangleMesh::Vertex> v1,
                                                                                  std::shared_ptr<SemantisedTriangleMesh::Vertex> v2, bool optimize = true);

    bool checkOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> p, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > polygons);

    bool checkOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> p, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > polygons);

    void removeLinePointsInPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                    std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > polygons);

    void removeLinePointsInPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > polygons);

    std::vector<std::pair<unsigned int, unsigned int> > removeSegmentsIntersectingPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                                                                           std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > &polygons);

    std::vector<std::pair<unsigned int, unsigned int> > removeSegmentsIntersectingPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                                                                           std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &polygons);

    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > getClosest(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > list,
                                                                                          std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > p);

    void interpolate(const unsigned char color1[], const unsigned char color2[], float fraction, unsigned char (&color)[3]);


    std::vector<std::string> globVector(const std::string& pattern);

    bool isPointOutsideBounds(std::shared_ptr<SemantisedTriangleMesh::Point> p, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > bounds);
}

#endif //UTILITIES_H
