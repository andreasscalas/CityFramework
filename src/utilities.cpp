#include "utilities.h"
#include "coordsconverter.h"

#include <shapefil.h>
#include <tinyxml2.h>
#include <KDTree.hpp>
#include <glob.h>
#include <fstream>
#include <algorithm>
#include <set>
#include <stack>


int Utilities::mod(int val, int m)
{
	return (val % m + m) % m;
}


bool Utilities::isPolygonClockwise(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon)
{
    double sum = 0.0;
    for(unsigned int i = 0; i < polygon.size(); i++)
    {
        sum += (polygon.at((i + 1) % polygon.size())->getX() - polygon.at(i)->getX()) *
               (polygon.at((i + 1) % polygon.size())->getY() + polygon.at(i)->getY());
    }

    return sum >= 0;
}

int Utilities::load_shapefile_shp(const std::string filename, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &boundaries)
{
    boundaries.clear();

    SHPHandle hSHP = SHPOpen(filename.c_str(), "rb");

    if (hSHP == NULL)
    {
        std::cerr << "Error while loading shapefile " << filename << std::endl;
        return IOERROR;
    }

    int nShapeType, nEntities;
    double adfBndsMin[4], adfBndsMax[4];

    SHPGetInfo(hSHP, &nEntities, &nShapeType, adfBndsMin, adfBndsMax);

    if ((nShapeType != SHPT_POLYGON) && (nShapeType != SHPT_POLYGONZ) &&
        (nShapeType != SHPT_ARC) && (nShapeType != SHPT_ARCZ))
    {
        std::cerr << "Error: no polygon nor line in the shapefile " << filename << std::endl;
        return 1;
    }

    for (int i = 0; i < nEntities; ++i)
    {
        SHPObject *obj = SHPReadObject(hSHP, i);


        uint size = obj->nVertices;
        uint polygonStart = 0;
        for(uint j = 0; j < obj->nParts; j++)
        {
            boundaries.push_back(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > ());
            uint partSize = 0;
            if(j < obj->nParts - 1)
                partSize = obj->panPartStart[j + 1];
            else
                partSize = size;
            partSize -=  obj->panPartStart[j];
            for(uint k = 0; k < partSize; k++)
            {
                uint index = obj->panPartStart[j] + k;
                std::shared_ptr<SemantisedTriangleMesh::Point> point =
                        std::make_shared<SemantisedTriangleMesh::Point>(obj->padfX[index], obj->padfY[index], obj->padfZ[index]);
                boundaries.back().push_back(point);
            }
        }

        SHPDestroyObject(obj);
    }

    SHPClose(hSHP);

    return 0;
}

unsigned int Utilities::findNodeInList(std::vector<OpenStreetMap::Node *> list, unsigned int begin, unsigned int end, OpenStreetMap::Node *p)
{
    for(unsigned int i = begin; i < end; i++)
        if(list.at(i)->getId() == p->getId())
            return i;
    return end;
}

std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator Utilities::findPointInList(
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator begin,
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator end,
        std::shared_ptr<SemantisedTriangleMesh::Point> p)
{
    for(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> >::iterator it = begin; it != end; it++)
        if(**it == *p)
            return it;
    return end;
}

bool Utilities::isPointInSegment(SemantisedTriangleMesh::Point *a, SemantisedTriangleMesh::Point *b, SemantisedTriangleMesh::Point *p)
{
    if((*p) == (*a) || (*p) == (*b))
        return true;
    double l = ((*b) - (*a)).norm();
    double l1 = ((*p) - (*a)).norm();
    double l2 = ((*b) - (*p)).norm();
    double w1 = l1 / l;
    double w2 = l2 / l;
    return w1 + w2 <= 1.0 + EPSILON_CONTAINMENT;
}

bool Utilities::segmentsIntersection(std::shared_ptr<SemantisedTriangleMesh::Point> p1, std::shared_ptr<SemantisedTriangleMesh::Point> p2, std::shared_ptr<SemantisedTriangleMesh::Point> p3, std::shared_ptr<SemantisedTriangleMesh::Point> p4, SemantisedTriangleMesh::Point &pout)
{
    double detL1 = det(p1->getX(), p1->getY(), p2->getX(), p2->getY());
    double detL2 = det(p3->getX(), p3->getY(), p4->getX(), p4->getY());
    double x1mx2 = p1->getX() - p2->getX();
    double x3mx4 = p3->getX() - p4->getX();
    double y1my2 = p1->getY() - p2->getY();
    double y3my4 = p3->getY() - p4->getY();

    double xnom = det(detL1, x1mx2, detL2, x3mx4);
    double ynom = det(detL1, y1my2, detL2, y3my4);
    double denom = det(x1mx2, y1my2, x3mx4, y3my4);
    if(abs(denom) < EPSILON_DETERMINANT)//Lines don't seem to cross
    {
        pout.setX(std::numeric_limits<double>::max());
        pout.setY(std::numeric_limits<double>::max());
        pout.setZ(0);
        return false;
    }

    pout.setX(xnom / denom);
    pout.setY(ynom / denom);
    pout.setZ(p1->getZ());
    if(!isfinite(pout.getX()) || !isfinite(pout.getY())) //Probably a numerical issue
        return false;

    bool result = isPointInSegment(p1.get(), p2.get(), &pout) && isPointInSegment(p3.get(), p4.get(), &pout);
    return result;
}

bool Utilities::polygonsIntersect(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon1, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > polygon2)
{
    for(unsigned int i = 1; i < polygon1.size(); i++)
    {
        for(unsigned int j = 1; j < polygon2.size(); j++)
        {
            SemantisedTriangleMesh::Point intersection;
            if(segmentsIntersection(polygon1[i - 1], polygon1[i], polygon2[j - 1], polygon2[j], intersection))
                return true;
        }
    }
    return false;
}

SemantisedTriangleMesh::Point Utilities::linePlaneIntersection(const SemantisedTriangleMesh::Point &p, const SemantisedTriangleMesh::Point &q, const SemantisedTriangleMesh::Point &r, const SemantisedTriangleMesh::Point &s, const SemantisedTriangleMesh::Point &t)
{
    SemantisedTriangleMesh::Point intersection(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
    double den = det3x3(p.getX() - q.getX(),
                        p.getY() - q.getY(),
                        p.getZ() - q.getZ(),
                        s.getX() - r.getX(),
                        s.getY() - r.getY(),
                        s.getZ() - r.getZ(),
                        t.getX() - r.getX(),
                        t.getY() - r.getY(),
                        t.getZ() - r.getZ());
    if (den == 0) return intersection;
    double num = det3x3( p.getX() - r.getX(),
                         p.getY() - r.getY(),
                         p.getZ() - r.getZ(),
                         s.getX() - r.getX(),
                         s.getY() - r.getY(),
                         s.getZ() - r.getZ(),
                         t.getX() - r.getX(),
                         t.getY() - r.getY(),
                         t.getZ() - r.getZ());
    double gamma = num / den;
    return p + ((q - p) * gamma);
}

std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > Utilities::bbExtraction(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > points)
{
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb;
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 0));

    for(unsigned int j = 0; j < points.size(); j++)
    {

        if(points[j]->getX() < bb[0]->getX())
        {
            bb[0]->setX(points[j]->getX());
            bb[3]->setX(points[j]->getX());
        }
        if(points[j]->getX() > bb[1]->getX())
        {
            bb[1]->setX(points[j]->getX());
            bb[2]->setX(points[j]->getX());
        }
        if(points[j]->getY() < bb[0]->getY())
        {
            bb[0]->setY(points[j]->getY());
            bb[1]->setY(points[j]->getY());
        }
        if(points[j]->getY() > bb[2]->getY())
        {
            bb[2]->setY(points[j]->getY());
            bb[3]->setY(points[j]->getY());
        }
    }
    bb.push_back(bb[0]);

    return bb;
}

std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > Utilities::bbExtraction(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > points)
{
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb;
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 0));
    bb.push_back(std::make_shared<SemantisedTriangleMesh::Point>(std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), 0));

    for(unsigned int j = 0; j < points.size(); j++)
    {

        if(points[j]->getX() < bb[0]->getX())
        {
            bb[0]->setX(points[j]->getX());
            bb[3]->setX(points[j]->getX());
        }
        if(points[j]->getX() > bb[1]->getX())
        {
            bb[1]->setX(points[j]->getX());
            bb[2]->setX(points[j]->getX());
        }
        if(points[j]->getY() < bb[0]->getY())
        {
            bb[0]->setY(points[j]->getY());
            bb[1]->setY(points[j]->getY());
        }
        if(points[j]->getY() > bb[2]->getY())
        {
            bb[2]->setY(points[j]->getY());
            bb[3]->setY(points[j]->getY());
        }
    }
    bb.push_back(bb[0]);

    return bb;
}



bool Utilities::isPointInsidePolygon(std::shared_ptr<SemantisedTriangleMesh::Point> v, std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > boundary){

    unsigned int c = 0;
    if(*boundary.back() != *boundary[0])
        boundary.push_back(boundary.at(0));
    for ( unsigned int i = 1; i < boundary.size(); i++) {
        if ( ((boundary[i]->getY() >= v->getY()) != (boundary[i - 1]->getY() >= v->getY())) &&
             (v->getX() <= (boundary[i - 1]->getX() - boundary[i]->getX()) * (v->getY() - boundary[i]->getY())
              / (boundary[i - 1]->getY() - boundary[i]->getY()) + boundary[i]->getX()) )
            c = !c;
    }
    return c;
}

bool Utilities::isPointInsidePolygon(std::shared_ptr<SemantisedTriangleMesh::Point> v, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary){

    unsigned int c = 0;
    if(*boundary.back() != *boundary[0])
        boundary.push_back(boundary.at(0));
    for ( unsigned int i = 1; i < boundary.size(); i++) {
        if ( ((boundary[i]->getY() >= v->getY()) != (boundary[i - 1]->getY() >= v->getY())) &&
             (v->getX() <= (boundary[i - 1]->getX() - boundary[i]->getX()) * (v->getY() - boundary[i]->getY())
              / (boundary[i - 1]->getY() - boundary[i]->getY()) + boundary[i]->getX()) )
            c = !c;
    }
    return c;
}

bool Utilities::isPointOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> v, std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary)
{
    for(uint i = 1; i < boundary.size(); i++)
        if(isPointInSegment(boundary.at(i - 1).get(), boundary.at(i).get(), v.get()))
            return true;
    return false;
}

bool Utilities::isPolygonInsidePolygon(std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary1,
                                       std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundary2){

    bool contained = false;
    for(auto v : boundary1)
        if(!isPointInsidePolygon(v, boundary2) && !isPointOnBoundary(v, boundary2))
            return false;
    return true;
}

std::vector<std::vector<std::pair<unsigned int, unsigned int> > > Utilities::extractPolygonsHeights(
        std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > boundaries, GeoTiff *dhm_tiff,
        double scale_factor, SemantisedTriangleMesh::Point origin)
{
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > heights;
    unsigned int counter = 0;
    std::map<unsigned int, std::pair<unsigned int, unsigned int> > index_to_rowcol;
    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::pair<unsigned int, unsigned int> > boundary_heights;
        heights.push_back(boundary_heights);
    }
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    std::cout << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << std::endl;
    std::cout << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << std::endl;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;
    for(unsigned int i = 0; i < static_cast<unsigned int>(dhm_tiff->GetDimensions()[0]); i++)
    {
        for(unsigned int j = 0; j < static_cast<unsigned int>(dhm_tiff->GetDimensions()[1]); j++)
        {

            std::vector<double> point = { geoTransform[0] + j * geoTransform[1],// + i * geoTransform[2],
                                          geoTransform[3] + i * geoTransform[5]};

            points_vector.push_back(point);
            index_to_rowcol.insert(std::make_pair(counter++, std::make_pair(i,j)));
        }
    }
    KDTree tree(points_vector);

    counter = 0;

    //#pragma omp parallel for num_threads(31)
    for(unsigned int i = 1; i < boundaries.size(); i++)
    {

        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(boundaries.at(i).at(0));
        double sphere_radius = std::max(pixel_diagonal_length, ((*bb[0]) - (*bb[2])).norm());
        SemantisedTriangleMesh::Point middle = ((*bb[0]) + (*bb[1]) + (*bb[2]) + (*bb[3])) / 4;


        std::vector<std::pair<unsigned int, unsigned int> > neighbors;
        point_t query_point = {middle.getX(), middle.getY()};
        std::vector<size_t> neighbors_distances = tree.neighborhood_indices(query_point, sphere_radius);

        if(neighbors_distances.size() == 0)
        {
            for(unsigned int j = 0; j < bb.size(); j++)
                bb.at(j)->print(std::cout);
            middle.print(std::cout);
            std::cout << query_point[0] << " " << query_point[1] << " " << query_point[2] << std::endl;
            std::cout << sphere_radius << std::endl;

        }
        for(std::vector<size_t>::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
            neighbors.push_back(index_to_rowcol.at(*it));
        }

        for(unsigned int j = 0; j < neighbors.size(); j++)
        {
            double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
            double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
            frame.push_back(frame[0]);
            if(((*frame[0]) - (*boundaries[i][0][0])).norm() < std::max(((*bb[0]) - (*bb[2])).norm(), ((*frame[0]) - (*frame[2])).norm()))
            {
                if(isPointInsidePolygon(boundaries[i][0][0], frame) ||                                         //Se un punto del poligono è dentro al pixel del geotiff...
                        (isPointInsidePolygon(frame[0], bb) && isPointInsidePolygon(frame[0], boundaries[i][0])) || //o il pixel è contenuto nel poligono...
                        polygonsIntersect(frame, boundaries[i][0]))                                                 //o ancora poligono e pixel si intersecano
                {
                    auto p = std::make_pair(neighbors.at(j).first, neighbors.at(j).second);
                    auto it = std::find_if(heights[i - 1].begin(), heights[i - 1].end(), [neighbors, i, j](std::pair<unsigned int, unsigned int> p)
                    {
                        return p.first == neighbors.at(j).first && p.second == neighbors.at(j).second;
                    });
                    if(it == heights[i - 1].end())
                        heights[i - 1].push_back(p);
                }
            }
            for(unsigned int l = 0; l < 4; l++)
                frame.at(l).reset();
        }
        if(heights[i - 1].size() == 0)
        {
#pragma omp critical
            {
                std::cerr << "Impossible case: boundary does not include any pixel while no pixel wholly include the boundary and boundary intersects no pixel" << std::endl;
                std::cerr << i << std::endl;
                std::cerr << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << " " << std::endl;
                std::cerr << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << " " << std::endl << std::flush;
                std::cerr << dhm_tiff->GetDimensions()[0] << " " << dhm_tiff->GetDimensions()[1] << std::endl;
                for(unsigned int j = 0; j < boundaries[i].size(); j++)
                    for(unsigned int k = 0; k < boundaries[i][j].size(); k++)
                        boundaries[i][j][k]->print(std::cerr);
                exit(6);
            }
        }
#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / boundaries.size() << "%\r" << std::flush;
        }

    }
    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];

    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
    return heights;
}

std::vector<std::vector<std::pair<unsigned int, unsigned int> > > Utilities::extractPolygonsHeights(
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > boundaries, GeoTiff *dhm_tiff,
        double scale_factor, SemantisedTriangleMesh::Point origin)
{
    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > heights;
    unsigned int counter = 0;
    std::map<unsigned int, std::pair<unsigned int, unsigned int> > index_to_rowcol;
    for(unsigned int i = 0; i < boundaries.size(); i++)
    {
        std::vector<std::pair<unsigned int, unsigned int> > boundary_heights;
        heights.push_back(boundary_heights);
    }
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1] + i * geoTransform[2],
                                          geoTransform[3] + j * geoTransform[5] + i * geoTransform[4],
                                          0};
            points_vector.push_back(point);
            index_to_rowcol.insert(std::make_pair(counter++, std::make_pair(i,j)));
        }
    }
    KDTree tree(points_vector);
    for(unsigned int i = 0; i < boundaries.at(0).size(); i++)
    {
        point_t query_point = {boundaries.at(0).at(i)->getX(), boundaries.at(0).at(i)->getY(), 0};

        std::vector<double> out_dist_sqr(1);
        size_t ret_index = tree.nearest_index(query_point);
        heights.at(0).push_back(index_to_rowcol.at(ret_index));
    }

    counter = 0;

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(boundaries[i]);
        double sphere_radius = std::max(pixel_diagonal_length, ((*bb[0]) - (*bb[2])).norm()) / 2;
        SemantisedTriangleMesh::Point middle = ((*bb[0]) + (*bb[1]) + (*bb[2]) + (*bb[3])) / 4;

        std::vector<size_t> neighbors_indices;
        std::vector<std::pair<unsigned int, unsigned int> > neighbors;
        point_t query_point = {middle.getX(), middle.getY(), 0};
        neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);

        for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
            neighbors.push_back(index_to_rowcol.at(*it));
        }

        for(unsigned int j = 0; j < neighbors.size(); j++)
        {
            double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
            double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
            frame.push_back(frame[0]);
            if(((*frame[0]) - (*boundaries[i][0])).norm() < std::max(((*bb[0]) - (*bb[2])).norm(), ((*frame[0]) - (*frame[2])).norm()))
            {
                if(isPointInsidePolygon(boundaries[i][0], frame) ||                                         //Se un punto del poligono è dentro al pixel del geotiff...
                        (isPointInsidePolygon(frame[0], bb) && isPointInsidePolygon(frame[0], boundaries[i])) || //o il pixel è contenuto nel poligono...
                        polygonsIntersect(frame, boundaries[i]))                                                 //o ancora poligono e pixel si intersecano
                {
                    heights[i].push_back(std::make_pair(neighbors.at(j).first, neighbors.at(j).second));
                }
            }
            for(unsigned int l = 0; l < 4; l++)
                frame.at(l).reset();
        }
        if(heights[i].size() == 0)
        {
#pragma omp critical
            {
                std::cerr << "Impossible case: boundary does not include any pixel while no pixel wholly include the boundary and boundary intersects no pixel" << std::endl;
                std::cerr << i << std::endl;
                std::cerr << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << " " << std::endl;
                std::cerr << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << " " << std::endl << std::flush;
                std::cerr << dhm_tiff->GetDimensions()[0] << " " << dhm_tiff->GetDimensions()[1] << std::endl;
                for(unsigned int j = 0; j < boundaries[i].size(); j++)
                    boundaries[i][j]->print(std::cerr);
                exit(6);
            }
        }
#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / boundaries.size() << "%\r" << std::flush;
        }

    }
    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];

    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
    return heights;
}

std::vector<std::vector<std::pair<unsigned int, unsigned int> > > Utilities::extractPolygonsHeights(
        std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > > boundaries, GeoTiff *dhm_tiff,
        double scale_factor, SemantisedTriangleMesh::Point origin)
{

    std::vector<std::vector<std::pair<unsigned int, unsigned int> > > heights;
    unsigned int counter = 0;
    std::map<unsigned int, std::pair<unsigned int, unsigned int> > index_to_rowcol;
    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::pair<unsigned int, unsigned int> > boundary_heights;
        heights.push_back(boundary_heights);
    }
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    std::cout << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << std::endl;
    std::cout << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << std::endl;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1] + i * geoTransform[2],
                                          geoTransform[3] + j * geoTransform[4] + i * geoTransform[5], //NEGATIVO???
                                          0};
            points_vector.push_back(point);
            index_to_rowcol.insert(std::make_pair(counter++, std::make_pair(i,j)));
        }
    }
    KDTree tree(points_vector);

    counter = 0;

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 1; i < boundaries.size(); i++)
    {
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(boundaries[i][0]);
        double sphere_radius = std::max(pixel_diagonal_length, ((*bb[0]) - (*bb[2])).norm());
        SemantisedTriangleMesh::Point middle(0,0,0);
        std::for_each(boundaries[i][0].begin(), boundaries[i][0].end(), [&middle](std::shared_ptr<SemantisedTriangleMesh::Vertex> v){
            middle += *v;
        });
        middle /= boundaries[i][0].size();

        std::vector<size_t> neighbors_indices;
        std::vector<std::pair<unsigned int, unsigned int> > neighbors;
        point_t query_point = {middle.getX(), middle.getY(), 0};
        neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);

        for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
            neighbors.push_back(index_to_rowcol.at(*it));
        }
        uint m = 0;

        for(unsigned int j = 0; j < neighbors.size(); j++)
        {
            double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
            double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
            frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
            frame.push_back(frame[0]);

            bool frameInBuilding = false, buildingInFrame = false;
            //Check frame interno (o intersecante) all'edificio
            for(uint k = 0; k < 4; k++)
                if(isPointInsidePolygon(frame[k], boundaries[i][0]))
                {
                    frameInBuilding = true;
                    for(uint l = 0; l <  boundaries[i].size(); l++)
                        if(isPointInsidePolygon(frame[0], boundaries[i][l]))
                        {
                            frameInBuilding = false;
                            break;
                        }
                    if(frameInBuilding)
                        break;
                }

            if(!frameInBuilding)
            {
                //Check bounding box interna al frame
                for(uint k = 0; k < 4; k++)
                {
                    if(isPointInsidePolygon(bb[k], frame))
                        for(uint l = 0; l < boundaries[i][0].size(); l++)
                        {
                            buildingInFrame = isPointInsidePolygon(boundaries[i][0][l], frame);
                            if(buildingInFrame)
                                break;
                        }

                    if(buildingInFrame)
                        break;
                }
            }

            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > boundaryPoints(boundaries[i][0].begin(), boundaries[i][0].end());
            if(frameInBuilding || buildingInFrame || polygonsIntersect(frame, boundaryPoints))
                heights[i - 1].push_back(std::make_pair(neighbors.at(j).first, neighbors.at(j).second));

            for(unsigned int l = 0; l < 4; l++)
                frame.at(l).reset();
        }
        if(heights[i - 1].size() == 0)
        {
#pragma omp critical
            {
                std::cerr << "Impossible case: boundary does not include any pixel while no pixel wholly include the boundary and boundary intersects no pixel" << std::endl;
                std::cerr << i << std::endl;
                std::cerr << geoTransform[0] << " " << geoTransform[1] << " " << geoTransform[2] << " " << std::endl;
                std::cerr << geoTransform[3] << " " << geoTransform[4] << " " << geoTransform[5] << " " << std::endl << std::flush;
                std::cerr << dhm_tiff->GetDimensions()[0] << " " << dhm_tiff->GetDimensions()[1] << std::endl;
                uint m = 0;
                for(auto list : boundaries[i])
                {
                    std::cerr << "A" << m++ << "=[" << std::endl;
                    for(auto p : list)
                        std::static_pointer_cast<SemantisedTriangleMesh::Point>(p)->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                    std::cerr << "];" << std::endl;
                }

                for(unsigned int j = 0; j < neighbors.size(); j++)
                {
                    double min_x = geoTransform[0] + neighbors.at(j).second * geoTransform[1] + neighbors.at(j).first * geoTransform[2];
                    double min_y = geoTransform[3] + neighbors.at(j).second * geoTransform[4] + neighbors.at(j).first * geoTransform[5];
                    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
                    frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
                    frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
                    frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
                    frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
                    frame.push_back(frame[0]);
                    frame[0]->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                    frame[1]->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                    frame[2]->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                    frame[3]->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                    frame[4]->print(std::cerr, SemantisedTriangleMesh::BracketsType::NONE, " ");
                }
                exit(6);
            }
        }
#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / boundaries.size() << "%\r" << std::flush;
        }

    }
    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];

    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
    return heights;
}

void Utilities::associateHeights(SemantisedTriangleMesh::TriangleMesh *mesh, GeoTiff *dhm_tiff, double scale_factor, SemantisedTriangleMesh::Point origin)
{
    unsigned int counter = 0;
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    pointVec points_vector;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1],
                                          geoTransform[3] + i * geoTransform[5],
                                          0};
            points_vector.push_back(point);
        }
    }

    KDTree tree(points_vector);

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
    {
        point_t point = {mesh->getVertex(i)->getX(), mesh->getVertex(i)->getY(), 0};
        size_t ret_index = tree.nearest_index(point);
        unsigned int maxPos = -1;
        unsigned int row = ret_index / dhm_tiff->GetDimensions()[1];
        unsigned int col = ret_index % dhm_tiff->GetDimensions()[1];
        mesh->getVertex(i)->setZ(dhm_heights[row][col]);
#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / mesh->getVerticesNumber() << "%\r" << std::flush;
        }

    }

    std::cout << std::endl;

    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];
    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;

}

void Utilities::refineLines(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > &lines, GeoTiff *dhm_tiff,
                            double scale_factor, SemantisedTriangleMesh::Point origin)
{
    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;
    unsigned int counter = 0;

    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1] + i * geoTransform[2],
                                          geoTransform[3] + j * geoTransform[4] + i * geoTransform[5],
                                          0};
            points_vector.push_back(point);

        }
    }
    KDTree tree(points_vector);

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < lines.size(); i++)
    {

        for(unsigned int j = 1; j < lines.at(i).size(); j++)
        {
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v1 = lines.at(i).at(j - 1);
            std::shared_ptr<SemantisedTriangleMesh::Point> v2 = lines.at(i).at(j);
            SemantisedTriangleMesh::Point middle(((*v1) + (*v2))  / 2);
            double sphere_radius = std::max(pixel_diagonal_length, ((*v1) - (*v2)).norm() / 2);

            std::vector<std::pair<unsigned int, unsigned int> > neighbors;
            point_t query_point = {middle.getX(), middle.getY(), 0};
            std::vector<size_t> neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);
            for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
                neighbors.push_back(std::make_pair(*it / dhm_tiff->GetDimensions()[0], *it % dhm_tiff->GetDimensions()[0]));
            }
            for(unsigned int k = 0; k < neighbors.size(); k++)
            {
                double min_x = geoTransform[0] + neighbors.at(k).second * geoTransform[1] + neighbors.at(k).first * geoTransform[2];
                double min_y = geoTransform[3] + neighbors.at(k).second * geoTransform[4] + neighbors.at(k).first * geoTransform[5];
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
                frame.push_back(frame[0]);

                for(unsigned int l = 1; l < 5; l++)
                {
                    std::shared_ptr<SemantisedTriangleMesh::Vertex> pout = std::make_shared<SemantisedTriangleMesh::Vertex>();
                    bool intersects = Utilities::segmentsIntersection(v1, v2, frame.at(l - 1), frame.at(l), *pout);

                    if(intersects)
                    {
                        int pos = -1;
                        for(unsigned int m = 0; m < intersections.size(); m++)
                            if(*intersections.at(m) == *pout)
                                pos = m;
                        if(pos < 0 && *pout != *v1 && *pout != *v2)
                            intersections.push_back(pout);
                        else if(pout != nullptr)
                            pout.reset();
                    } else if(pout != nullptr)
                        pout.reset();
                }

                for(unsigned int l = 1; l < 4; l++)
                    frame.at(l).reset();
            }
            neighbors.clear();
            neighbors_indices.clear();
            std::sort(intersections.begin(), intersections.end(), [v1](std::shared_ptr<SemantisedTriangleMesh::Point> a, std::shared_ptr<SemantisedTriangleMesh::Point> b) {
                return ((*a) - (*v1)).norm() < ((*b) - (*v1)).norm();
            });

#pragma omp critical
            {
                lines.at(i).insert(lines.at(i).begin() + j, intersections.begin(), intersections.end());
            }
            j += intersections.size();

        }

#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
        }
    }

    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];
    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
}

void Utilities::refineLines(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines, GeoTiff *dhm_tiff,
                            double scale_factor, SemantisedTriangleMesh::Point origin)
{

    float** dhm_heights = dhm_tiff->GetRasterBand(1);
    double* geoTransform = dhm_tiff->GetGeoTransform();
    geoTransform[0] = (geoTransform[0] - origin.getX()) / scale_factor;  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] - origin.getY()) / scale_factor;  //y del punto in alto a sinistra
    geoTransform[1] /= scale_factor;
    geoTransform[5] /= scale_factor;
    double pixel_diagonal_length = sqrt(pow(geoTransform[1], 2) + pow(geoTransform[5], 2));
    pointVec points_vector;
    unsigned int counter = 0;

    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
    {
        for(unsigned int j = 0; j < dhm_tiff->GetDimensions()[1]; j++)
        {
            std::vector<double> point = { geoTransform[0] + j * geoTransform[1] + i * geoTransform[2],
                                          geoTransform[3] + j * geoTransform[4] + i * geoTransform[5],
                                          0};
            points_vector.push_back(point);

        }
    }
    KDTree tree(points_vector);

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < lines.size(); i++)
    {

        for(unsigned int j = 1; j < lines.at(i).size(); j++)
        {
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v1 = lines.at(i).at(j - 1);
            std::shared_ptr<SemantisedTriangleMesh::Point> v2 = lines.at(i).at(j);
            SemantisedTriangleMesh::Point middle(((*v1) + (*v2))  / 2);
            double sphere_radius = std::max(pixel_diagonal_length, ((*v1) - (*v2)).norm() / 2);

            std::vector<std::pair<unsigned int, unsigned int> > neighbors;
            point_t query_point = {middle.getX(), middle.getY(), 0};
            std::vector<size_t> neighbors_indices = tree.neighborhood_indices(query_point, sphere_radius);
            for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
                neighbors.push_back(std::make_pair(*it / dhm_tiff->GetDimensions()[0], *it % dhm_tiff->GetDimensions()[0]));
            }
            for(unsigned int k = 0; k < neighbors.size(); k++)
            {
                double min_x = geoTransform[0] + neighbors.at(k).second * geoTransform[1] + neighbors.at(k).first * geoTransform[2];
                double min_y = geoTransform[3] + neighbors.at(k).second * geoTransform[4] + neighbors.at(k).first * geoTransform[5];
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > frame;
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, min_y, 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x + geoTransform[1], min_y, 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(frame.back()->getX(), min_y + geoTransform[5], 0));
                frame.push_back(std::make_shared<SemantisedTriangleMesh::Point>(min_x, frame.back()->getY(), 0));
                frame.push_back(frame[0]);

                for(unsigned int l = 1; l < 5; l++)
                {
                    std::shared_ptr<SemantisedTriangleMesh::Point> pout = std::make_shared<SemantisedTriangleMesh::Point>();
                    bool intersects = Utilities::segmentsIntersection(v1, v2, frame.at(l - 1), frame.at(l), *pout);

                    if(intersects)
                    {
                        int pos = -1;
                        for(unsigned int m = 0; m < intersections.size(); m++)
                            if(*intersections.at(m) == *pout)
                                pos = m;
                        if(pos < 0 && *pout != *v1 && *pout != *v2)
                            intersections.push_back(pout);
                        else if(pout != nullptr)
                            pout.reset();
                    } else if(pout != nullptr)
                        pout.reset();
                }

                for(unsigned int l = 1; l < 4; l++)
                    frame.at(l).reset();
            }
            neighbors.clear();
            neighbors_indices.clear();
            std::sort(intersections.begin(), intersections.end(), [v1](std::shared_ptr<SemantisedTriangleMesh::Point> a, std::shared_ptr<SemantisedTriangleMesh::Point> b) {
                return ((*a) - (*v1)).norm() < ((*b) - (*v1)).norm();
            });

#pragma omp critical
            {
                lines.at(i).insert(lines.at(i).begin() + j, intersections.begin(), intersections.end());
            }
            j += intersections.size();

        }

#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
        }
    }

    std::cout << std::endl;
    for(unsigned int i = 0; i < dhm_tiff->GetDimensions()[0]; i++)
        if(dhm_heights[i] != nullptr)
            delete[] dhm_heights[i];
    geoTransform[0] = (geoTransform[0] * scale_factor + origin.getX());  //x del punto in alto a sinistra
    geoTransform[3] = (geoTransform[3] * scale_factor + origin.getY());  //y del punto in alto a sinistra
    geoTransform[1] *= scale_factor;
    geoTransform[5] *= scale_factor;
}

std::shared_ptr<SemantisedTriangleMesh::Vertex> Utilities::extractNearestVertex(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > &frontier,
                                                                                std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, double> distances){

    if(frontier.size() == 0)
        return nullptr;

    double minDist = std::numeric_limits<double>::max();
    int minPos = -1;
    for(unsigned int i = 0; i < frontier.size(); i++){
        if(distances[frontier[i]] < minDist && frontier[i]->searchFlag(SemantisedTriangleMesh::FlagType::VISITED) < 0){
            minPos = i;
            minDist = distances[frontier[i]];
        }
    }

    if(minPos == -1)
        return nullptr;

    std::shared_ptr<SemantisedTriangleMesh::Vertex> nearest = frontier[minPos];

    nearest->addFlag(SemantisedTriangleMesh::FlagType::VISITED);

    frontier.erase(frontier.begin() + minPos);

    return nearest;

}

std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > Utilities::dijkstra(std::shared_ptr<SemantisedTriangleMesh::Vertex> v1,
                                                                                  std::shared_ptr<SemantisedTriangleMesh::Vertex> v2){

    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> frontier;
    std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, double> distances = {{v1, 0}};
    std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, std::shared_ptr<SemantisedTriangleMesh::Vertex>> predecessors = {{v1, nullptr}};
    std::set<SemantisedTriangleMesh::Vertex*> v21RingNeighbors;
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > shortestPath, visitedVertices;
    std::shared_ptr<SemantisedTriangleMesh::Vertex> v;
    bool v2visited = false;

    if((*v1) == (*v2)){
        return shortestPath;
    }

    frontier.push_back(v1);

    do{

        v = extractNearestVertex(frontier, distances);
        if(v == nullptr)
        {
            for(auto v_ : visitedVertices)
                v_->removeFlag(SemantisedTriangleMesh::FlagType::VISITED);
            return shortestPath;
        }

        visitedVertices.push_back(v);
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> neighbors = v->getVV();
        for(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> >::iterator n = neighbors.begin(); n != neighbors.end(); n++){
            std::shared_ptr<SemantisedTriangleMesh::Vertex> x = *n;

            if(x->getId() == v2->getId() && v21RingNeighbors.find(&(*v)) == v21RingNeighbors.end())
                v21RingNeighbors.insert(&(*v));

            std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, std::shared_ptr<SemantisedTriangleMesh::Vertex>>::iterator pit = predecessors.find(x);
            double distanceVX;

            distanceVX = distances[v] + (*x - *v).norm();


            if(pit != predecessors.end()){
                if(distances[x] > distanceVX){
                    distances[x] = distanceVX;
                    predecessors[x] = v;
                }
            } else {
                distances.insert(std::make_pair(x, distanceVX));
                predecessors.insert(std::make_pair(x, v));
                if(!v2visited)
                    frontier.push_back(x);
            }
        }
        if(v->getId() == v2->getId())
            v2visited = true;

    } while(!v2visited);

    shortestPath.push_back(v2);
    v = predecessors[v2];
    while(v != v1){
        shortestPath.insert(shortestPath.begin(), v);
        v = predecessors[v];
    }

    for(auto v : visitedVertices)
        v->removeFlag(SemantisedTriangleMesh::FlagType::VISITED);

    return shortestPath;
}

std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > Utilities::polylineDijkstra(std::shared_ptr<SemantisedTriangleMesh::Vertex> v1,
                                                                                          std::shared_ptr<SemantisedTriangleMesh::Vertex> v2, bool optimize){

    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> frontier;
    std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, double> distances = {{v1, 0}};
    std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, std::shared_ptr<SemantisedTriangleMesh::Vertex>> predecessors = {{v1, nullptr}};
    std::set<SemantisedTriangleMesh::Vertex*> v21RingNeighbors;
    std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > shortestPath, visitedVertices;
    std::shared_ptr<SemantisedTriangleMesh::Vertex> v;
    bool v2visited = false;

    if((*v1) == (*v2)){
        return shortestPath;
    }

    frontier.push_back(v1);

    do{

        v = extractNearestVertex(frontier, distances);
        if(v == nullptr)
        {
            for(auto v_ : visitedVertices)
                v_->removeFlag(SemantisedTriangleMesh::FlagType::VISITED);
            return shortestPath;
        }

        visitedVertices.push_back(v);
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex>> neighbors = v->getVV();
        for(std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> >::iterator n = neighbors.begin(); n != neighbors.end(); n++){
            std::shared_ptr<SemantisedTriangleMesh::Vertex> x = *n;

            if(x->getId() == v2->getId() && v21RingNeighbors.find(&(*v)) == v21RingNeighbors.end())
                v21RingNeighbors.insert(&(*v));

            std::map<std::shared_ptr<SemantisedTriangleMesh::Vertex>, std::shared_ptr<SemantisedTriangleMesh::Vertex>>::iterator pit = predecessors.find(x);
            double distanceVX;

            distanceVX = distances[v] + (*x - *v).norm();


            if(pit != predecessors.end()){
                if(distances[x] > distanceVX){
                    distances[x] = distanceVX;
                    predecessors[x] = v;
                }
            } else {
                distances.insert(std::make_pair(x, distanceVX));
                predecessors.insert(std::make_pair(x, v));
                if(!v2visited && (x->searchFlag(SemantisedTriangleMesh::FlagType::INSIDE) >= 0 || !optimize))
                    frontier.push_back(x);
            }
        }
        if(v->getId() == v2->getId())
            v2visited = true;

    } while(!v2visited);

    shortestPath.push_back(v2);
    v = predecessors[v2];
    while(v != v1){
        shortestPath.insert(shortestPath.begin(), v);
        v = predecessors[v];
    }

    for(auto v : visitedVertices)
        v->removeFlag(SemantisedTriangleMesh::FlagType::VISITED);

    return shortestPath;
}

bool Utilities::checkOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> p, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Vertex> > > polygons)
{
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 1; j < polygons.at(i).size(); j++)
        {
            std::shared_ptr<SemantisedTriangleMesh::Point> p1 = polygons.at(i).at(j - 1);
            std::shared_ptr<SemantisedTriangleMesh::Point> p2 = polygons.at(i).at(j);
            if(Utilities::isPointInSegment(p1.get(), p2.get(), p.get()))
            {
                return true;
            }
        }
    }
    return false;

}

bool Utilities::checkOnBoundary(std::shared_ptr<SemantisedTriangleMesh::Point> p, std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > polygons)
{
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 1; j < polygons.at(i).size(); j++)
        {
            std::shared_ptr<SemantisedTriangleMesh::Point> p1 = polygons.at(i).at(j - 1);
            std::shared_ptr<SemantisedTriangleMesh::Point> p2 = polygons.at(i).at(j);
            if(Utilities::isPointInSegment(p1.get(), p2.get(), p.get()))
            {
                return true;
            }
        }
    }
    return false;

}

void Utilities::removeLinePointsInPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                           std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > polygons)
{
    pointVec points_vector;
    std::map<unsigned int, unsigned int > pointPolygonLink;
    unsigned int id = 0;
    double meanDiagonal = 0;
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 0; j < polygons[i].size() - 1; j++)
            for(unsigned int k = 0; k < polygons.at(i).at(j).size(); k++)
            {

                std::vector<double> point = { polygons.at(i).at(j).at(k)->getX(),
                                              polygons.at(i).at(j).at(k)->getY(),
                                              polygons.at(i).at(j).at(k)->getZ()};
                points_vector.push_back(point);
                pointPolygonLink.insert(std::make_pair(id++, i));
            }
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(polygons.at(i).at(0));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
    }
    meanDiagonal /= polygons.size();

    KDTree tree(points_vector);
    unsigned int counter = 0;

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < lines.size(); i++)
    {
        for(unsigned int j = 0; j < lines[i].size(); j++){
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v = lines.at(i).at(j);
            point_t query_point = {v->getX(), v->getY(), v->getZ()};
            std::vector<size_t> neighbors_indices = tree.neighborhood_indices(query_point, meanDiagonal / 2);
            std::vector<unsigned int> neighboringPolygons;
            for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
                std::vector<unsigned int>::iterator pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(*it));
            }
            for(unsigned int k = 0; k < neighboringPolygons.size(); k++)
            {
                if(isPointInsidePolygon(v, polygons.at(neighboringPolygons.at(k)).at(0)))
                {
                    bool inside = false;
                    for(unsigned int l = 1; l < polygons.at(neighboringPolygons.at(k)).size(); l++)
                    {
                        if(isPointInsidePolygon(v, polygons.at(neighboringPolygons.at(k)).at(l)))
                        {
                            inside = true;
                            break;
                        }
                    }
                    bool onBoundary = checkOnBoundary(v, polygons.at(neighboringPolygons.at(k)));

                    if(!inside && !onBoundary)
                    {
                        lines.at(i).erase(lines.at(i).begin() + j);
                        v.reset();
                        j--;
                        break;
                    }
                }
            }
        }

#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
        }
    }
    for(unsigned int i = 0; i < lines.size(); i++)
        if(lines.at(i).size() < 2)
        {
            lines.erase(lines.begin() + i);
            i--;
        }
    std::cout << std::endl;
}

void Utilities::removeLinePointsInPolygons(std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
                                           std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > polygons)
{
    pointVec points_vector;
    std::map<unsigned int, unsigned int > pointPolygonLink;
    unsigned int id = 0;
    double meanDiagonal = 0;
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 0; j < polygons[i].size() - 1; j++){
            std::vector<double> point = { polygons.at(i).at(j)->getX(),
                                          polygons.at(i).at(j)->getY(),
                                          polygons.at(i).at(j)->getZ()};
            points_vector.push_back(point);
            pointPolygonLink.insert(std::make_pair(id++, i));
        }
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(polygons[i]);
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
    }
    meanDiagonal /= polygons.size();

    KDTree tree(points_vector);
    unsigned int counter = 0;

#pragma omp parallel for num_threads(31)
    for(unsigned int i = 0; i < lines.size(); i++)
    {
        for(unsigned int j = 0; j < lines[i].size(); j++){
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v = lines.at(i).at(j);
            point_t query_point = {v->getX(), v->getY(), v->getZ()};
            std::vector<size_t> neighbors_indices = tree.neighborhood_indices(query_point, meanDiagonal / 2);
            std::vector<unsigned int> neighboringPolygons;
            for(std::vector<size_t>::iterator it = neighbors_indices.begin(); it != neighbors_indices.end(); it++){
                std::vector<unsigned int>::iterator pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(*it));
            }
            for(unsigned int k = 0; k < neighboringPolygons.size(); k++)
            {
                if(isPointInsidePolygon(v, polygons.at(neighboringPolygons.at(k))))
                {
                    bool onBoundary = false;
                    for(unsigned int l = 1; l < polygons.at(neighboringPolygons.at(k)).size(); l++)
                    {
                        std::shared_ptr<SemantisedTriangleMesh::Point> p1 = polygons.at(neighboringPolygons.at(k)).at(l - 1);
                        std::shared_ptr<SemantisedTriangleMesh::Point> p2 = polygons.at(neighboringPolygons.at(k)).at(l);
                        if(Utilities::isPointInSegment(p1.get(), p2.get(), v.get()))
                        {
                            onBoundary = true;
                            break;
                        }
                    }
                    if(!onBoundary)
                    {
                        lines.at(i).erase(lines.at(i).begin() + j);
                        v.reset();
                        j--;
                        break;
                    }
                }
            }
        }

#pragma omp critical
        {
            counter++;
            std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
        }
    }
    for(unsigned int i = 0; i < lines.size(); i++)
        if(lines.at(i).size() < 2)
        {
            lines.erase(lines.begin() + i);
            i--;
        }
    std::cout << std::endl;

}

std::vector<std::pair<unsigned int, unsigned int> > Utilities::removeSegmentsIntersectingPolygons(
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
        std::vector<std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > > &polygons)
{
    std::vector<std::pair<unsigned int, unsigned int>> keptClippings;
    pointVec points_vector;
    std::map<unsigned int, unsigned int > pointPolygonLink;
    unsigned int id = 0, counter = 0;
    double meanDiagonal = 0;
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 0; j < polygons[i].size() - 1; j++)
            for(unsigned int k = 0; k < polygons.at(i).at(j).size(); k++)
            {

                std::vector<double> point = { polygons.at(i).at(j).at(k)->getX(),
                                              polygons.at(i).at(j).at(k)->getY(),
                                              polygons.at(i).at(j).at(k)->getZ()};
                points_vector.push_back(point);
                pointPolygonLink.insert(std::make_pair(id++, i));
            }
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(polygons.at(i).at(0));
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
    }
    meanDiagonal /= polygons.size();

    KDTree tree(points_vector);

    for(unsigned int i = 0; i < lines.size(); i++)
    {
        unsigned int deviation = 0;
        for(unsigned int j = 1; j < lines[i].size(); j++){
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v = lines.at(i).at(j);
            std::vector<unsigned int> neighboringPolygons;
            point_t query_point = {v->getX(), v->getY(), v->getZ()};
            std::vector<size_t> neighbors_distances = tree.neighborhood_indices(query_point, meanDiagonal / 2);
            for(std::vector<size_t>::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
                std::vector<unsigned int>::iterator pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(*it));
            }

            for(unsigned int k = 0; k < neighboringPolygons.size(); k++)
            {
                std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > polygon = polygons.at(neighboringPolygons.at(k));
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
                bool found = false;
                for(unsigned int l = 0; l < polygon.size(); l++)
                {
                    for(unsigned int m = 0; m < polygon.at(l).size(); m++)
                    {
                        std::shared_ptr<SemantisedTriangleMesh::Point> pout = std::make_shared<SemantisedTriangleMesh::Point>();
                        if(segmentsIntersection(lines.at(i).at(j - 1), lines.at(i).at(j), polygon.at(l).at(m), polygon.at(l).at((m + 1) % polygon.at(l).size()), *pout))
                        {
                            auto it = std::find_if(intersections.begin(), intersections.end(), [pout](std::shared_ptr<SemantisedTriangleMesh::Point> p) { return *p == *pout; });
                            if(it == intersections.end())
                            {
                                polygon.at(l).insert(polygon.at(l).begin() + m + 1, pout);
                                m++;
                                intersections.push_back(pout);
                            }
                        }
                    }

                }
                if(intersections.size() > 1)
                {

                    std::sort(intersections.begin(), intersections.end(),
                              [lines, i, j](std::shared_ptr<SemantisedTriangleMesh::Point> p1, std::shared_ptr<SemantisedTriangleMesh::Point> p2)
                    {
                        return ((*p1) - (*lines.at(i).at(j))).norm() < ((*p2) - (*lines.at(i).at(j))).norm();
                    });

                    if(intersections.size() % 2 != 0)
                    {
                        //Da gestire caso in cui segmento traversa uno "spuntone" di poligono e poi "tocca" una punta (tecnicamente sono 3 punti).
                        //Al momento si ignora il caso, eliminando l'ultima intersezione e considerandolo come un caso con numero pari di intersezioni
                        intersections.pop_back();
                    }

                    unsigned int firstPos = intersections.size() - 1, lastPos = 0;
                    if((*intersections.at(intersections.size() - 1) - *lines.at(i).at(j - 1)).norm() < (*intersections.at(0) - *lines.at(i).at(j - 1)).norm())
                    {
                        firstPos = 0;
                        lastPos = intersections.size() - 1;
                    }
                    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > remaining(lines.at(i).begin() + j, lines.at(i).end());

                    lines.insert(lines.begin() + i + 1, remaining);
                    lines.at(i + 1).insert(lines.at(i + 1).begin(), intersections.at(firstPos));
                    lines.at(i).erase(lines.at(i).begin() + j, lines.at(i).end());
                    lines.at(i).push_back(intersections.at(lastPos));
                    deviation = intersections.size() / 2;
                    keptClippings.push_back(std::make_pair(i, i + deviation));
                    for(unsigned int l = 1; l < intersections.size() / 2; l++)
                    {
                        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > exteriorSegment = {intersections.at(l * 2 - 1), intersections.at(l * 2)};
                        lines.insert(lines.begin() + i + 1, exteriorSegment);
                    }

                }
            }

            i += deviation;
        }

        counter++;
        std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
    }

    std::cout << std::endl;
    return keptClippings;
}

std::vector<std::pair<unsigned int, unsigned int> > Utilities::removeSegmentsIntersectingPolygons(
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &lines,
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > &polygons)
{
    std::vector<std::pair<unsigned int, unsigned int>> keptClippings;
    pointVec points_vector;
    std::map<unsigned int, unsigned int > pointPolygonLink;
    unsigned int id = 0, counter = 0;
    double meanDiagonal = 0;
    for(unsigned int i = 0; i < polygons.size(); i++)
    {
        for(unsigned int j = 0; j < polygons[i].size() - 1; j++){
            std::vector<double> point = { polygons.at(i).at(j)->getX(),
                                          polygons.at(i).at(j)->getY(),
                                          polygons.at(i).at(j)->getZ()};
            points_vector.push_back(point);
            pointPolygonLink.insert(std::make_pair(id++, i));
        }
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > bb = bbExtraction(polygons[i]);
        double diagMeasure = (*bb[2] - *bb[0]).norm();
        meanDiagonal += diagMeasure;
    }
    meanDiagonal /= polygons.size();

    KDTree tree(points_vector);

    for(unsigned int i = 0; i < lines.size(); i++)
    {
        unsigned int deviation = 0;
        for(unsigned int j = 1; j < lines[i].size(); j++){
            std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
            std::shared_ptr<SemantisedTriangleMesh::Point> v = lines.at(i).at(j);
            std::vector<unsigned int> neighboringPolygons;
            point_t query_point = {v->getX(), v->getY(), v->getZ()};
            std::vector<size_t> neighbors_distances = tree.neighborhood_indices(query_point, meanDiagonal / 2);
            for(std::vector<size_t>::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
                std::vector<unsigned int>::iterator pit = std::find(neighboringPolygons.begin(), neighboringPolygons.end(), pointPolygonLink.at(*it));
                if(pit == neighboringPolygons.end())
                    neighboringPolygons.push_back(pointPolygonLink.at(*it));
            }

            for(unsigned int k = 0; k < neighboringPolygons.size(); k++)
            {
                std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > intersections;
                for(unsigned int l = 0; l < polygons.at(neighboringPolygons.at(k)).size(); l++)
                {
                    std::shared_ptr<SemantisedTriangleMesh::Point> pout = std::make_shared<SemantisedTriangleMesh::Point>();
                    if(segmentsIntersection(lines.at(i).at(j - 1), lines.at(i).at(j), polygons.at(neighboringPolygons.at(k)).at(l), polygons.at(neighboringPolygons.at(k)).at((l + 1) % polygons.at(neighboringPolygons.at(k)).size()), *pout))
                    {
                        polygons.at(neighboringPolygons.at(k)).insert(polygons.at(neighboringPolygons.at(k)).begin() + l + 1, pout);
                        l++;
                        intersections.push_back(pout);
                    }

                }
                if(intersections.size() > 1)
                {

                    if(intersections.size() % 2 != 0)
                    {
                        //Da gestire caso in cui segmento traversa uno "spuntone" di poligono e poi "tocca" una punta (tecnicamente sono 3 punti).
                        //Al momento si ignora il caso, eliminando l'ultima intersezione e considerandolo come un caso con numero pari di intersezioni
                        intersections.pop_back();
                    }

                    unsigned int firstPos = intersections.size() - 1, lastPos = 0;
                    if((*intersections.at(intersections.size() - 1) - *lines.at(i).at(j - 1)).norm() < (*intersections.at(0) - *lines.at(i).at(j - 1)).norm())
                    {
                        firstPos = 0;
                        lastPos = intersections.size() - 1;
                    }
                    std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > remaining(lines.at(i).begin() + j, lines.at(i).end());
                    lines.insert(lines.begin() + i + 1, remaining);
                    lines.at(i + 1).insert(lines.at(i + 1).begin(), intersections.at(firstPos));
                    lines.at(i).erase(lines.at(i).begin() + j, lines.at(i).end());
                    lines.at(i).push_back(intersections.at(lastPos));
                    deviation = intersections.size() / 2;
                    keptClippings.push_back(std::make_pair(i, i + deviation));
                    for(unsigned int l = 1; l < intersections.size() / 2; l++)
                    {
                        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > exteriorSegment = {intersections.at(l * 2 - 1), intersections.at(l * 2)};
                        lines.insert(lines.begin() + i + 1, exteriorSegment);
                    }

                }
            }

            i += deviation;
        }

        counter++;
        std::cout << counter * 100 / lines.size() << "%\r" << std::flush;
    }

    std::cout << std::endl;
    return keptClippings;
}

std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > Utilities::getClosest(
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > list,
        std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > p)
{
    pointVec points_vector;
    std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > closest_points;
    std::map<unsigned int, std::shared_ptr<SemantisedTriangleMesh::Point> > points_pos;
    unsigned int counter = 0;
    for(unsigned int i = 0; i < list.size(); i++)
    {
        for(unsigned int j = 0; j < list[i].size() - 1; j++){
            std::vector<double> point = { list.at(i).at(j)->getX(),
                                          list.at(i).at(j)->getY(),
                                          list.at(i).at(j)->getZ()};
            points_vector.push_back(point);
            points_pos.insert(std::make_pair(counter++, list.at(i).at(j)));
        }
    }

    KDTree tree(points_vector);
    for(unsigned int i = 0; i < p.size(); i++)
    {
        std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > closest;
        for(unsigned int j = 0; j < p.at(i).size(); j++)
        {
            point_t point = {p.at(i).at(j)->getX(), p.at(i).at(j)->getY(), p.at(i).at(j)->getZ()};
            size_t ret_index = tree.nearest_index(point);
            closest.push_back(points_pos.at(ret_index));
        }
        closest_points.push_back(closest);
    }
    return closest_points;
}

void Utilities::interpolate(const unsigned char color1[], const unsigned char color2[], float fraction, unsigned char (&color)[3])
{
    unsigned char r1 = color1[0];
    unsigned char r2 = color2[0];
    unsigned char g1 = color1[1];
    unsigned char g2 = color2[1];
    unsigned char b1 = color1[2];
    unsigned char b2 = color2[2];
    color[0] = (r2 - r1) * fraction + r1;
    color[1] = (g2 - g1) * fraction + g1;
    color[2] = (b2 - b1) * fraction + b1;
}


std::vector<std::string> Utilities::globVector(const std::string &pattern)
{
    glob_t glob_result;
    glob(pattern.c_str(),GLOB_TILDE,nullptr, &glob_result);
    std::vector<std::string> files;
    for(unsigned int i = 0; i < glob_result.gl_pathc;++i)
        files.push_back(std::string(glob_result.gl_pathv[i]));

    globfree(&glob_result);
    return files;
}

bool Utilities::isPointOutsideBounds(std::shared_ptr<SemantisedTriangleMesh::Point> p,  std::vector<std::vector<std::shared_ptr<SemantisedTriangleMesh::Point> > > bounds)
{
    bool outside = true, inside = false;
    outside = !isPointInsidePolygon(p, bounds[0]);
    for(uint i = 1; i < bounds.size(); i++)
    {
        if(isPointInsidePolygon(p, bounds[i]))
        {
            return true;
        }
    }
    return outside;
}
