#include <iostream>
#include <fstream>
#include <TriangleMesh.h>
#include <trianglehelper.h>
#include <quadtree.h>
const int MAX_VALUE = 10000;
int main(int argc, char* argv[])
{
    std::vector<Point*> points;
    bool ended = false;
    double x, y;
    srand(time(nullptr));
    for(uint i = 0; i < MAX_VALUE; i++)
    {
        x = rand() % MAX_VALUE - MAX_VALUE / 2;
        y = rand() % MAX_VALUE - MAX_VALUE / 2;
        auto point = new Point(x, y, 0);
        points.push_back(point);

    }

    std::vector<Node<void*>* > nodes;
    for(auto point : points)
    {
        auto n = new Node<void*>(*point, static_cast<void*>(nullptr));
        nodes.push_back(n);
    }

    auto root = new QuadTree<void*>();
    root->setTopLeft(Point(-MAX_VALUE / 2,MAX_VALUE / 2,0));
    root->setWidth(MAX_VALUE);
    root->setHeight(MAX_VALUE);
    root->setMaxNodes(4);
    std::ofstream stream("prova.txt");
    stream << "A=[" << std::endl;
    for(auto node : nodes)
    {
        node->getPos().print(stream, BracketsType::NONE, " ");
        root->insert(node);
    }
    stream << "];" << std::endl;
    stream << std::endl;
    root->drawForMatlab(stream);
    stream << std::endl;
    do
    {
        std::cout << "Insert query point coordinates separated with a space or press q and enter to close application" << std::endl;
        std::cin >> x;
        std::cin >> y;
        if(std::cin.good())
        {
            auto point = new Point(x, y, 0);
            auto node = new Node<void*>(*point,  static_cast<void*>(nullptr));
            auto closest = root->getClosest(node);
            closest->getPos().print(std::cout);
            std::cout << std::endl;
            delete node;
        } else
            break;

    } while(true);

    for(auto point : points)
        delete point;
    for(auto node : nodes)
        delete node;
    delete root;

//    if(argc < 2)
//        return -1;

//    std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
////    mesh->load(argv[1]);
////    mesh->save("prova.ply");

//    std::vector<std::vector<std::shared_ptr<Point>> > boundaries;
//    std::vector<std::vector<std::shared_ptr<Point>> > constraints;
//    std::vector<std::shared_ptr<Point>> boundary = {
//        std::make_shared<Point>(0,-3,0),
//        std::make_shared<Point>(3,0,0),
//        std::make_shared<Point>(0,3,0),
//        std::make_shared<Point>(-3,0,0)
//    };
//    boundaries.push_back(boundary);
//    std::vector<std::shared_ptr<Point>> hole = {
//        std::make_shared<Point>(-1,-1,0),
//        std::make_shared<Point>(-1,1,0),
//        std::make_shared<Point>(1,1,0),
//        std::make_shared<Point>(1,-1,0)
//    };
//    boundaries.push_back(hole);

//    std::vector<std::shared_ptr<Point>> constraint = {
//        std::make_shared<Point>(-0.5,0.5,0),
//        std::make_shared<Point>(-0.5,-0.5,0)
//    };
//    constraints.push_back(constraint);
//    mesh->triangulate(boundaries, constraints);
////    TriHelper::TriangleHelper helper(boundary, boundaries, constraints);
////    std::vector<unsigned int> triangles = helper.getTriangles();
////    for(unsigned int i = 0; i < triangles.size() / 3; i++)
////        std::cout << triangles[i * 3] << " " << triangles[i * 3 + 1] << " " << triangles[i * 3 + 2] << std::endl;
//    mesh->save("prova_triangulation.ply");
//    mesh.reset();
    return 0;
}
