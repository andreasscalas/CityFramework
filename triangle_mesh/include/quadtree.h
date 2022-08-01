#ifndef QUADTREE_H
#define QUADTREE_H

#include <Point.h>

#include <array>
#include <queue>
#include <vector>
#include <fstream>

template <class T>
class Node
{
    private:
        Point pos;
        T data;
    public:
        Node(Point _pos, T _data)
        {
            pos = _pos;
            data = _data;
        }
        Node()
        {
            data = 0;
        }

        T getData() const
        {
            return data;
        }

        void setData(T newData)
        {
            data = newData;
        }

        Point getPos() const
        {
            return pos;
        }

        void setPos(Point newPos)
        {
            pos = newPos;
        }

        bool operator==(Node<T> other)
        {
            return (this->getPos()) == (other.getPos());
        }

        bool operator!=(Node<T> other)
        {
            return !((*this) == other);
        }
};

template <class T>
class QuadTree
{
public:
    QuadTree()
    {
        nw = nullptr;
        ne = nullptr;
        sw = nullptr;
        se = nullptr;
        father = nullptr;
        topLeft = Point(0,0,0);
        width = 0;
        height = 0;
    }

    ~QuadTree()
    {
        if(nw != nullptr)
        {
            delete nw;
            delete ne;
            delete sw;
            delete se;
        }

    }

    inline Point getTopLeft() const
    {
        return topLeft;
    }

    inline void setTopLeft(Point newTopLeft)
    {
        topLeft = newTopLeft;
    }

    double getWidth() const{
        return width;
    }

    void setWidth(double newWidth)
    {
        width = newWidth;
    }

    double getHeight() const
    {
        return height;
    }

    void setHeight(double newHeight)
    {
        height = newHeight;
    }

    inline const std::vector<Node<T> *> &getNodes() const
    {
        return nodes;
    }

    inline void setNodes(const std::vector<Node<T> *> &newNodes)
    {
        nodes = newNodes;
    }

    inline int getMaxNodes() const{
        return max_nodes;
    }

    inline void setMaxNodes(int newMaxNodes)
    {
        max_nodes = newMaxNodes;
    }

    inline std::vector<QuadTree*> getSubTrees()
    {
        std::vector<QuadTree*> subtrees = {nw, ne, sw, se};
        return subtrees;
    }

    QuadTree *getFather() const
    {
        return father;
    }

    void setFather(QuadTree *newFather)
    {
        father = newFather;
    }

    inline bool isLeaf()
    {
        return nw == nullptr;
    }

    inline void subdivide()
    {
        double newWidth = width / 2;
        double newHeight = height / 2;
        Point newTopLeft(topLeft);
        nw = new QuadTree();
        ne = new QuadTree();
        sw = new QuadTree();
        se = new QuadTree();
        nw->setTopLeft(newTopLeft);
        nw->setWidth(newWidth);
        nw->setHeight(newHeight);
        nw->setFather(this);
        nw->setMaxNodes(max_nodes);
        newTopLeft.setX(topLeft.getX() + width / 2);
        ne->setTopLeft(newTopLeft);
        ne->setWidth(newWidth);
        ne->setHeight(newHeight);
        ne->setFather(this);
        ne->setMaxNodes(max_nodes);
        newTopLeft.setY(topLeft.getY() - height / 2);
        se->setTopLeft(newTopLeft);
        se->setWidth(newWidth);
        se->setHeight(newHeight);
        se->setFather(this);
        se->setMaxNodes(max_nodes);
        newTopLeft.setX(topLeft.getX());
        sw->setTopLeft(newTopLeft);
        sw->setWidth(newWidth);
        sw->setHeight(newHeight);
        sw->setFather(this);
        sw->setMaxNodes(max_nodes);

        for(auto node : nodes)
            if(nw->contains(node->getPos()))
                nw->insert(node);
            else if(ne->contains(node->getPos()))
                ne->insert(node);
            else if(sw->contains(node->getPos()))
                sw->insert(node);
            else if(se->contains(node->getPos()))
                se->insert(node);
        nodes.erase(nodes.begin() + 1, nodes.end());
    }

    inline bool insert(Node<T>* newNode)
    {
        if(!contains(newNode->getPos()))
        {
            return false;
        }

        auto nit = std::find_if(nodes.begin(), nodes.end(), [newNode](Node<T>* n){ return (*n) == (*newNode);});
        if(nw == nullptr && nit == nodes.end() && nodes.size() + 1 > max_nodes)
            subdivide();

        if(nw != nullptr)
        {
            bool retValue = false;
            if(nw->contains(newNode->getPos()))
            {
                retValue = nw->insert(newNode);
            } else if (ne->contains(newNode->getPos()))
            {
                retValue = ne->insert(newNode);
            } else if (sw->contains(newNode->getPos()))
            {
                retValue = sw->insert(newNode);
            } else if (se->contains(newNode->getPos()))
            {
                retValue = se->insert(newNode);
            } else
            {
                std::cerr << "Impossible case: the node is contained in the father quadrant but is contained in no sub-tree" << std::endl;
                return false;
            }
            if(retValue && this->nodes.size() == 0)
                this->nodes.push_back(newNode);
            return retValue;
        }

        auto it = std::find_if(nodes.begin(), nodes.end(), [newNode](Node<T>* n){ return ((*n) == (*newNode)); });
        if(it == nodes.end())
        {
            nodes.push_back(newNode);
            return true;
        }
        std::cerr << "Node already present in tree" << std::endl;
        return false;


    }

    inline bool contains(Point pos)
    {
        double xDelta = pos.getX() - topLeft.getX();
        double yDelta = pos.getY() - (topLeft.getY() - height);
        return xDelta >= 0 && xDelta < width && yDelta >= 0 && yDelta < height;
    }

    inline Node<T>* getClosest(Node<T>* queryNode)
    {
        std::vector<std::vector<QuadTree*> > E = {{this}};
        uint i = 1;
        do {
            double minDist = std::numeric_limits<double>::max();
            Node<T>* best = nullptr;
            for(uint j = 0; j < i; j++)
            {
                std::vector<QuadTree*> Ej = E[j];
                for(uint k = 0; k < Ej.size(); k++)
                {
                    auto reps = Ej[k]->getNodes();
                    double dist = (reps[0]->getPos() - queryNode->getPos()).norm();
                    if(*reps[0] != *queryNode && dist < minDist)
                    {
                        minDist = dist;
                        best = reps[0];
                    }
                }
            }

            std::vector<QuadTree*> Ei;
            for(uint j = 0; j < E[i - 1].size(); j++)
            {
                if(!E[i - 1][j]->isLeaf())
                {
                    auto sons = E[i - 1][j]->getSubTrees();
                    for(uint k = 0; k < sons.size(); k++)
                    {
                        if(sons[k]->getNodes().size() > 0)
                        {
                            double dist = (sons[k]->getNodes()[0]->getPos() - queryNode->getPos()).norm();
                            if(dist <= minDist)
                            {
                                Ei.push_back(sons[k]);
                            }
                        }
                    }
                }
            }
            E.push_back(Ei);

            i++;
        } while(E[i - 1].size() != 0);

        double minDist = std::numeric_limits<double>::max();
        Node<T>* best = nullptr;
        for(uint j = 0; j < E.size(); j++)
        {
            for(uint k = 0; k < E[j].size(); k++)
            {
                auto reps = E[j][k]->getNodes();
                double dist = (reps[0]->getPos() - queryNode->getPos()).norm();
                if(reps[0] != queryNode && dist < minDist)
                {
                    minDist = dist;
                    best = reps[0];
                }
            }
        }

        return best;
    }


    void drawForMatlab(std::ofstream &stream)
    {
        Point topRight(topLeft.getX() + width, topLeft.getY(), 0);
        Point bottomRight(topLeft.getX() + width, topLeft.getY() - height, 0);
        Point bottomLeft(topLeft.getX(), topLeft.getY() - height, 0);
        stream << "A=[" << std::endl;
        topLeft.print(stream, BracketsType::NONE, " ");
        topRight.print(stream, BracketsType::NONE, " ");
        bottomRight.print(stream, BracketsType::NONE, " ");
        bottomLeft.print(stream, BracketsType::NONE, " ");
        topLeft.print(stream, BracketsType::NONE, " ");
        stream << "];" << std::endl;
        stream << "plot(A(:,1),A(:,2));" << std::endl;
        if(nw != nullptr)
        {
            nw->drawForMatlab(stream);
            ne->drawForMatlab(stream);
            se->drawForMatlab(stream);
            sw->drawForMatlab(stream);
        }



    }
private:
    Point topLeft;
    double width, height;
    QuadTree<T>* father;
    QuadTree<T>* nw;
    QuadTree<T>* ne;
    QuadTree<T>* sw;
    QuadTree<T>* se;
    std::vector<Node<T>*> nodes;
    int max_nodes = 1;
};



#endif // QUADTREE_H
