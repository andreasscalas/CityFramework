#include <Vertex.h>
#include <algorithm>

Vertex::Vertex()
{
    x = -std::numeric_limits<double>::min();
    y = -std::numeric_limits<double>::min();
    z = -std::numeric_limits<double>::min();
    e0 = nullptr;
    id = "";
    associated_flags.clear();
}

Vertex::Vertex(double x, double y, double z) : Point(x,y,z)
{
    e0 = nullptr;
    id = "";
    associated_flags.clear();
}

Vertex::Vertex(double *p) : Point(p[0], p[1], p[2])
{
    e0 = nullptr;
    id = "";
    associated_flags.clear();
}

Vertex::Vertex(Point p) : Point(p.getX(), p.getY(), p.getZ())
{
    id = "";
    associated_flags.clear();
}

Vertex::Vertex(std::shared_ptr<Point> p) : Point(p->getX(), p->getY(), p->getZ())
{
    info = p->getInfo();
    e0 = nullptr;
    id = "";
    associated_flags.clear();
}

Vertex::Vertex(std::shared_ptr<Vertex> v) : Point(v->getX(), v->getY(), v->getZ())
{
    this->e0 = v->getE0();
    this->id = v->getId();

    this->associated_flags.insert(this->associated_flags.end(), v->getAssociatedFlags().begin(), v->getAssociatedFlags().end());
    this->information.insert(this->information.end(), v->getInformation().begin(), v->getInformation().end());

}


Vertex::~Vertex()
{
    e0.reset();


}

std::shared_ptr<Edge> Vertex::getCommonEdge(std::shared_ptr<Vertex> v)
{
    std::vector<std::shared_ptr<Edge> > ve = this->getVE();
    for(unsigned int i = 0; i < ve.size(); i++)
        if(ve.at(i)->hasVertex(v))
            return ve.at(i);

    ve = v->getVE();
    for(unsigned int i = 0; i < ve.size(); i++)
        if(ve.at(i)->hasVertex(this))
            return ve.at(i);
    return nullptr;

}

Point Vertex::computeNormal()
{
    Point normal(0, 0, 0);
    std::vector<std::shared_ptr<Triangle>> vt = this->getVT();
    for(unsigned int i = 0; i < vt.size(); i++)
        normal += vt.at(i)->computeNormal();

    normal /= vt.size();

    return normal;
}

std::vector<std::shared_ptr<Vertex> > Vertex::getNeighbourhood(uint size)
{

    std::vector<std::shared_ptr<Vertex> > neighbourhood;
    if(size == 0)
        return neighbourhood;
    neighbourhood = this->getVV();
    if(neighbourhood.size() == 0)
        return neighbourhood;
    this->addFlag(FlagType::USED);
    for(auto v : neighbourhood)
        v->addFlag(FlagType::USED);

    for(uint i = 1; i < size; i++)
    {
        uint startingSetSize = neighbourhood.size();
        for(uint j = 0; j < startingSetSize; j++)
        {
            auto v = neighbourhood[j];
            auto vv = v->getVV();
            for(auto v_ : vv)
                if(v_->searchFlag(FlagType::USED) < 0)
                {
                    v_->addFlag(FlagType::USED);
                    neighbourhood.push_back(v_);
                }
        }

    }
    this->removeFlag(FlagType::USED);
    for(auto v : neighbourhood)
        v->removeFlag(FlagType::USED);

    return neighbourhood;

}

bool Vertex::addFlag(FlagType f)
{
    if(std::find(associated_flags.begin(), associated_flags.end(), f) == associated_flags.end())
    {
        associated_flags.push_back(f);
        return true;
    }
    return false;
}

int Vertex::searchFlag(FlagType f)
{
    std::vector<FlagType>::iterator it = std::find(associated_flags.begin(), associated_flags.end(), f);
    if(it != associated_flags.end())
        return  it - associated_flags.begin();
    return -1;
}

bool Vertex::removeFlag(FlagType f)
{
    int flagPosition = searchFlag(f);
    if(flagPosition >= 0)
    {
        associated_flags.erase(associated_flags.begin() + flagPosition);
        return true;
    }
    return false;

}

bool Vertex::removeFlag(unsigned int position)
{
    if(position < associated_flags.size())
    {
        associated_flags.erase(associated_flags.begin() + position);
        return true;
    }
    return false;

}

bool Vertex::addInformation(void *info)
{
    if(std::find(information.begin(), information.end(), info) == information.end())
    {
        information.push_back(info);
        return true;
    }
    return false;
}

int Vertex::searchInfo(void * info)
{
    std::vector<void*>::iterator it = std::find(information.begin(), information.end(), info);
    if(it != information.end())
        return  it - information.begin();
    return -1;
}

bool Vertex::removeInfo(void *info)
{
    int flagPosition = searchInfo(info);
    if(flagPosition >= 0)
    {
        information.erase(information.begin() + flagPosition);
        return true;
    }
    return false;
}

bool Vertex::removeInfo(unsigned int position)
{
    if(position < information.size())
    {
        information.erase(information.begin() + position);
        return true;
    }
    return false;
}

bool Vertex::clearInfo()
{
    information.clear();
    return true;
}

std::string Vertex::getId() const
{
    return id;
}

void Vertex::setId(std::string newId)
{
    id = newId;
}

void Vertex::print(std::ostream &stream)
{
    stream << "Vertex with id: " << id << " and coordinates: ";
    static_cast<Point*>(this)->print(stream, BracketsType::SQUARE, ",");
}

const std::vector<void *> &Vertex::getInformation() const
{
    return information;
}

void Vertex::setInformation(const std::vector<void *> &newInformation)
{
    information = newInformation;
}

const std::vector<FlagType> &Vertex::getAssociatedFlags() const
{
    return associated_flags;
}

void Vertex::setAssociatedFlags(const std::vector<FlagType> &newAssociatedFlags)
{
    associated_flags = newAssociatedFlags;
}

std::shared_ptr<Edge> Vertex::getE0() const
{
    return e0;
}

void Vertex::setE0(std::shared_ptr<Edge> newE0)
{
    e0 = newE0;
}

std::vector<std::shared_ptr<Vertex> > Vertex::getVV()
{
    std::vector<std::shared_ptr<Vertex> > vv;
    std::vector<std::shared_ptr<Edge> > ve = getVE();
    for(unsigned int i = 0; i < ve.size(); i++)
        vv.push_back(ve.at(i)->getOppositeVertex(std::make_shared<Vertex>(*this)));
    return vv;
}

std::vector<std::shared_ptr<Edge> > Vertex::getVE()
{
    std::vector<std::shared_ptr<Edge> > ve;
    std::shared_ptr<Triangle> t;
    std::shared_ptr<Edge> e = e0;
    std::shared_ptr<Vertex> v;

    if (e == nullptr) return ve;

    do
    {
        if(e == nullptr)
            break;
        ve.push_back(e);
        v = e->getOppositeVertex(std::make_shared<Vertex>(*this));
        t = e->getLeftTriangle(std::make_shared<Vertex>(*this));
        if (t == nullptr)
            break;
        e = t->getOppositeEdge(v);
    } while (e->getId().compare(e0->getId()) != 0);

    if (e != nullptr && e->getId().compare(e0->getId()) == 0 && ve.size() > 1) return ve;

    e = e0;
    do
    {
        v = e->getOppositeVertex(std::make_shared<Vertex>(*this));
        t = e->getRightTriangle(std::make_shared<Vertex>(*this));
        if (t == nullptr) break;
        e = t->getOppositeEdge(v);
        if(e == nullptr)
            break;
        ve.insert(ve.begin(), e);
    } while (e->getId().compare(e0->getId()) != 0);

    return ve;


}

std::vector<std::shared_ptr<Triangle> > Vertex::getVT()
{
    std::vector<std::shared_ptr<Triangle> > vt;
    std::vector<std::shared_ptr<Edge>> ve = getVE();
    for(unsigned int i = 0; i < ve.size(); i++)
        vt.push_back(ve.at(i)->getLeftTriangle(std::make_shared<Vertex>(*this)));
    return vt;
}
