#include "Edge.h"
#include <algorithm>

Edge::Edge()
{
    setId("-1");
    v1 = nullptr;
    v2 = nullptr;
    t1 = nullptr;
    t2 = nullptr;
}

Edge::Edge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    setId("-1");
    setV1(v1);
    setV2(v2);
    t1 = nullptr;
    t2 = nullptr;
}

Edge::Edge(std::shared_ptr<Edge> e)
{
    setId(e->getId());
    setV1(e->getV1());
    setV2(e->getV2());
    setT1(e->getT1());
    setT2(e->getT2());
    this->associated_flags.insert(this->associated_flags.end(), e->getAssociatedFlags().begin(), e->getAssociatedFlags().end());
    this->information.insert(this->information.end(), e->getInformation().begin(), e->getInformation().end());
}

Edge::~Edge()
{
    v1.reset();
    v2.reset();
    t1.reset();
    t2.reset();


}

std::shared_ptr<Vertex> Edge::getV1() const
{
    return v1;
}

void Edge::setV1(std::shared_ptr<Vertex> newV1)
{
    v1 = newV1;
}

std::shared_ptr<Vertex> Edge::getV2() const
{
    return v2;
}

void Edge::setV2(std::shared_ptr<Vertex> newV2)
{
    v2 = newV2;
}

bool Edge::setVertex(std::shared_ptr<Vertex> v, std::shared_ptr<Vertex> newV)
{
    if(v1->getId().compare(v->getId()) == 0)
    {
        setV1(newV);
        return true;
    } else if (v2->getId().compare(v->getId()) == 0)
    {
        setV2(newV);
        return true;
    }
    return false;
}

const std::vector<void *> &Edge::getInformation() const
{
    return information;
}

void Edge::setInformation(const std::vector<void *> &newInformation)
{
    information = newInformation;
}

const std::vector<FlagType> &Edge::getAssociatedFlags() const
{
    return associated_flags;
}

void Edge::setAssociatedFlags(const std::vector<FlagType> &newAssociatedFlags)
{
    associated_flags = newAssociatedFlags;
}

std::shared_ptr<Vertex> Edge::getCommonVertex(std::shared_ptr<Edge> e)
{
    if(e->hasVertex(v1))
        return v1;
    if(e->hasVertex(v2))
        return v2;
    return nullptr;

}

std::shared_ptr<Vertex> Edge::getOppositeVertex(std::shared_ptr<Vertex> v)
{
    if(v1 != nullptr && v1->getId().compare( v->getId()) == 0)
        return v2;
    if(v2 != nullptr && v2->getId().compare( v->getId()) == 0)
        return v1;

    return nullptr;
}

std::shared_ptr<Triangle> Edge::getLeftTriangle(std::shared_ptr<Vertex>  v)
{
    std::shared_ptr<Vertex>  v_;
    std::shared_ptr<Vertex>  v__ = this->getOppositeVertex(v);
    if(t1 != nullptr)
    {
        v_ = t1->getNextVertex(v);
        if(v_->getId().compare( v__->getId()) == 0)
            return t1;
    }
    if(t2 != nullptr)
    {
        v_ = t2->getNextVertex(v);
        if(v_->getId().compare( v__->getId()) == 0)
            return t2;
    }

    return nullptr;

}

std::shared_ptr<Triangle> Edge::getRightTriangle(std::shared_ptr<Vertex>  v)
{

    if(t1 != nullptr && t1->getPreviousVertex(v)->getId().compare( this->getOppositeVertex(v)->getId()) == 0)
        return t1;
    if(t2 != nullptr && t2->getPreviousVertex(v)->getId().compare( this->getOppositeVertex(v)->getId()) == 0)
        return t2;

    return nullptr;

}

std::shared_ptr<Triangle> Edge::getOppositeTriangle(std::shared_ptr<Triangle> t)
{
    if(t->getId().compare( t1->getId()) == 0)
        return t2;
    if(t->getId().compare( t2->getId()) == 0)
        return t1;
    return nullptr;
}

bool Edge::hasVertex(std::shared_ptr<Vertex>  v)
{
    return ((v1 != nullptr && v1->getId().compare( v->getId()) == 0) ||
            (v2 != nullptr && v2->getId().compare( v->getId())) == 0);
}

bool Edge::hasVertex(Vertex *v)
{
    return ((v1 != nullptr && v1->getId().compare( v->getId()) == 0) ||
            (v2 != nullptr && v2->getId().compare( v->getId())) == 0);
}


std::shared_ptr<Triangle> Edge::getT1() const
{
    return t1;
}

void Edge::setT1(std::shared_ptr<Triangle> newT1)
{
    t1 = newT1;
}

std::shared_ptr<Triangle> Edge::getT2() const
{
    return t2;
}

void Edge::setT2(std::shared_ptr<Triangle> newT2)
{
    t2 = newT2;
}

bool Edge::setTriangle(std::shared_ptr<Triangle> t, std::shared_ptr<Triangle> newT)
{
    if(t1 != nullptr && t1->getId().compare(t->getId()) == 0)
    {
        setT1(newT);
        return true;
    } else if (t2 != nullptr && t2->getId().compare(t->getId()) == 0)
    {
        setT2(newT);
        return true;
    }
    return false;
}


bool Edge::addFlag(FlagType f)
{
    if(std::find(associated_flags.begin(), associated_flags.end(), f) == associated_flags.end())
    {
        associated_flags.push_back(f);
        return true;
    }
    return false;
}

int Edge::searchFlag(FlagType f)
{
    std::vector<FlagType>::iterator it = std::find(associated_flags.begin(), associated_flags.end(), f);
    if(it != associated_flags.end())
        return  it - associated_flags.begin();
    return -1;
}

bool Edge::removeFlag(FlagType f)
{
    int flagPosition = searchFlag(f);
    if(flagPosition >= 0)
    {
        associated_flags.erase(associated_flags.begin() + flagPosition);
        return true;
    }
    return false;

}

bool Edge::removeFlag(unsigned int position)
{
    if(position < associated_flags.size())
    {
        associated_flags.erase(associated_flags.begin() + position);
        return true;
    }
    return false;

}

bool Edge::addInformation(void *info)
{
    if(std::find(information.begin(), information.end(), info) == information.end())
    {
        information.push_back(info);
        return true;
    }
    return false;
}


int Edge::searchInfo(void * info)
{
    std::vector<void*>::iterator it = std::find(information.begin(), information.end(), info);
    if(it != information.end())
        return  it - information.begin();
    return -1;
}

bool Edge::removeInfo(void *info)
{
    int flagPosition = searchInfo(info);
    if(flagPosition >= 0)
    {
        information.erase(information.begin() + flagPosition);
        return true;
    }
    return false;
}

bool Edge::removeInfo(unsigned int position)
{
    if(position < information.size())
    {
        information.erase(information.begin() + position);
        return true;
    }
    return false;
}

bool Edge::clearInfo()
{
    information.clear();
    return true;
}

std::string Edge::getId() const
{
    return id;
}

void Edge::setId(std::string newId)
{
    id = newId;
}

double Edge::computeLength() const
{
    return ((*v1) - (*v2)).norm();
}

void Edge::print(std::ostream &stream)
{
    stream << "Edge with id: " << id << " connecting vertices with ids: " << v1->getId() << " and " << v2->getId()<< ". Incident triangles have ids: "  << std::endl;
    if(t1 != nullptr)
        stream << t1->getId();
    else
        stream << "not available";
    stream << " and ";
    if(t2 != nullptr)
        stream << t2->getId();
    else
        stream << "not available";
    stream << std::endl;
}
