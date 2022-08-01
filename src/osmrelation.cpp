#include "osmrelation.h"
#include <algorithm>

OSMRelation::OSMRelation()
{

}

std::string OSMRelation::getId() const
{
    return id;
}

void OSMRelation::setId(const std::string &value)
{
    id = value;
}

std::string OSMRelation::getUserName() const
{
    return user_name;
}

void OSMRelation::setUserName(const std::string &value)
{
    user_name = value;
}

std::string OSMRelation::getUserId() const
{
    return user_id;
}

void OSMRelation::setUserId(const std::string &value)
{
    user_id = value;
}

bool OSMRelation::getIsVisible() const
{
    return is_visible;
}

void OSMRelation::setIsVisible(bool value)
{
    is_visible = value;
}

std::string OSMRelation::getVersion() const
{
    return version;
}

void OSMRelation::setVersion(const std::string &value)
{
    version = value;
}

std::string OSMRelation::getChangeset() const
{
    return changeset;
}

void OSMRelation::setChangeset(const std::string &value)
{
    changeset = value;
}

std::vector<std::pair<std::string, std::string> > OSMRelation::getTags() const
{
    return tags;
}

void OSMRelation::setTags(const std::vector<std::pair<std::string, std::string> > &value)
{
    tags = value;
}

bool OSMRelation::checkTag(std::pair<std::string, std::string> tag)
{
    for(unsigned int i = 0; i < tags.size(); i++)
        if(tag.first == tags.at(i).first)
            return (tag.second.compare("") == 0 || tag.second.compare(tags.at(i).second) == 0);
    return false;
}

std::string OSMRelation::getTimestamp() const
{
    return timestamp;
}

void OSMRelation::setTimestamp(const std::string &value)
{
    timestamp = value;
}

std::vector<std::pair<std::shared_ptr<OSMRelation>, std::string> > OSMRelation::getRelations() const
{
    return relations;
}

void OSMRelation::setRelations(const std::vector<std::pair<std::shared_ptr<OSMRelation>, std::string> > &value)
{
    relations = value;
}

bool OSMRelation::addRelation(const std::pair<std::shared_ptr<OSMRelation>, std::string> relation)
{
    relations.push_back(relation);
    return true;
}

bool OSMRelation::removeRelation(std::shared_ptr<OSMRelation> relation)
{
    return removeRelation(relation->getId());
}

bool OSMRelation::removeRelation(std::string osmid)
{
    auto it = std::find_if(relations.begin(), relations.end(),
                           [osmid](std::pair<std::shared_ptr<OSMRelation>, std::string> r) { return r.first->getId().compare(osmid) == 0;});
    if(it != relations.end())
    {
        relations.erase(it);
        return true;
    }
    return false;
}

bool OSMRelation::removeRelation(const unsigned int position)
{
    if(position >= relations.size())
        return false;
    relations.erase(relations.begin() + position);
    return true;
}

std::vector<std::pair<std::shared_ptr<OSMNode>, std::string> > OSMRelation::getNodes() const
{
    return nodes;
}

void OSMRelation::setNodes(const std::vector<std::pair<std::shared_ptr<OSMNode>, std::string> > &value)
{
    nodes = value;
}

bool OSMRelation::addNode(const std::pair<std::shared_ptr<OSMNode>, std::string> node)
{
    nodes.push_back(node);
    return true;
}

bool OSMRelation::removeNode(std::shared_ptr<OSMNode> node)
{
    return removeNode(node->getId());
}

bool OSMRelation::removeNode(std::string osmid)
{

    auto it = std::find_if(nodes.begin(), nodes.end(),
                           [osmid](std::pair<std::shared_ptr<OSMNode>, std::string> n) { return n.first->getId().compare(osmid) == 0;});
    if(it != nodes.end())
    {
        nodes.erase(it);
        return true;
    }
    return false;
}

bool OSMRelation::removeNode(const unsigned int position)
{
    if(position >= nodes.size())
        return false;
    nodes.erase(nodes.begin() + position);
    return true;
}

std::vector<std::pair<std::shared_ptr<OSMWay>, std::string> > OSMRelation::getWays() const
{
    return ways;
}

void OSMRelation::setWays(const std::vector<std::pair<std::shared_ptr<OSMWay>, std::string> > &value)
{
    ways = value;
}

bool OSMRelation::addWay(const std::pair<std::shared_ptr<OSMWay>, std::string> way)
{
    ways.push_back(way);
    return true;
}

bool OSMRelation::removeWay(std::shared_ptr<OSMWay> relation)
{
    return removeWay(relation->getId());
}

bool OSMRelation::removeWay(std::string osmid)
{
    auto it = std::find_if(ways.begin(), ways.end(),
                           [osmid](std::pair<std::shared_ptr<OSMWay>, std::string> w) { return w.first->getId().compare(osmid) == 0;});
    if(it != ways.end())
    {
        ways.erase(it);
        return true;
    }
    return false;
}

bool OSMRelation::removeWay(const unsigned int position)
{
    if(position >= ways.size())
        return false;

    ways.erase(ways.begin() + position);
    return true;

}
