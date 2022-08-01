#include "osmway.h"

OSMWay::OSMWay()
{

}

std::string OSMWay::getId() const
{
    return id;
}

void OSMWay::setId(const std::string &value)
{
    id = value;
}

std::string OSMWay::getUserName() const
{
    return user_name;
}

void OSMWay::setUserName(const std::string &value)
{
    user_name = value;
}

std::string OSMWay::getUserId() const
{
    return user_id;
}

void OSMWay::setUserId(const std::string &value)
{
    user_id = value;
}

bool OSMWay::getIsVisible() const
{
    return is_visible;
}

void OSMWay::setIsVisible(bool value)
{
    is_visible = value;
}

std::string OSMWay::getVersion() const
{
    return version;
}

void OSMWay::setVersion(const std::string &value)
{
    version = value;
}

std::string OSMWay::getChangeset() const
{
    return changeset;
}

void OSMWay::setChangeset(const std::string &value)
{
    changeset = value;
}

std::vector<std::pair<std::string, std::string> > OSMWay::getTags() const
{
    return tags;
}

void OSMWay::setTags(const std::vector<std::pair<std::string, std::string> > &value)
{
    tags = value;
}

bool OSMWay::checkTag(std::pair<std::string, std::string> tag)
{
    for(unsigned int i = 0; i < tags.size(); i++)
        if(tag.first == tags.at(i).first)
            return (tag.second.compare("") == 0 || tag.second.compare(tags.at(i).second) == 0);
    return false;
}

std::string OSMWay::getTimestamp() const
{
    return timestamp;
}

void OSMWay::setTimestamp(const std::string &value)
{
    timestamp = value;
}

std::vector<std::shared_ptr<OSMNode> > OSMWay::getNodes() const
{
    return nodes;
}

void OSMWay::setNodes(const std::vector<std::shared_ptr<OSMNode> > &value)
{
    nodes = value;
}

bool OSMWay::removeNode(std::string id)
{
    for(unsigned int i = 0; i < nodes.size(); i++)
        if(nodes.at(i)->getId().compare(id) == 0)
        {
            nodes.erase(nodes.begin() + i);
            return true;
        }
    return false;
}
