#ifndef OSMRELATION_H
#define OSMRELATION_H

#include <osmnode.h>
#include <osmway.h>

class OSMRelation
{
public:
    OSMRelation();

    std::string getId() const;
    void setId(const std::string &value);

    std::vector<std::pair<std::shared_ptr<OSMNode>, std::string> > getNodes() const;
    void setNodes(const std::vector<std::pair<std::shared_ptr<OSMNode>, std::string> > &value);
    bool addNode(const std::pair<std::shared_ptr<OSMNode>, std::string> node);
    bool removeNode(std::shared_ptr<OSMNode>);
    bool removeNode(std::string osmid);
    bool removeNode(const unsigned int position);

    std::vector<std::pair<std::shared_ptr<OSMWay>, std::string> > getWays() const;
    void setWays(const std::vector<std::pair<std::shared_ptr<OSMWay>, std::string> > &value);
    bool addWay(const std::pair<std::shared_ptr<OSMWay>, std::string> way);
    bool removeWay(std::shared_ptr<OSMWay>);
    bool removeWay(std::string osmid);
    bool removeWay(const unsigned int position);

    std::vector<std::pair<std::shared_ptr<OSMRelation>, std::string> > getRelations() const;
    void setRelations(const std::vector<std::pair<std::shared_ptr<OSMRelation>, std::string> > &value);
    bool addRelation(const std::pair<std::shared_ptr<OSMRelation>, std::string> relation);
    bool removeRelation(std::shared_ptr<OSMRelation>);
    bool removeRelation(std::string osmid);
    bool removeRelation(const unsigned int position);

    std::string getUserName() const;
    void setUserName(const std::string &value);

    std::string getUserId() const;
    void setUserId(const std::string &value);

    bool getIsVisible() const;
    void setIsVisible(bool value);

    std::string getVersion() const;
    void setVersion(const std::string &value);

    std::string getChangeset() const;
    void setChangeset(const std::string &value);

    std::vector<std::pair<std::string, std::string> > getTags() const;
    void setTags(const std::vector<std::pair<std::string, std::string> > &value);
    bool checkTag(std::pair<std::string, std::string> tag);

    std::string getTimestamp() const;
    void setTimestamp(const std::string &value);


private:
    std::string id;
    std::vector<std::pair<std::shared_ptr<OSMNode>, std::string> > nodes;
    std::vector<std::pair<std::shared_ptr<OSMWay>, std::string> > ways;
    std::vector<std::pair<std::shared_ptr<OSMRelation>, std::string> > relations;
    std::string user_name;
    std::string user_id;
    bool is_visible;
    std::string version;
    std::string changeset;
    std::string timestamp;
    std::vector<std::pair<std::string, std::string> > tags; //Array of pairs <key, value>
};

#endif // OSMRELATION_H
