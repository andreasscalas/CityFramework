#ifndef OSMWAY_H
#define OSMWAY_H

#include <osmnode.h>
#include <memory>

class OSMWay
{
public:
    OSMWay();
    std::string getId() const;
    void setId(const std::string &value);

    std::vector<std::shared_ptr<OSMNode> > getNodes() const;
    void setNodes(const std::vector<std::shared_ptr<OSMNode> > &value);
    bool removeNode(std::string id);

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
    std::vector<std::shared_ptr<OSMNode> > nodes;
    std::string user_name;
    std::string user_id;
    bool is_visible;
    std::string version;
    std::string changeset;
    std::string timestamp;
    std::vector<std::pair<std::string, std::string> > tags; //Array of pairs <key, value>

};

#endif // OSMWAY_H
