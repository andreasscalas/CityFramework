#ifndef OSMNODE_H
#define OSMNODE_H

#include <string>
#include <vector>
#include <memory>
#include <Point.h>

class OSMNode
{
public:
    OSMNode();

    std::string getId() const;
    void setId(const std::string &value);

    std::string getUserName() const;
    void setUserName(const std::string &value);

    std::string getUserId() const;
    void setUserId(const std::string &value);

    std::string getVersion() const;
    void setVersion(const std::string &value);

    std::string getChangeset() const;
    void setChangeset(const std::string &value);

    bool getIsVisible() const;
    void setIsVisible(bool value);

    std::vector<std::pair<std::string, std::string> > getTags() const;
    void setTags(const std::vector<std::pair<std::string, std::string> > &value);

    std::shared_ptr<Point> getCoordinates() const;
    void setCoordinates(const std::shared_ptr<Point> &value);

    std::string getTimestamp() const;
    void setTimestamp(const std::string &value);

    void print(std::ostream &stream);
private:
    std::string id;
    std::string user_name;
    std::string user_id;
    std::string version;
    std::string changeset;
    std::string timestamp;
    std::shared_ptr<Point> coordinates;
    bool is_visible;
    std::vector<std::pair<std::string, std::string> > tags; //Array of pairs <key, value>

};

#endif // OSMNODE_H
