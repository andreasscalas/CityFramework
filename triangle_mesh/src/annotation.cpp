#include <annotation.h>

void Annotation::print(std::ostream &os)
{
   os << "id: " << id << std::endl;
   os << "tag: " << tag << std::endl;
   //os << "level: " << hierarchyLevel << std::endl;
   os << "attributes:" << std::endl;
   if(attributes.size() != 0){
       os << "[" << std::endl;
       for(unsigned int i = 0; i < attributes.size(); i++)
           attributes[i]->print(os);
       os << "]";
   }
   os << std::endl;
}

void Annotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    writer.Key("id");
    writer.Uint(id);
    writer.Key("tag");
    writer.String(tag.c_str());

    writer.Key("color");
    writer.StartArray();
    writer.Int(color[0]);
    writer.Int(color[1]);
    writer.Int(color[2]);
    writer.EndArray();

    writer.Key("attributes");
    writer.StartArray();
    for(unsigned int i = 0; i < attributes.size(); i++)
    {
        writer.StartObject();
        attributes[i]->printJson(writer);
        writer.EndObject();
    }
    writer.EndArray();
}
