#ifndef ANNOTATION_H
#define ANNOTATION_H

#include <vector>
#include <string>
#include <ostream>
#include <algorithm>
#include <rapidjson/prettywriter.h>
#include <attribute.h>
#include <Vertex.h>
#include <TriangleMesh.h>

class TriangleMesh;

enum class AnnotationType{
    Volume,
    Surface,
    Line,
    Point
};


class Annotation {

    public:

        /**
         * @brief Annotation Contructor of the annotation class. The only property that is initialized is the color, to which we only associate a memory portion (3 bytes, corresponding to 3 slots containing numbers from 0 to 255).
         */
        Annotation(){ color = static_cast<unsigned char*>(std::malloc(3 * sizeof(unsigned char)));}

        Annotation(std::shared_ptr<Annotation> other) {
            mesh = other->getMesh();
            type = other->getType();
            id = other->getId();
            tag = other->getTag();                                        //The tag of the annotation
            hierarchyLevel = other->getHierarchyLevel();
            auto otherAttributes = other->getAttributes();
            attributes.insert(attributes.end(), otherAttributes.begin(), otherAttributes.end());
            color = other->getColor();
        };

        virtual ~Annotation(){ delete color;}

        /**
         * @brief print Prints main information related to the annotation (id, tag and attributes)
         * @param os The stream onto which the information should be written
         */
        virtual void print(std::ostream& os);

        /**
         * @brief print Prints main information related to the annotation (id, tag and attributes) in JSON format.
         * @param writer The interface with the RapidJSON library for writing the information in JSON format
         */
        virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer);

        /**
         * @brief getInvolvedVertices abstract method for obtaining the vertices involved in the annotation (vertices are the only always-present geometric entity of an annotation)
         * @return a vector containing pointers to the involved vertices.
         */
        virtual std::vector<std::shared_ptr<Vertex> > getInvolvedVertices() = 0;

        /**
         * @brief isPointInAnnotation checks if a given vertex is involved in this annotation
         * @param p the vertex that needs to be checked
         * @return true if the vertex is involved, false otherwise.
         */
        virtual bool isPointInAnnotation(std::shared_ptr<Vertex> p) = 0;

        //Getter and setter methods
        std::shared_ptr<TriangleMesh> getMesh() const{ return mesh; }
        void setMesh(std::shared_ptr<TriangleMesh> value){ this->mesh = value; }
        AnnotationType getType() const { return type; }
        std::string getTag() const { return tag; }
        void setTag(const std::string &value) { tag = value; }
        unsigned char* getColor(){ return this->color; }
        void setColor(unsigned char value[3]) { this->color[0] = value[0]; this->color[1] = value[1]; this->color[2] = value[2]; }
        std::vector<std::shared_ptr<Attribute> > getAttributes() const { return attributes; }
        void setAttributes(const std::vector<std::shared_ptr<Attribute> > &value) { attributes = value; }
        void addAttribute(std::shared_ptr<Attribute> value){ attributes.push_back(value); }
        void removeAttribute(std::shared_ptr<Attribute> value){
            std::vector<std::shared_ptr<Attribute> >::iterator it = std::find(attributes.begin(), attributes.end(), value);
            if(it != attributes.end())
                attributes.erase(it);
        }
        unsigned int getHierarchyLevel() const { return hierarchyLevel; }
        void setHierarchyLevel(unsigned int value) { hierarchyLevel = value; }
        unsigned int getId() const { return id; }
        void setId(unsigned int value){ id = value; }
        virtual Point* getCenter() = 0;
        virtual Point* getOrientation() = 0;

protected:
        std::shared_ptr<TriangleMesh> mesh;
        AnnotationType type;
        unsigned int id;
        std::string tag;                                        //The tag of the annotation
        unsigned int hierarchyLevel;
        std::vector<std::shared_ptr<Attribute> > attributes;
        unsigned char* color;                                   //The color associated to the annotation
        const unsigned short NUM_OF_THREADS = 8;                //Number of threads used for the transfer procedure
        double sphereRay;                                       //The radius of the sphere for the neighborhood search

};

#endif // ANNOTATION_H

