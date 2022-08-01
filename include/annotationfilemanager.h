#ifndef ANNOTATIONFILEMANAGER_H
#define ANNOTATIONFILEMANAGER_H

#include <string>
#include <iostream>
#include <TriangleMesh.h>

class AnnotationFileManager
{
public:
    static const int BUFFER_SIZE = 65536;

    AnnotationFileManager();
    bool writeAnnotations(std::string fileName);
    bool readAnnotations(std::string fileName);
    std::vector<std::shared_ptr<Annotation> > readAndStoreAnnotations(std::string fileName);

    std::shared_ptr<TriangleMesh> getMesh() const;
    void setMesh(std::shared_ptr<TriangleMesh>value);

private:
    std::shared_ptr<TriangleMesh> mesh;
};

#endif // ANNOTATIONFILEMANAGER_H
