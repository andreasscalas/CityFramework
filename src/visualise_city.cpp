#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <Point.h>
#include <utilities.h>

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPropAssembly.h>
#include <vtkLine.h>
#include <vtkTriangle.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleJoystickCamera.h>
#include <vtkInteractorStyle3D.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAutoInit.h>
#include <annotation.h>
#include <annotationfilemanager.h>
#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace std;

/*************TEMPORANEO, LE VARIABILI GLOBALI SONO IL MALE***************/
vtkSmartPointer<vtkPropAssembly> assembly;
vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
vtkSmartPointer<vtkPropAssembly> canvas;
unsigned int layerId = 0;
unsigned int shadowId = 0;
bool showShadows = false;

static const unsigned char RED[3] =    {255,0,0};
static const unsigned char GREEN[3] =  {0,255,0};
static const unsigned char GRAY[3] =   {220,220,220};
static const unsigned char BLUE[3] =   {0,0,255};
static const unsigned char BLACK[3] =  {0,0,0};
static const unsigned char WHITE[3] =  {255,255,255};
static const unsigned char BROWN[3] =  {98,76,54};
static const unsigned char BEIGE[3] =  {245,245,220};

void updateView();

namespace {

    // Define interaction style
    class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
        public:
          static KeyPressInteractorStyle* New();
          vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

          virtual void OnKeyPress() override
          {
                // Get the keypress
                vtkRenderWindowInteractor* rwi = this->Interactor;
                std::string key = rwi->GetKeySym();
                // Output the key that was pressed
                std::cout << "Pressed " << key << std::endl;

                if(key == "o"){
                    showShadows = !showShadows;
                    buildVTKStructure();
                    updateView();
                } else {
                    unsigned int currentPos;
                    if(showShadows)
                        currentPos = shadowId;
                    else
                        currentPos = layerId;


                    // Handle an arrow key
                    if (key == "Up" || key == "Right")
                    {
                        if(showShadows && shadowId < shadows.size() - 1)
                            shadowId++;
                        else if(layerId < annotations.size() - 1)
                            layerId++;
                    } else if (key == "Down" || key == "Left")
                    {
                        if(showShadows && shadowId > 0)
                            shadowId--;
                        else if(layerId > 0)
                            layerId--;
                    } else if(key.size() == 1 && key.at(0) >= '0' && key.at(0) <= '9'){
                        int keyValue = atoi(key.c_str());
                        if(showShadows && keyValue < shadows.size())
                            shadowId = static_cast<unsigned int>(keyValue);
                        else if(keyValue < annotations.size())
                            layerId = static_cast<unsigned int>(keyValue);
                    }

                    if((showShadows && shadowId != currentPos) || (!showShadows && layerId != currentPos))
                    {
                        buildVTKStructure();
                        updateView();
                    }
                }

                // Forward events
                vtkInteractorStyleTrackballCamera::OnKeyPress();
          }

          void buildVTKStructure()
          {
              if(showShadows)
              {
                    std::cout << "shadows" << std::endl;
                    assembly = vtkSmartPointer<vtkPropAssembly>::New();
                    vtkSmartPointer<vtkUnsignedCharArray> meshVColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                    meshVColors->SetNumberOfComponents(3);
                    meshVColors->SetName("PColors");
                    for(unsigned int i = 0; i < shadows.at(shadowId).size(); i++)
                    {
                        bool lightened = shadows.at(shadowId).at(i);
                        if(!lightened)
                            meshVColors->InsertNextTypedTuple(BLACK);
                        else
                            meshVColors->InsertNextTypedTuple(WHITE);
                    }

                  vtkSmartPointer<vtkActor> meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
                  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                  vtkMesh->GetPointData()->SetScalars(meshVColors);
                  mapper->SetInputData(vtkMesh);
                  meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
                  meshSurfaceActor->SetMapper(mapper);
                  meshSurfaceActor->GetProperty()->SetRepresentationToSurface();
                  assembly->AddPart(meshSurfaceActor);
              } else
              {
                  assembly = vtkSmartPointer<vtkPropAssembly>::New();
                  vtkSmartPointer<vtkPoints> annotatedPoints = vtkSmartPointer<vtkPoints>::New();
                  vtkSmartPointer<vtkCellArray> annotatedVertices = vtkSmartPointer<vtkCellArray>::New();
                  vtkSmartPointer<vtkUnsignedCharArray> meshEColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                  meshEColors->SetNumberOfComponents(3);
                  meshEColors->SetName("LColors");
                  vtkSmartPointer<vtkUnsignedCharArray> meshTColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                  meshTColors->SetNumberOfComponents(3);
                  meshTColors->SetName ("TColors");
                  vtkSmartPointer<vtkActor> meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
                  vtkSmartPointer<vtkActor> meshPointsActor = vtkSmartPointer<vtkActor>::New();
                  vtkSmartPointer<vtkActor> meshWireframeActor = vtkSmartPointer<vtkActor>::New();

                  vtkSmartPointer<vtkCellArray> annotatedLines = vtkSmartPointer<vtkCellArray>::New();
                  vtkSmartPointer<vtkUnsignedCharArray> annotatedLinesColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                  annotatedLinesColors->SetNumberOfComponents(3);
                  annotatedLinesColors->SetName("AnnotationsEColors");

                  vtkSmartPointer<vtkUnsignedCharArray> annotatedPointsColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
                  annotatedPointsColors->SetNumberOfComponents(3);
                  annotatedPointsColors->SetName("AnnotationsVColors");

                  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
                  std::vector<unsigned int*> triangles_colors;
                  std::vector<unsigned int> triangles_colors_counter;
                  for(unsigned int i = 0; i < mesh->getTrianglesNumber(); i++)
                  {
                      triangles_colors.push_back(new unsigned int(3));
                      triangles_colors.at(i)[0] = BLACK[0];
                      triangles_colors.at(i)[1] = BLACK[1];
                      triangles_colors.at(i)[2] = BLACK[2];
                      triangles_colors_counter.push_back(0);
                  }

                  for(unsigned int j = 0; j < annotations.at(layerId).size(); j++)
                  {
                      if(dynamic_pointer_cast<SurfaceAnnotation>(annotations.at(layerId).at(j)) != nullptr)
                      {
                          std::shared_ptr<SurfaceAnnotation> a = dynamic_pointer_cast<SurfaceAnnotation>(annotations.at(layerId).at(j));
                          vector<std::shared_ptr<Triangle> > annotationTriangles = a->getTriangles();

                          //Update of the data-visualization linking
                          for(vector<std::shared_ptr<Triangle> >::iterator tit = annotationTriangles.begin(); tit != annotationTriangles.end(); tit++){
                              unsigned int tid = stoi((*tit)->getId());
                              triangles_colors.at(tid)[0] += a->getColor()[0];
                              triangles_colors.at(tid)[1] += a->getColor()[1];
                              triangles_colors.at(tid)[2] += a->getColor()[2];
                              triangles_colors_counter.at(tid)++;
                          }
                          if(annotationTriangles.size() > 20000)
                              std::cout << "Annotation " << j << std::endl;
                      } else if(dynamic_pointer_cast<LineAnnotation>(annotations.at(layerId).at(j)) != nullptr)
                      {

                          std::shared_ptr<LineAnnotation> a = dynamic_pointer_cast<LineAnnotation>(annotations.at(layerId).at(j));

                          //Update of the data-visualization linking
                          for(unsigned int k = 0; k < a->getPolyLines().size(); k++)
                              for (unsigned int l = 1; l < a->getPolyLines().at(k).size(); l++) {
                                  vtkSmartPointer<vtkLine> segment = vtkSmartPointer<vtkLine>::New();
                                  int vid1 = stoi(a->getPolyLines().at(k).at(l - 1)->getId());
                                  int vid2 = stoi(a->getPolyLines().at(k).at(l)->getId());
                                  segment->GetPointIds()->SetId(0, static_cast<vtkIdType>(vid1));
                                  segment->GetPointIds()->SetId(1, static_cast<vtkIdType>(vid2));
                                  annotatedLines->InsertNextCell(segment);
                                  annotatedLinesColors->InsertNextTypedTuple(a->getColor());
                              }
                      } else if(dynamic_pointer_cast<PointAnnotation>(annotations.at(layerId).at(j)) != nullptr)
                      {
                          continue;
                          std::shared_ptr<PointAnnotation> a = dynamic_pointer_cast<PointAnnotation>(annotations.at(layerId).at(j));
                          for(unsigned int k = 0; k < a->getPoints().size(); k++)
                          {
                              vtkIdType id[1];
                              id[0] = annotatedPoints->InsertNextPoint(a->getPoints().at(k)->getX(), a->getPoints().at(k)->getY(), a->getPoints().at(k)->getZ());
                              annotatedVertices->InsertNextCell(1,id);
                              annotatedPointsColors->InsertNextTypedTuple(RED);
                          }
                      }
                  }

                  for(unsigned int j = 0; j < mesh->getTrianglesNumber(); j++)
                  {
                      if( triangles_colors_counter.at(j) > 0)
                      {
                          triangles_colors.at(j)[0] /= triangles_colors_counter.at(j);
                          triangles_colors.at(j)[1] /= triangles_colors_counter.at(j);
                          triangles_colors.at(j)[2] /= triangles_colors_counter.at(j);
                      } else
                      {
                          triangles_colors.at(j)[0] = BEIGE[0];
                          triangles_colors.at(j)[1] = BEIGE[1];
                          triangles_colors.at(j)[2] = BEIGE[2];
                      }
                  }


                  for(unsigned int j = 0; j < mesh->getEdgesNumber(); j++)
                      meshEColors->InsertNextTypedTuple(BLACK);



                  for(unsigned int j = 0; j < mesh->getTrianglesNumber(); j++){
                      unsigned char* color = new unsigned char(3);
                      color[0] = triangles_colors.at(j)[0];
                      color[1] = triangles_colors.at(j)[1];
                      color[2] = triangles_colors.at(j)[2];
                      delete triangles_colors.at(j);
                      meshTColors->InsertNextTypedTuple(color);
                  }

                  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                  vtkMesh->GetCellData()->SetScalars(meshTColors);
                  mapper->SetInputData(vtkMesh);
                  meshSurfaceActor = vtkSmartPointer<vtkActor>::New();
                  meshSurfaceActor->SetMapper(mapper);
                  meshSurfaceActor->GetProperty()->SetRepresentationToSurface();
                  assembly->AddPart(meshSurfaceActor);

                  /************ANNOTATIONS***********/

                  vtkSmartPointer<vtkPolyData> annotatedLinesPolydata = vtkSmartPointer<vtkPolyData>::New();
                  annotatedLinesPolydata->SetPoints(meshPoints);
                  annotatedLinesPolydata->SetLines(annotatedLines);
                  annotatedLinesPolydata->GetCellData()->SetScalars(annotatedLinesColors);
                  vtkSmartPointer<vtkPolyDataMapper> annotatedLinesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                  vtkSmartPointer<vtkActor> annotatedLinesActor = vtkSmartPointer<vtkActor>::New();
                  annotatedLinesMapper->SetInputData(annotatedLinesPolydata);
                  annotatedLinesActor->SetMapper(annotatedLinesMapper);
                  annotatedLinesActor->GetProperty()->SetOpacity(0.8);
                  annotatedLinesActor->GetProperty()->SetLineWidth(8);
                  assembly->AddPart(annotatedLinesActor);


                  vtkSmartPointer<vtkPolyData> annotatedPointsPolydata = vtkSmartPointer<vtkPolyData>::New();
                  annotatedPointsPolydata->SetPoints(annotatedPoints);
                  annotatedPointsPolydata->SetVerts(annotatedVertices);
                  annotatedPointsPolydata->GetPointData()->SetScalars(annotatedPointsColors);
                  vtkSmartPointer<vtkPolyDataMapper> annotatedPointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                  vtkSmartPointer<vtkActor> annotatedPointsActor = vtkSmartPointer<vtkActor>::New();
                  annotatedPointsMapper->SetInputData(annotatedPointsPolydata);
                  annotatedPointsActor->SetMapper(annotatedPointsMapper);
                  annotatedPointsActor->GetProperty()->SetOpacity(1.0);
                  annotatedPointsActor->GetProperty()->SetPointSize(10.0f);
                  annotatedPointsActor->GetProperty()->SetRepresentationToPoints();
                  assembly->AddPart(annotatedPointsActor);


          //        vtkSmartPointer<vtkPolyData> wireframePolydata = vtkSmartPointer<vtkPolyData>::New();
          //        vtkSmartPointer<vtkPolyDataMapper> wireframeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
          //        wireframePolydata->SetPoints(meshPoints);
          //        wireframePolydata->SetLines(meshEdges);
          //        wireframePolydata->GetCellData()->SetScalars(meshEColors);
          //        wireframeMapper->SetInputData(wireframePolydata);
          //        meshWireframeActor = vtkSmartPointer<vtkActor>::New();
          //        meshWireframeActor->SetMapper(wireframeMapper);
          //        meshWireframeActor->GetProperty()->SetRepresentationToWireframe();


          //        meshWireframeActor->GetProperty()->SetOpacity(1.0);
          //        meshWireframeActor->GetProperty()->SetLineWidth(3.0);

                  //assembly->AddPart(meshWireframeActor);


          //        vtkSmartPointer<vtkPolyData> pointPolydata = vtkSmartPointer<vtkPolyData>::New();
          //        vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
          //        meshPointsActor = vtkSmartPointer<vtkActor>::New();
          //        pointPolydata->SetPoints(meshPoints);
          //        pointPolydata->SetPolys(meshTriangles);
          //        pointPolydata->GetPointData()->SetScalars(meshVColors);
          //        pointMapper->SetInputData(pointPolydata);
          //        meshPointsActor->SetMapper(pointMapper);
          //        meshPointsActor->GetProperty()->SetPointSize(5.0f);
          //        meshPointsActor->GetProperty()->SetRepresentationToPoints();
          //        assembly->AddPart(meshPointsActor);

                  assembly->Modified();
              }
          }


          std::vector<std::vector<std::shared_ptr<Annotation> > > getAnnotations() const
          {
              return annotations;
          }

          void setAnnotations(const std::vector<std::vector<std::shared_ptr<Annotation> > > &value)
          {
              annotations = value;
          }

          std::vector<std::vector<bool> > getShadows() const
          {
              return shadows;
          }

          void setShadows(const std::vector<std::vector<bool> > &value)
          {
              shadows = value;
          }

          std::shared_ptr<TriangleMesh> getMesh() const
          {
              return mesh;
          }

          void setMesh(std::shared_ptr<TriangleMesh> value)
          {
              mesh = value;
              meshPoints = vtkSmartPointer<vtkPoints>::New();
              meshEdges = vtkSmartPointer<vtkCellArray>::New();
              meshTriangles = vtkSmartPointer<vtkCellArray>::New();
              vtkMesh = vtkSmartPointer<vtkPolyData>::New();
              for(unsigned int j = 0; j < mesh->getVerticesNumber(); j++){
                  std::shared_ptr<Vertex> v = mesh->getVertex(j);
                  meshPoints->InsertNextPoint(v->getX(), v->getY(), v->getZ());
                  //meshVColors->InsertNextTypedTuple(BLUE);
              }


              for(unsigned int j = 0; j < mesh->getEdgesNumber(); j++){
                  std::shared_ptr<Edge> e = mesh->getEdge(j);
                  vtkSmartPointer<vtkLine> edge = vtkSmartPointer<vtkLine>::New();
                  edge->GetPointIds()->SetNumberOfIds(2);
                  edge->GetPointIds()->SetId(0, static_cast<vtkIdType>(stoi(e->getV1()->getId())));
                  edge->GetPointIds()->SetId(1, static_cast<vtkIdType>(stoi(e->getV2()->getId())));
                  meshEdges->InsertNextCell(edge);
              }

              for(unsigned int j = 0; j < mesh->getTrianglesNumber(); j++){
                  std::shared_ptr<Triangle> t = mesh->getTriangle(j);
                  std::shared_ptr<Vertex> v1, v2, v3;
                  v1 = t->getV1();
                  v2 = t->getV2();
                  v3 = t->getV3();
                  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
                  triangle->GetPointIds()->SetNumberOfIds(3);
                  triangle->GetPointIds()->SetId(0, static_cast<vtkIdType>(stoi(v1->getId())));
                  triangle->GetPointIds()->SetId(1, static_cast<vtkIdType>(stoi(v2->getId())));
                  triangle->GetPointIds()->SetId(2, static_cast<vtkIdType>(stoi(v3->getId())));
                  meshTriangles->InsertNextCell(triangle);
              }

              vtkMesh->SetPoints(meshPoints);
              vtkMesh->SetPolys(meshTriangles);
          }

    protected:
          std::shared_ptr<TriangleMesh> mesh;
          vtkSmartPointer<vtkPoints> meshPoints;
          vtkSmartPointer<vtkCellArray> meshEdges;
          vtkSmartPointer<vtkCellArray> meshTriangles;
          vtkSmartPointer<vtkPolyData> vtkMesh;
          std::vector<std::vector<std::shared_ptr<Annotation> > > annotations;
          std::vector<std::vector<bool> > shadows;


    };

    vtkStandardNewMacro(KeyPressInteractorStyle);

}

vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();




int main(int argc, char *argv[])
{

    if(argc < 2)
    {
        std::cerr << "Missing mesh parameter" << std::endl;
        return 1;
    }
    if(argc < 3)
    {
        std::cerr << "Missing annotations' folder parameter" << std::endl;
        return 2;
    }
    if(argc < 4)
    {
        std::cerr << "Missing shadows' folder parameter" << std::endl;
        return 3;
    }
    std::shared_ptr<TriangleMesh> mesh = std::make_shared<TriangleMesh>();
    int retValue = mesh->load(argv[1]);
    if(retValue != 0)
    {
        std::cerr << "Error while reading mesh file" << std::endl;
        return 4;
    }
    Point min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
          max(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
    for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
    {
        std::shared_ptr<Vertex> v = mesh->getVertex(i);
        if(v->getX() < min.getX())
            min.setX(v->getX());
        if(v->getY() < min.getY())
            min.setY(v->getY());
        if(v->getZ() < min.getZ())
            min.setZ(v->getZ());

        if(v->getX() > max.getX())
            max.setX(v->getX());
        if(v->getY() > max.getY())
            max.setY(v->getY());
        if(v->getZ() > max.getZ())
            max.setZ(v->getZ());
    }

    for(unsigned int i = 0; i < mesh->getVerticesNumber(); i++)
    {
        *mesh->getVertex(i) -= min;
        *mesh->getVertex(i) /= (max - min).norm();
    }

    std::string annotations_directory = argv[2];
    std::string shadows_directory = argv[3];
    if(annotations_directory.at(annotations_directory.size() - 1) != '/')
        annotations_directory.append("/");
    if(shadows_directory.at(shadows_directory.size() - 1) != '/')
        shadows_directory.append("/");
    std::vector<std::string> annotations_files = Utilities::globVector(annotations_directory+"*.ant");
    std::vector<std::string> shadows_files = Utilities::globVector(shadows_directory+"*.shdw");
    std::vector<std::vector<std::shared_ptr<Annotation> > > all_annotations;
    std::vector<std::vector<bool> > shadows;
    std::vector<std::shared_ptr<Annotation> > emptyAnnotation;
    all_annotations.push_back(emptyAnnotation);

    AnnotationFileManager manager;
    manager.setMesh(mesh);
    for(unsigned int i = 0; i < annotations_files.size(); i++)
    {
        std::vector<std::shared_ptr<Annotation> > annotations = manager.readAndStoreAnnotations(annotations_files.at(i));
        if(annotations.size() == 0)
        {
            std::cerr << "Error loading annotations." << std::endl << std::flush;
            exit(3);
        }
        all_annotations.push_back(annotations);
    }
    for(unsigned int i = 0; i < shadows_files.size(); i++)
    {
        std::ifstream shadow_file_stream(shadows_files.at(i));
        if(shadow_file_stream.is_open())
        {
            std::string line;
            std::vector<bool> file_shadows;
            while(std::getline(shadow_file_stream, line))
                file_shadows.push_back(atoi(line.c_str()));
            shadow_file_stream.close();
            shadows.push_back(file_shadows);
        }
    }

    if(retValue == 0)
    {
        style->setMesh(mesh);
        style->setAnnotations(all_annotations);
        style->setShadows(shadows);

        style->buildVTKStructure();
        updateView();
        mesh.reset();
    } else
        std::cerr << "Program terminated with return value " <<  retValue << std::endl;


    return EXIT_SUCCESS;

}

void updateView()
{
    renderer->RemoveActor(canvas);
    canvas = assembly;
    renderWindow->RemoveRenderer(renderer);
    renderer->SetBackground(255,255,255);
    renderer->AddActor(canvas);
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(640, 480);
    renderWindow->SetWindowName("CityViewer");
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindowInteractor->SetInteractorStyle(style);
    renderWindow->Render();
    renderWindowInteractor->Start();

}
