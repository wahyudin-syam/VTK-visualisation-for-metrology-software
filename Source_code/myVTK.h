//========== C/C++ Library =========================================
#include <stdlib.h>
#include <time.h> // for generating the seed of random number generator
#include <string.h>

//=========VTK Library=================================================
#include "vtkConeSource.h"
#include "vtkPolyData.h"
#include "vtkPoints.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"
#include "vtkIntArray.h"
#include "vtkPointData.h"
#include "vtkCellArray.h"
#include "vtkCylinderSource.h"
#include "vtkSTLReader.h"
#include "vtkShrinkPolyData.h"
#include "vtkOutlineFilter.h"
#include "vtkPolyDataNormals.h"
#include "vtkPolyDataMapper.h"
#include "vtkWindow.h"
#include "vtkCamera.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkLODActor.h"
#include "vtkCubeAxesActor2D.h"
#include "vtkRenderer.h"
#include "vtkProperty.h"
#include "vtkCommand.h"
#include "vtkBoxWidget.h"
#include "vtkTransform.h"
#include "vtkInteractorStyleTrackballCamera.h"


//=====================================================================
class vtkMyCallback : public vtkCommand
{
public:
  static vtkMyCallback *New() 
    { return new vtkMyCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      vtkTransform *t = vtkTransform::New();
      vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);
      widget->GetTransform(t);
      widget->GetProp3D()->SetUserTransform(t);
      t->Delete();
    }
};
//=====================================================================

class myVTK{
public:
	myVTK();
	~myVTK();
	void cone();
	void boxWidget();
	void cylinder();
	void stlModel();
	void stlModelShrink();
	void actorAxis();
	void ownData();
	void pointCloud();
};

myVTK::myVTK(){
}

myVTK::~myVTK(){
}

void myVTK::pointCloud(){

	//Visualization Pipeline
	vtkPoints *points=vtkPoints::New();
	vtkCellArray *strips=vtkCellArray::New();

	int i,j,k,n=0,counterLine=0, counter=0, tokenCounter=0, searchToken=0;
	float point[3],a,b,c;
	char strLine[100];
	char* token;
	char* tokenTemp;
	//char* strLine;

	//Setting the color base on the Z value
	vtkDoubleArray* pointColor = vtkDoubleArray::New();
	pointColor->SetName("pointColor");
	
	FILE* pFile;
	//pFile=fopen("Data2.txt","r");
	//n=499089;
	pFile=fopen("Data04b.txt","r");
	n=1799350;
	strips->InsertNextCell(n+1);
	while(!feof(pFile)){
		//fscanf(pFile,"%f %f %f %f %f %f",&point[0],&point[1],&point[2],&a, &b, &c);  //with normal vector: XYZ and abc sirection
		fscanf(pFile,"%f %f %f",&point[0],&point[1],&point[2]);    //without normal vector: only XYZ coordinate
		points->InsertPoint(counter,point);
		strips->InsertCellPoint(counter);
		counter++;		

		//set the color
		pointColor->InsertNextValue(point[2]*1000.0);		
	}
	fclose(pFile);

	/*while(!feof(pFile)){//(fgetc!=EOF){
		counterLine++;
		fgets(strLine,100,pFile);
		token=strtok(strLine," ");		
		tokenCounter=0; //calculating number of word inside the whole string
		tokenTemp=strtok(strLine," ");
		while(!tokenTemp){
			tokenCounter++;
			tokenTemp=strtok(NULL," ");
		}
		if(tokenCounter<6){
			if(counterLine==1){
				searchToken=0;
				while(!token){//move to the number of points
					if(searchToken==2){
						n=atoi(token);
					}
					searchToken++;
					token=strtok(NULL," ");
				}				
				strips->InsertNextCell(n);
			}
			counterLine++;
		}
		else{			
			searchToken=0;
			while(!token){//move to the number of points
				if(searchToken<3){
					point[searchToken]=atof(token);
				}
				searchToken++;
				token=strtok(NULL," ");
			}	
			points->InsertPoint(counter,point);
			strips->InsertCellPoint(counter);
			counter++;	
			counterLine++;
		}
	}
	fclose(pFile);
	*/

	/*vtkDoubleArray* pointColor = vtkDoubleArray::New();
	pointColor->SetName("pointColor");
	pointColor->InsertNextValue(2.7);
	pointColor->InsertNextValue(4.1);
	pointColor->InsertNextValue(5.3);
	pointColor->InsertNextValue(3.4);*/

	vtkPolyData *ownData = vtkPolyData::New();
	ownData->SetPoints(points);
	ownData->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
	//ownData->SetStrips(strips); //show as triangulation surface
	ownData->SetVerts(strips); //show as points
	
	vtkPolyDataMapper *ownDataMapper= vtkPolyDataMapper::New();
	ownDataMapper->SetInput(ownData);

	//Graphics Pipeline
	vtkActor *ownDataActor = vtkActor::New();
	ownDataActor->SetMapper(ownDataMapper);
	ownDataActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//cylinderActor->RotateX(30.0);
	//cylinderActor->RotateY(-45.0);

	vtkRenderer *ren1=vtkRenderer::New();
	ren1->AddActor(ownDataActor);
	ren1->SetBackground(1.0,1.0,0.9);

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(500,500);

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	//memory free
	
	ownDataMapper->Delete();
	ownDataActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();
	ownData->Delete();
	points->Delete();
	strips->Delete();
	pointColor->Delete();

}

void myVTK::ownData(){
	
	//Visualization Pipeline
	int i,j,k;
	float point[10][3];
	srand(time(NULL)); 
	for(i=0;i<10;i++){//set the coordinate
		point[i][0]=(float)(rand()%10)/10;//i%10;
		point[i][1]=(float)(rand()%10)/10;//i%10;
		point[i][2]=(float)(rand()%10)/10;
	}

	vtkPoints *points=vtkPoints::New();
	vtkCellArray *strips=vtkCellArray::New();
	strips->InsertNextCell(10);
	for(i=0;i<10;i++){
		points->InsertPoint(i,point[i]);
		strips->InsertCellPoint(i);
	}

	vtkDoubleArray* pointColor = vtkDoubleArray::New();
	pointColor->SetName("pointColor");
	pointColor->InsertNextValue(2.7);
	pointColor->InsertNextValue(4.1);
	pointColor->InsertNextValue(5.3);
	pointColor->InsertNextValue(3.4);

	vtkPolyData *ownData = vtkPolyData::New();
	ownData->SetPoints(points);
	ownData->GetPointData()->SetScalars(pointColor);
	//ownData->SetStrips(strips); //show as triangulation surface
	ownData->SetVerts(strips); //show as points
	
	vtkPolyDataMapper *ownDataMapper= vtkPolyDataMapper::New();
	ownDataMapper->SetInput(ownData);

	//Graphics Pipeline
	vtkActor *ownDataActor = vtkActor::New();
	ownDataActor->SetMapper(ownDataMapper);
	ownDataActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//cylinderActor->RotateX(30.0);
	//cylinderActor->RotateY(-45.0);

	vtkRenderer *ren1=vtkRenderer::New();
	ren1->AddActor(ownDataActor);
	ren1->SetBackground(1.0,1.0,0.9);

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(500,500);

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	//memory free
	
	ownDataMapper->Delete();
	ownDataActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();
	ownData->Delete();
	points->Delete();
	strips->Delete();
	pointColor->Delete();
	
	
 /* int i;
  // Create a float array which represents the points.
  vtkFloatArray* pcoords = vtkFloatArray::New();

  // Note that by default, an array has 1 component.
  // We have to change it to 3 for points
  pcoords->SetNumberOfComponents(3);
  // We ask pcoords to allocate room for at least 4 tuples
  // and set the number of tuples to 4.
  pcoords->SetNumberOfTuples(4);
  // Assign each tuple. There are 5 specialized versions of SetTuple:
  // SetTuple1 SetTuple2 SetTuple3 SetTuple4 SetTuple9
  // These take 1, 2, 3, 4 and 9 components respectively.
  float pts[4][3] = { {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
                      {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0} };
  for (i=0; i<4; i++)
    {
    pcoords->SetTuple(i, pts[i]);
    }

  // Create vtkPoints and assign pcoords as the internal data array.
  vtkPoints* points = vtkPoints::New();
  points->SetData(pcoords);

  // Create the cells. In this case, a triangle strip with 2 triangles
  // (which can be represented by 4 points)
  vtkCellArray* strips = vtkCellArray::New();
  strips->InsertNextCell(4);
  strips->InsertCellPoint(0);
  strips->InsertCellPoint(1);
  strips->InsertCellPoint(2);
  strips->InsertCellPoint(3);

  // Create an integer array with 4 tuples. Note that when using
  // InsertNextValue (or InsertNextTuple1 which is equivalent in
  // this situation), the array will expand automatically
  vtkIntArray* temperature = vtkIntArray::New();
  temperature->SetName("Temperature");
  temperature->InsertNextValue(10);
  temperature->InsertNextValue(20);
  temperature->InsertNextValue(30);
  temperature->InsertNextValue(40);

  // Create a double array.
  vtkDoubleArray* vorticity = vtkDoubleArray::New();
  vorticity->SetName("Vorticity");
  vorticity->InsertNextValue(2.7);
  vorticity->InsertNextValue(4.1);
  vorticity->InsertNextValue(5.3);
  vorticity->InsertNextValue(3.4);

  // Create the dataset. In this case, we create a vtkPolyData
  vtkPolyData* polydata = vtkPolyData::New();
  // Assign points and cells
  polydata->SetPoints(points);
  polydata->SetStrips(strips);
  // Assign scalars
  polydata->GetPointData()->SetScalars(temperature);
  // Add the vorticity array. In this example, this field
  // is not used.
  polydata->GetPointData()->AddArray(vorticity);

  // Create the mapper and set the appropriate scalar range
  // (default is (0,1)
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInput(polydata);
  mapper->SetScalarRange(0, 40);

  // Create an actor.
  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);

  // Create the rendering objects.
  vtkRenderer* ren = vtkRenderer::New();
  ren->AddActor(actor);

  vtkRenderWindow* renWin = vtkRenderWindow::New();
  renWin->AddRenderer(ren);

  vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);
  iren->Initialize();
  iren->Start();

  pcoords->Delete();
  points->Delete();
  strips->Delete();
  temperature->Delete();
  vorticity->Delete();
  polydata->Delete();
  mapper->Delete();
  actor->Delete();
  ren->Delete();
  renWin->Delete();
  iren->Delete();
*/
}

void myVTK::actorAxis(){
	//Visualization Pipeline/////////////////////////////////////////////
	vtkSTLReader *STLModel= vtkSTLReader::New(); //Reader Source Object
	STLModel->SetFileName("test.stl");

	vtkPolyDataNormals *normals=vtkPolyDataNormals::New();
	normals->SetInput(STLModel->GetOutput());
	
	vtkPolyDataMapper *STLModelMapper= vtkPolyDataMapper::New();
	STLModelMapper->SetInput(normals->GetOutput());

	vtkOutlineFilter *outline=vtkOutlineFilter::New();
	outline->SetInput(normals->GetOutput());

	vtkPolyDataMapper *outlineMapper=vtkPolyDataMapper::New();
	outlineMapper->SetInput(outline->GetOutput());
	//outlineMapper->SetInputConnection(outline->GetOutputPort()); //-> they are the same only for backward compatibility
	
	
	//Graphics Pipeline//////////////////////////////////////////////
	vtkActor *STLModelActor = vtkActor::New();
	STLModelActor->SetMapper(STLModelMapper);

	vtkActor *outlineActor=vtkActor::New();
	outlineActor->SetMapper(outlineMapper);
	//STLModelActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//STLModelActor->RotateX(30.0);
	//STLModelActor->RotateY(-45.0);

	//vtkLODActor *STLModelLODActor = vtkLODActor::New();
	//STLModelLODActor->SetMapper(STLModelMapper);

	//CREATE the camera
	vtkCamera *camera=vtkCamera::New();
	camera->SetClippingRange(1.0, 20.0);
    camera->SetFocalPoint( 1.5, 1.5, 0);
    camera->SetPosition( 0.0, 0.0, 0.0);
    camera->SetViewUp( 0,0, 0);
	
	vtkRenderer *ren1=vtkRenderer::New();
	ren1->SetActiveCamera(camera);
	ren1->AddActor(STLModelActor);
	ren1->AddActor(outlineActor);
	//ren1->AddActor(STLModelLODActor);
	ren1->SetBackground(0.1,0.2,0.4);

	//===========set the axis===============================
	vtkCubeAxesActor2D *axes=vtkCubeAxesActor2D::New();
	//axes->SetInput(normals->GetOutput());
	axes->SetViewProp(STLModelActor);
	axes->SetCamera(ren1->GetActiveCamera());
	axes->SetLabelFormat("%6.4g");
	axes->SetFlyModeToOuterEdges();
	//axes->ShadowOn();
	axes->SetFlyModeToOuterEdges();
	axes->SetFontFactor(0.8);
	//(axes->GetProperty())->SetColor(1.0,1.0,1.0);
	ren1->AddActor(axes);
	//ren1->AddActor2D(axes);
	//ren1->AddViewProp(axes);
	//======================================================

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->SetWindowName("Testing vtkCubeActor2D");
	renWin->AddRenderer(ren1);	
	renWin->SetSize(500,500);
	renWin->WindowRemap();

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	//memory free
	STLModel->Delete();
	STLModelMapper->Delete();
	STLModelActor->Delete();
	//STLModelLODActor->Delete();
	outlineActor->Delete();
	normals->Delete();
	outline->Delete();
	camera->Delete();
	outlineMapper->Delete();
	axes->Delete();	
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();
}

void myVTK::stlModelShrink(){
	//Visualization Pipeline
	vtkSTLReader *STLModel= vtkSTLReader::New(); //Reader Source Object
	STLModel->SetFileName("test.stl");

	vtkShrinkPolyData *STLModelShrink= vtkShrinkPolyData::New();
	STLModelShrink->SetInput(STLModel->GetOutput());
	STLModelShrink->SetShrinkFactor(0.86);
	
	vtkPolyDataMapper *STLModelMapper= vtkPolyDataMapper::New();
	STLModelMapper->SetInput(STLModelShrink->GetOutput());

	//Graphics Pipeline
	//vtkActor *STLModelActor = vtkActor::New();
	//STLModelActor->SetMapper(STLModelMapper);
	//STLModelActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//STLModelActor->RotateX(30.0);
	//STLModelActor->RotateY(-45.0);

	vtkLODActor *STLModelLODActor = vtkLODActor::New();
	STLModelLODActor->SetMapper(STLModelMapper);

	vtkRenderer *ren1=vtkRenderer::New();
	//ren1->AddActor(STLModelActor);
	ren1->AddActor(STLModelLODActor);
	ren1->SetBackground(0.1,0.2,0.4);

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(500,500);

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	//memory free
	STLModel->Delete();
	STLModelMapper->Delete();
	//STLModelActor->Delete();
	STLModelLODActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();

}

void myVTK::cylinder(){
	//Visualization Pipeline
	vtkCylinderSource *cylinder= vtkCylinderSource::New(); //procedural source object
	cylinder->SetResolution(10);
	
	vtkPolyDataMapper *cylinderMapper= vtkPolyDataMapper::New();
	cylinderMapper->SetInput(cylinder->GetOutput());

	//Graphics Pipeline
	vtkActor *cylinderActor = vtkActor::New();
	cylinderActor->SetMapper(cylinderMapper);
	cylinderActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	cylinderActor->RotateX(30.0);
	cylinderActor->RotateY(-45.0);

	vtkRenderer *ren1=vtkRenderer::New();
	ren1->AddActor(cylinderActor);
	ren1->SetBackground(0.1,0.2,0.4);

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(500,500);

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	//memory free
	cylinder->Delete();
	cylinderMapper->Delete();
	cylinderActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();

}

void myVTK::stlModel(){
	//Visualization Pipeline
	vtkSTLReader *STLModel= vtkSTLReader::New(); //Reader Source Object
	STLModel->SetFileName("implant.stl");
	
	vtkPolyDataMapper *STLModelMapper= vtkPolyDataMapper::New();
	STLModelMapper->SetInput(STLModel->GetOutput());
	//STLModelMapper->SetInput(STLModel->GetOutputPort());

	//Graphics Pipeline
	vtkActor *STLModelActor = vtkActor::New();
	STLModelActor->SetMapper(STLModelMapper);
	STLModelActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//STLModelActor->RotateX(30.0);
	//STLModelActor->RotateY(-45.0);

	//vtkLODActor *STLModelLODActor = vtkLODActor::New();
	//STLModelLODActor->SetMapper(STLModelMapper);

	vtkRenderer *ren1=vtkRenderer::New();
	ren1->AddActor(STLModelActor);
	//ren1->AddActor(STLModelLODActor);
	ren1->SetBackground(0.1,0.2,0.4);

	vtkRenderWindow *renWin=vtkRenderWindow::New();
	renWin->AddRenderer(ren1);
	renWin->SetWindowName("WAHYUDIN PERMANA");
	renWin->Finalize();
	renWin->SetSize(800,800);

	//char *name;
	//CString nameString;	
	//name=renWin->GetWindowName();	
	//nameString=CString(name);
	//AfxMessageBox(nameString,0,0);

	vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renWin);

	vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	iren->SetInteractorStyle(style);
	

	iren->Initialize();
	iren->Start();

	//memory free
	STLModel->Delete();
	STLModelMapper->Delete();
	STLModelActor->Delete();
	//STLModelLODActor->Delete();
	ren1->Delete();
	renWin->Delete();
	iren->Delete();
	style->Delete();
}

void myVTK::cone(){
// TODO: Add your control notification handler code here
	 // 
  // Next we create an instance of vtkConeSource and set some of its
  // properties. The instance of vtkConeSource "cone" is part of a
  // visualization pipeline (it is a source process object); it produces data
  // (output type is vtkPolyData) which other filters may process.
  //
  vtkConeSource *cone = vtkConeSource::New();
  cone->SetHeight( 3.0 );
  cone->SetRadius( 1.0 );
  cone->SetResolution( 10 );
  
  // 
  // In this example we terminate the pipeline with a mapper process object.
  // (Intermediate filters such as vtkShrinkPolyData could be inserted in
  // between the source and the mapper.)  We create an instance of
  // vtkPolyDataMapper to map the polygonal data into graphics primitives. We
  // connect the output of the cone souece to the input of this mapper.
  //
  vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
  coneMapper->SetInputConnection( cone->GetOutputPort() );

  // 
  // Create an actor to represent the cone. The actor orchestrates rendering
  // of the mapper's graphics primitives. An actor also refers to properties
  // via a vtkProperty instance, and includes an internal transformation
  // matrix. We set this actor's mapper to be coneMapper which we created
  // above.
  //
  vtkActor *coneActor = vtkActor::New();
  coneActor->SetMapper( coneMapper );
  coneActor->GetProperty()->SetColor(0.1,1,0.1);

  //
  // Create the Renderer and assign actors to it. A renderer is like a
  // viewport. It is part or all of a window on the screen and it is
  // responsible for drawing the actors it has.  We also set the background
  // color here.
  //
  vtkRenderer *ren1= vtkRenderer::New();
  ren1->AddActor( coneActor );
  ren1->SetBackground( 1, 1, 1 );

  //
  // Finally we create the render window which will show up on the screen.
  // We put our renderer into the render window using AddRenderer. We also
  // set the size to be 300 pixels by 300.
  //
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer( ren1 );
  renWin->SetSize( 300, 300 );

  // 
  // The vtkRenderWindowInteractor class watches for events (e.g., keypress,
  // mouse) in the vtkRenderWindow. These events are translated into
  // event invocations that VTK understands (see VTK/Common/vtkCommand.h
  // for all events that VTK processes). Then observers of these VTK
  // events can process them as appropriate.
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  //
  // By default the vtkRenderWindowInteractor instantiates an instance
  // of vtkInteractorStyle. vtkInteractorStyle translates a set of events
  // it observes into operations on the camera, actors, and/or properties
  // in the vtkRenderWindow associated with the vtkRenderWinodwInteractor. 
  // Here we specify a particular interactor style.
  vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
  iren->SetInteractorStyle(style);

  //
  // Unlike the previous scripts where we performed some operations and then
  // exited, here we leave an event loop running. The user can use the mouse
  // and keyboard to perform the operations on the scene according to the
  // current interaction style. When the user presses the "e" key, by default
  // an ExitEvent is invoked by the vtkRenderWindowInteractor which is caught
  // and drops out of the event loop (triggered by the Start() method that
  // follows.
  //
  iren->Initialize();
  iren->Start();
  
  // 
  // Final note: recall that an observers can watch for particular events and
  // take appropriate action. Pressing "u" in the render window causes the
  // vtkRenderWindowInteractor to invoke a UserEvent. This can be caught to
  // popup a GUI, etc. So the Tcl Cone5.tcl example for an idea of how this
  // works.

  //
  // Free up any objects we created. All instances in VTK are deleted by
  // using the Delete() method.
  //
  cone->Delete();
  coneMapper->Delete();
  coneActor->Delete();
  ren1->Delete();
  renWin->Delete();
  iren->Delete();
  style->Delete();

}

void myVTK::boxWidget(){
	// TODO: Add your control notification handler code here
	// TODO: Add your control notification handler code here
	// 
  // Next we create an instance of vtkConeSource and set some of its
  // properties. The instance of vtkConeSource "cone" is part of a
  // visualization pipeline (it is a source process object); it produces data
  // (output type is vtkPolyData) which other filters may process.
  //
  vtkConeSource *cone = vtkConeSource::New();
  cone->SetHeight( 3.0 );
  cone->SetRadius( 1.0 );
  cone->SetResolution( 10 );
  
  // 
  // In this example we terminate the pipeline with a mapper process object.
  // (Intermediate filters such as vtkShrinkPolyData could be inserted in
  // between the source and the mapper.)  We create an instance of
  // vtkPolyDataMapper to map the polygonal data into graphics primitives. We
  // connect the output of the cone souece to the input of this mapper.
  //
  vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
  coneMapper->SetInputConnection( cone->GetOutputPort() );

  // 
  // Create an actor to represent the cone. The actor orchestrates rendering
  // of the mapper's graphics primitives. An actor also refers to properties
  // via a vtkProperty instance, and includes an internal transformation
  // matrix. We set this actor's mapper to be coneMapper which we created
  // above.
  //
  vtkActor *coneActor = vtkActor::New();
  coneActor->SetMapper( coneMapper );

  //
  // Create the Renderer and assign actors to it. A renderer is like a
  // viewport. It is part or all of a window on the screen and it is
  // responsible for drawing the actors it has.  We also set the background
  // color here.
  //
  vtkRenderer *ren1= vtkRenderer::New();
  ren1->AddActor( coneActor );
  ren1->SetBackground( 0.1, 0.2, 0.6 );

  //
  // Finally we create the render window which will show up on the screen.
  // We put our renderer into the render window using AddRenderer. We also
  // set the size to be 300 pixels by 300.
  //
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer( ren1 );
  renWin->SetSize( 300, 300 );

  // 
  // The vtkRenderWindowInteractor class watches for events (e.g., keypress,
  // mouse) in the vtkRenderWindow. These events are translated into
  // event invocations that VTK understands (see VTK/Common/vtkCommand.h
  // for all events that VTK processes). Then observers of these VTK
  // events can process them as appropriate.
  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);

  //
  // By default the vtkRenderWindowInteractor instantiates an instance
  // of vtkInteractorStyle. vtkInteractorStyle translates a set of events
  // it observes into operations on the camera, actors, and/or properties
  // in the vtkRenderWindow associated with the vtkRenderWinodwInteractor. 
  // Here we specify a particular interactor style.
  vtkInteractorStyleTrackballCamera *style = 
    vtkInteractorStyleTrackballCamera::New();
  iren->SetInteractorStyle(style);

  //
  // Here we use a vtkBoxWidget to transform the underlying coneActor (by
  // manipulating its transformation matrix). Many other types of widgets
  // are available for use, see the documentation for more details.
  //
  // The SetInteractor method is how 3D widgets are associated with the render
  // window interactor. Internally, SetInteractor sets up a bunch of callbacks
  // using the Command/Observer mechanism (AddObserver()). The place factor 
  // controls the initial size of the widget with respect to the bounding box
  // of the input to the widget.
  vtkBoxWidget *boxWidget = vtkBoxWidget::New();
  boxWidget->SetInteractor(iren);
  boxWidget->SetPlaceFactor(1.25);

  //
  // Place the interactor initially. The input to a 3D widget is used to 
  // initially position and scale the widget. The EndInteractionEvent is
  // observed which invokes the SelectPolygons callback.
  //
  boxWidget->SetProp3D(coneActor);
  boxWidget->PlaceWidget();
  vtkMyCallback *callback = vtkMyCallback::New();
  boxWidget->AddObserver(vtkCommand::InteractionEvent, callback);

  //
  // Normally the user presses the "i" key to bring a 3D widget to life. Here
  // we will manually enable it so it appears with the cone. 
  //
  boxWidget->On();

  //
  // Start the event loop.
  //
  iren->Initialize();
  iren->Start();
  
  //
  // Free up any objects we created. All instances in VTK are deleted by
  // using the Delete() method.
  //
  cone->Delete();
  coneMapper->Delete();
  coneActor->Delete();
  callback->Delete();
  boxWidget->Delete();
  ren1->Delete();
  renWin->Delete();
  iren->Delete();
  style->Delete();

}

//void myVTK::myVTK(){
//}

//void ~myVTK::myVTK(){
//}