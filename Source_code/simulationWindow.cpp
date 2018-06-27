#include "simulationWindow.h"

//VTK Library
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
#include <vtkAxesActor.h>
#include "vtkPlaneSource.h"
#include "vtkLineSource.h"
#include "vtkSphereSource.h"
#include "vtkTextProperty.h"
#include "vtkOutlineFilter.h"
#include "vtkPolyDataNormals.h" //Normal for cell

//Qt Library
#include <QFile>
#include <QTextStream>
#include <QMenu>
#include <QAction>
#include <QKeySequence>
#include <QMessageBox>
#include <QToolButton>
#include <QToolBar>
#include <QIcon>
#include <QFileDialog>


//C/C++ LIB
#include <conio.h>
#include <stdio.h> //C I/O library
#include <iostream> //C++ I/O library
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h> 
using namespace std;

//Own function
//#include "functionClass.h" //why there are errors when we include this???
							 //because it is already have .obj when it is called by mainwindow class
							 //SOLUTION: make simulator function as seperate class.
#include "Simulator.h"		 //solved with this solution

//EIGEN LIB
#include <Eigen/Dense>
using namespace Eigen;

simulationWindow::simulationWindow(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	//ui= new Ui::simulationWindow;
	//ui->setupUi(this);	

	//cretae signal and slot
	createSignalAndSlot();	
	valueInitialization();

	//Initializing VTK objects
	pointCloud = vtkPolyData::New();
	pointCloudMapper = vtkPolyDataMapper::New();
	mapper = vtkPolyDataMapper::New();
	actor = vtkActor::New();
	cubeAxesActor=vtkCubeAxesActor2D::New();
	axes=vtkCubeAxesActor2D::New();
	outlineActor=vtkActor::New();
	cameraGlobal=vtkCamera::New();
	pointsGlobal=vtkPoints::New();
	stripsGlobal=vtkCellArray::New();
	pointColorGlobal=vtkDoubleArray::New();

	//SETTING THE RENDERER
	ren = vtkRenderer::New();
	ren->SetBackground(0.53,0.66,1);
	
	ui.vtkWidget->GetRenderWindow()->AddRenderer(ren);
	ui.vtkWidget->setMaxRenderRateForImageCache(0.1);
	ui.vtkWidget->setAutomaticImageCacheEnabled(true);
		
	//========================================================================================	
}

simulationWindow::~simulationWindow()
{
	//Delete dynamic variable here
	//delete ui; //do not need, because in this case, the ui is static object
	pointsGlobal->Delete();
	stripsGlobal->Delete();
	pointColorGlobal->Delete();
	pointCloud->Delete();
	pointCloudMapper->Delete();
	mapper->Delete();
	actor->Delete();
	ren->Delete();
	cameraGlobal->Delete();
	cubeAxesActor->Delete();
	axes->Delete();
	outlineActor->Delete();
	cameraGlobal->Delete();
}

//Set data receieved from the caller function
void simulationWindow::setVTKData(vtkActor* actor1, vtkActor* outlineActor1, vtkCubeAxesActor2D* cubeAxesActor1){
	actor=actor1;
	if(points<50000){
		actor->GetProperty()->SetPointSize(10.0);
	}
	outlineActor=outlineActor1;
	cubeAxesActor=cubeAxesActor1;
	
	//Setting camera and re-render according to the received actor
	vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*4);
	camera->Azimuth(-45.0);
	camera->Elevation(-45.0);
	ren->SetActiveCamera(camera);
	ren->AddActor(actor);
	ren->GetRenderWindow()->Render();
	camera->Delete();	
}

void simulationWindow::setGeneralData(MatrixXd m1,double minX1,double minY1,double minZ1,double maxX1,double maxY1,double maxZ1,double length1,double width1,double height1,double deltaX1,double deltaY1,int points1){
//void simulationWindow::setGeneralData(MatrixXd m1,double *minX1,double *minY1,double *minZ1,double *maxX1,double *maxY1,double *maxZ1,double *length1,double *width1,double *height1,double *deltaX1,double *deltaY1,int *points1){
	m=m1;
	minX=minX1;
	minY=minY1;
	minZ=minZ1;
	maxX=maxX1;
	maxY=maxY1;
	maxZ=maxZ1;
	length=length1;
	width=width1;
	height=height1;
	deltaX=deltaX1;
	deltaY=deltaY1;
	points=points1;

	/*//using pointer
	m=m1;
	minX=*minX1;
	minY=*minY1;
	minZ=*minZ1;
	maxX=*maxX1;
	maxY=*maxY1;
	maxZ=*maxZ1;
	length=*length1;
	width=*width1;
	height=*height1;
	deltaX=*deltaX1;
	deltaY=*deltaY1;
	points=*points1;*/

	showData();
}

void simulationWindow::setNoOfPoints(int points1){
	points=points1;
}

void simulationWindow::showData(){
	ui.textEdit->setText("Point cloud data: \n\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Number of points = "+QString::number(points)+"\n\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Max. X = "+QString::number(maxX)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Min. X = "+QString::number(minX)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Max. Y = "+QString::number(maxY)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Min. Y = "+QString::number(minY)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Max. Z = "+QString::number(maxZ)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Min. Z = "+QString::number(minZ)+"\n\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Pixel distance X: "+QString::number(deltaX)+"\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Pixel distance Y: "+QString::number(deltaY)+"\n\n");
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Bounding Box (LxWxH)= "+QString::number(length)+" x "+QString::number(width)+" x "+QString::number(height)+"\n\n");
	
}

void simulationWindow::valueInitialization(){

	//variables
	minX=0.0;
	minY=0.0; 
	minZ=0.0;
	maxX=0.0; 
	maxY=0.0; 
	maxZ=0.0;
	length=0.0; 
	width=0.0;
	height=0.0;
	deltaX=0.0;
	deltaY=0.0;
	points=0;

	//Widget conmponents
	ui.comboBoxVariogramModel->addItem("Gaussian");
	ui.comboBoxVariogramModel->addItem("Exponential");
	ui.comboBoxVariogramModel->addItem("Spherical"); //to accsess use currentText()

	//setFlags
	flag_animation_enabled=0;
}

//SIGNALs and SLOTS
void simulationWindow::createSignalAndSlot(){
	connect(ui.pushButtonSimulation,SIGNAL(clicked()),this,SLOT(pushButtonSimulation_clicked()));
	connect(ui.pushButtonCancel,SIGNAL(clicked()),this,SLOT(pushButtonCancel_clicked()));
}

void simulationWindow::pushButtonSimulation_clicked(){
	//Testing passing parameters
	/*double points_double=points;
	QString text(QString::number(points_double));//QString text=QString::number(points_double) is does not work;	
	QMessageBox::information(this,tr("Message"),text); //before there was error due to forget to input library
	*/
	
	Simulator simul;
	//Simulation run
	//=========================================================================================================
	double s=0.0, n=0.0, r=0.0;
	//copying s,n,r value fro QLineEdit
	s=QString(ui.lineEditSill->text()).toDouble();
	n=QString(ui.lineEditNugget->text()).toDouble();
	r=QString(ui.lineEditRange->text()).toDouble();
	
	int no_runs=0;
	//copying number of runs value fro QLineEdit
	no_runs=QString(ui.lineEditRun->text()).toInt();

	int mode=0; //to determine the model of the used variogram
	//checking the mode value (1,2 or 3)
	QString text_gauss="Gaussian";
	QString text_exp="Exponential";
	QString text_sph="Spherical";
	QString mode_type(ui.comboBoxVariogramModel->currentText());
	if(mode_type.compare(text_gauss)==0){
		//QMessageBox::information(this,tr("Message"),"Gaussian");
		mode=1;
	}
	else if(mode_type.compare(text_exp)==0){
		//QMessageBox::information(this,tr("Message"),"Exponential");
		mode=2;
	}
	else if(mode_type.compare(text_sph)==0){
		//QMessageBox::information(this,tr("Message"),"Spherical");
		mode=3;
	}
	else{
		mode=1;// default is gaussian variogram model.
	}

	//checking animation enabled or disabled
	if(ui.checkBoxAnimation->isChecked()){
		flag_animation_enabled=1;
		//QMessageBox::information(this,tr("Message"),"animation ON");
	}
	else{
		flag_animation_enabled=0;
	}

	//point perturbation process
	MatrixXd m_perturbed, m_temp, dz_all;
	double dz=0.0;

	m_perturbed.resize(m.rows(),3);

	int i=0, j=0, k=0;
	int indexPoints=0, indexMaxPoint=0, indexPoint=0;
	int maxPoints=3000;	//we process each batch contains 3000 points

	if(m.rows()<=3000){
		dz_all.resize(1,m.rows());
		m_temp.resize(m.rows(),3);
		dz_all=MatrixXd::Zero(1,m.rows());
		m_temp=MatrixXd::Zero(m.rows(),3);
	}
	else{ //if number of points >3000
		dz_all.resize(1,maxPoints);
		m_temp.resize(maxPoints,3);
		dz_all=MatrixXd::Zero(1,maxPoints);
		m_temp=MatrixXd::Zero(maxPoints,3);
	}

	ui.textEdit->setText(ui.textEdit->toPlainText()+"Simulation started\n");
	Sleep(100);

	for(k=0;k<no_runs;k++){//number of runs
	
	indexPoints=0;
	indexMaxPoint=0;
	indexPoint=0;
	for(i=0;i<points;i++){ //collecting points as a batch
		if(indexMaxPoint<maxPoints){
			m_temp(indexMaxPoint,0)=m(i,0);
			m_temp(indexMaxPoint,1)=m(i,1);
			m_temp(indexMaxPoint,2)=m(i,2);
			indexMaxPoint++;
		}
		if((indexMaxPoint==maxPoints) || (i==points-1)){//the point perturbation
			dz_all=simul.errorSimulator(m_temp, 1, s, n, r);
			
			if(i==points-1){
				for(j=0;j<points-1;j++){
					dz=dz_all(0,j);
					m_perturbed(indexPoint,0)=m(indexPoint,0);
					m_perturbed(indexPoint,1)=m(indexPoint,1);
					m_perturbed(indexPoint,2)=m(indexPoint,2)+dz;
					indexPoint++;
				}
			}
			else{
				for(j=0;j<maxPoints;j++){
					dz=dz_all(0,j);
					m_perturbed(indexPoint,0)=m(indexPoint,0);
					m_perturbed(indexPoint,1)=m(indexPoint,1);
					m_perturbed(indexPoint,2)=m(indexPoint,2)+dz;
					indexPoint++;
				}
			}

			/*
			//SHOW POINT AFTER PARTIAL PERTURBATION
			if(flag_animation_enabled){//Point simulation if the animation check box is selected
				//if(k==0){//the first run
					//ui.vtkWidget->GetRenderWindow()->RemoveRenderer(ren);
				//}
				//cameraGlobal=ren->GetActiveCamera();
				//ren->GetRenderWindow()->Render();
				pointAnimation(m_perturbed);
				//delay(100);
				Sleep(100);
				//QMessageBox::information(this,tr("Message"),"ok");
			}*/
			
			//m_temp.resize(maxPoints,3);
			if(m.rows()<=3000){
				dz_all.resize(1,m.rows());
				m_temp.resize(m.rows(),3);
				dz_all=MatrixXd::Zero(1,m.rows());
				m_temp=MatrixXd::Zero(m.rows(),3);
			}
			else{ //if number of points >3000
				dz_all.resize(1,maxPoints);
				m_temp.resize(maxPoints,3);
				dz_all=MatrixXd::Zero(1,maxPoints);
				m_temp=MatrixXd::Zero(maxPoints,3);
			}
			indexMaxPoint=0;	
		}
	} //end of for(i=0;i<points;i++){

	//SHOW POINT AFTER EACH COMPLETE PERTURBATION
	if(flag_animation_enabled){//Point simulation if the animation check box is selected		
		cameraGlobal=ren->GetActiveCamera();
		//ren->GetRenderWindow()->Render();
		pointAnimation(m_perturbed);
		Sleep(1000);		
	}

	/*//---CHEKCING PER ITERATION AND WTIRING m_perturbed to file---------------------
	QMessageBox::information(this,tr("Message"),"Iteration"+QString::number(k));
	FILE *pFile2;
	char fileName2[200];
	sprintf(fileName2,"C:\\Documents and Settings\\Wahyudin\\Desktop\\ALICONA\\m_perturbed.txt");
	printf("\n");
	puts(fileName2);
	printf("\n");
	pFile2=fopen(fileName2,"a");
	int iter;
	for(iter=0;iter<m.rows();iter++){	
		fprintf(pFile2,"%.9f %.9f %.9f\n",(float) m_perturbed(iter,0), (float) m_perturbed(iter,1), (float) m_perturbed(iter,2));
	}  
	fprintf(pFile2,"\n");
	fclose(pFile2);
	//-----------------------------------------------------------------------------*/

	Sleep(1000);	
	ui.textEdit->setText(ui.textEdit->toPlainText()+"run "+QString::number(k)+ " of " +QString::number(no_runs)+"\n");
	} //end of for(k=0;k<no_runs;k++){
	ui.textEdit->setText(ui.textEdit->toPlainText()+"Simulation finished\n\n");

	//SHOW POINT AFTER ALL PERTURBATION FINISHED
	/*if(flag_animation_enabled){//Point simulation if the animation check box is selected		
		cameraGlobal=ren->GetActiveCamera();
		//ren->GetRenderWindow()->Render();
		pointAnimation(m_perturbed);
		Sleep(100);		
	}*/

	QMessageBox::information(this,tr("Message"),"SIMULATION FINISHED");
	
	//=========================================================================================================	

}

void simulationWindow::pointAnimation(MatrixXd m_perturbed){
	//This function is to show the point aniumation in the perturbation process

	/*//VISUALIZATION Pipeline
	vtkPoints *points=vtkPoints::New(); //Point Data
	vtkCellArray *strips=vtkCellArray::New(); 
	vtkDoubleArray* pointColor = vtkDoubleArray::New();
	pointColor->SetName("pointColor");
	cameraGlobal=ren->GetActiveCamera();

	double point[3];

	int i=0;
	int no_of_points=0;
	no_of_points=m_perturbed.rows();
	strips->InsertNextCell(no_of_points+1);
	for(i=0;i<no_of_points;i++){
		point[0]=m_perturbed(i,0);
		point[1]=m_perturbed(i,1);
		point[2]=m_perturbed(i,2);

		points->InsertPoint(i,point);
		
		strips->InsertCellPoint(i);

		pointColor->InsertNextValue(point[2]*1000.0);
	}
	
	//---------------GLOBAL VTK OBJECT-------------------------------------
    pointCloud->Reset();
	pointCloud->SetPoints(points);
	pointCloud->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
	pointCloud->SetVerts(strips); //show as points
	//set color from pointCloud (vtkPolyData)
	pointCloud->Update(); //We need to update because it will be rendered not now (later)--> VTK Lazy update
	pointCloudMapper->SetInput(pointCloud);
	pointCloudMapper->Update(); //We need to update because it will be rendered not now (later) --> VTK Lazy update

	//GRAPHICAL pipeline
	actor->SetMapper(pointCloudMapper);
	actor->GetProperty()->SetPointSize(10.0); //set point size, actually we set from the mapper or the polydata (dataset)
	actor->GetProperty()->SetColor(0,1,0); //gren color, actually we set from the mapper or the polydata (dataset)
	//actor->Update();
		
	ren->RemoveAllViewProps();
	ren->SetActiveCamera(cameraGlobal);
	ren->AddActor(actor);
	ren->GetRenderWindow()->Render();
	//QMessageBox::information(this,tr("Message"),"OK 1");
	Sleep(100);

	//Deleting VTK dynamic object
	points->Delete();
	strips->Delete();
	pointColor->Delete();
	//-----------------------------------------------------------------------*/

	/*//---------------LOCAL VTK OBJECT---------------------------------------
	vtkPolyData* pointCloudLocal=vtkPolyData::New();
	vtkPolyDataMapper* pointCloudMapperLocal=vtkPolyDataMapper::New();
	vtkActor* actorLocal=vtkActor::New();	

	pointCloudLocal->SetPoints(points);
	pointCloudLocal->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
	pointCloudLocal->SetVerts(strips); //show as points
	pointCloudLocal->Update();

	pointCloudMapperLocal->SetInput(pointCloud);
	pointCloudMapperLocal->Update();

	actorLocal->SetMapper(pointCloudMapper);
	actorLocal->GetProperty()->SetPointSize(10.0); //set point size, actually we set from the mapper or the polydata (dataset)
	actorLocal->GetProperty()->SetColor(0,1,0); 
	
	//ren->RemoveAllViewProps();
	ren->SetActiveCamera(cameraGlobal);
	ren->AddActor(actorLocal);
	ren->GetRenderWindow()->Render();
	Sleep(100);
	
	//Deleting VTK dynamic object
	points->Delete();
	strips->Delete();
	pointColor->Delete();
	pointCloudLocal->Delete();
	pointCloudMapperLocal->Delete();
	actorLocal->Delete();
	//-----------------------------------------------------------------------*/

	/*//---------------LOCAL VTK OBJECT including the renderer---------------------------------------
	//VISUALIZATION Pipeline
	vtkPoints *pointsLocal=vtkPoints::New();
	vtkCellArray *stripsLocal=vtkCellArray::New();
	vtkDoubleArray* pointColorLocal=vtkDoubleArray::New();
	vtkCamera* cameraLocal=vtkCamera::New();

	pointColorLocal->SetName("pointColor");
	cameraLocal=ren->GetActiveCamera();

	double point[3];

	int i=0;
	int no_of_points=0;
	no_of_points=m_perturbed.rows();

	//stripsGlobal->InsertNextCell(no_of_points+1);
	if(no_of_points<100000){
		stripsLocal->InsertNextCell(100001);
	}
	else{
		stripsLocal->InsertNextCell(no_of_points+1);
	}

	for(i=0;i<no_of_points;i++){
		point[0]=m_perturbed(i,0);
		point[1]=m_perturbed(i,1);
		point[2]=m_perturbed(i,2);

		pointsLocal->InsertPoint(i,point);
		
		stripsLocal->InsertCellPoint(i);
	}
	int iter;
	if(no_of_points<100000){
		for(iter=no_of_points;iter<=100000;iter++){
			pointsLocal->InsertPoint(iter,point); //inserting the value of the last point
			stripsLocal->InsertCellPoint(iter);
			//pointColor->InsertNextValue(point[2]*1000.0);
		}
	}
	
	vtkPolyData* pointCloudLocal=vtkPolyData::New();
	vtkPolyDataMapper* pointCloudMapperLocal=vtkPolyDataMapper::New();
	vtkActor* actorLocal=vtkActor::New();	
	vtkRenderer* renLocal=vtkRenderer::New();

	pointCloudLocal->SetPoints(pointsLocal);
	pointCloudLocal->GetPointData()->SetScalars(pointColorLocal);//set the point color based on the value we insert to the pointcolor object
	pointCloudLocal->SetVerts(stripsLocal); //show as points
	pointCloudLocal->Update();

	pointCloudMapperLocal->SetInput(pointCloudLocal);
	pointCloudMapperLocal->Update();
	pointCloudMapperLocal->Render(ren,actorLocal);

	actorLocal->SetMapper(pointCloudMapperLocal);
	actorLocal->GetProperty()->SetPointSize(10.0); //set point size, actually we set from the mapper or the polydata (dataset)
	actorLocal->GetProperty()->SetColor(0,1,0); 
	actorLocal->Render(ren,pointCloudMapperLocal);
	
	//ren->RemoveAllViewProps();
	ren->SetActiveCamera(cameraLocal);
	actorLocal->SetVisibility(false);
	ren->AddActor(actorLocal);	
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	
	actorLocal->SetVisibility(true);	
	ren->GetRenderWindow()->Render();
	//QMessageBox::information(this,tr("Message"),"OKKKKKKKKKBOS");
	
	//Deleting VTK dynamic object
	pointsLocal->Delete();
	stripsLocal->Delete();
	pointColorLocal->Delete();
	cameraLocal->Delete();
	pointCloudLocal->Delete();
	pointCloudMapperLocal->Delete();
	actorLocal->Delete();
	renLocal->Delete();
	//-----------------------------------------------------------------------*/

	//------------------------ALL GLOBAL ---------------------------------------
	//VISUALIZATION Pipeline
	pointsGlobal->Reset(); //to reset the data inside the vtkPoints object (Release the data)
	stripsGlobal->Reset(); //to reset the data inside the vtkCellArray object (Release the data)

	pointColorGlobal->SetName("pointColor");
	cameraGlobal=ren->GetActiveCamera();

	double point[3];

	int i=0;
	int no_of_points=0;
	no_of_points=m_perturbed.rows();

	//stripsGlobal->InsertNextCell(no_of_points+1);
	if(no_of_points<100000){
		stripsGlobal->InsertNextCell(100001);
	}
	else{
		stripsGlobal->InsertNextCell(no_of_points+1);
	}

	for(i=0;i<no_of_points;i++){
		point[0]=m_perturbed(i,0);
		point[1]=m_perturbed(i,1);
		point[2]=m_perturbed(i,2);

		pointsGlobal->InsertPoint(i,point);
		
		stripsGlobal->InsertCellPoint(i);
	}
	int iter;
	if(no_of_points<100000){
		for(iter=no_of_points;iter<=100000;iter++){
			pointsGlobal->InsertPoint(iter,point); //inserting the value of the last point
			stripsGlobal->InsertCellPoint(iter);
			//pointColor->InsertNextValue(point[2]*1000.0);
		}
	}
	
	//---------------GLOBAL VTK OBJECT-------------------------------------
    pointCloud->Reset(); //to reset the data inside the vtkPolyData dataset object (Release the data)
	pointCloud->SetPoints(pointsGlobal);
	//pointCloud->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
	pointCloud->SetVerts(stripsGlobal); //show as points	
	pointCloud->Update(); //We need to update because it will be rendered not now (later)--> VTK Lazy update

	pointCloudMapper->SetInput(pointCloud);
	pointCloudMapper->Update(); //We need to update because it will be rendered not now (later) --> VTK Lazy update
	pointCloudMapper->Render(ren,actor);

	//GRAPHICAL pipeline
	actor->SetMapper(pointCloudMapper);
	actor->GetProperty()->SetPointSize(10.0); //set point size, actually we set from the mapper or the polydata (dataset)
	actor->GetProperty()->SetColor(0,1,0); //gren color, actually we set from the mapper or the polydata (dataset)
	actor->Render(ren,pointCloudMapper);
		
	//ren->RemoveAllViewProps();
	ren->SetActiveCamera(cameraGlobal);
	actor->SetVisibility(false);
	//ren->AddActor(actor);
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();
	actor->SetVisibility(true);
	ren->GetRenderWindow()->Render();

	//pointCloudMapper->Render(ren,actor);
	//actor->Render(ren,pointCloudMapper);

	//QMessageBox::information(this,tr("Message"),"OK 1");

	/*Sleep(100);
	ui.vtkWidget->GetRenderWindow()->RemoveRenderer(ren);
	Sleep(100);
	ui.vtkWidget->GetRenderWindow()->AddRenderer(ren);
	ren->GetRenderWindow()->Render();*/	
	
	//-----------------------------------------------------------------------

	
}

void simulationWindow::pushButtonCancel_clicked(){
	//this.close();
	close();
}
