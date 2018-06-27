#include "mainwindow.h"
#include "ui_mainwindow.h"

//VTK Library
#include "vtkSmartPointer.h"
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
#include "vtkType.h" 
#include "vtkDelaunay2D.h"
#include "vtkBox.h"
#include "vtkCubeSource.h"
#include "vtkClipPolyData.h"
#include "vtkCutter.h"
#include "vtkIdFilter.h"
#include "vtkIdFilter.h" //Library to get points and cells
#include "vtkIdTypeArray.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkIdList.h"
#include "vtkTriangle.h" //to build STL dataset

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
#include <assert.h>
using namespace std;

////--OpenCV Library (reccomended opencv2.2)-----
//#include <opencv/cv.h>             		    // OPENCVMain OpenCV library.
//#include <opencv/highgui.h>	// OpenCV functions for files and graphical windows.
//#include <opencv/cxcore.h>
////#include <opencv/cvaux.h>
//#include <opencv/ml.h>

////PCL Library
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/common_headers.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/ModelCoefficients.h>

/*#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include "vtkCylinderSource.h"
#include <vtkPolyDataMapper.h>*/

//own library and VTK library
//#include "myVTK.h"
#include "functionClass.h"

//EIGEN LIB
#include <Eigen/Dense>
using namespace Eigen;

//#include "QVTKWidget.h" //NOTE: If we put the QVTKWidget on the designer, we do not need to add this

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
	
	/*QSize vtkSize;
	vtkSize.setHeight(100);
	vtkSize.setWidth(100);
	
	QVTKWidget *vtkWidget;
	//vtkWidget = new QVTKWidget(this,0);
	vtkWidget = new QVTKWidget(this,Qt::Widget);
	//vtkWidget->create(0, true, true);
	//vtkWidget->setFixedSize(
	//vtkWidget = new QVTKWidget(this,Qt::Widget);	
	//vtkWidget->GetRenderWindow()->AddRenderer(ren);*/

	ui->setupUi(this);	

	ren = vtkRenderer::New();
	ren->SetBackground(0.53,0.66,1);
	ui->vtkWidget->GetRenderWindow()->AddRenderer(ren);	

    //CREATING MENU
    createActions();
    createMenus();

    //CREATING TOOLBAR
    createToolBar();

	//CREATING SIGNAL (event) and SLOT (called function)
	createSignalAndSlot();

	//Defining tool tip: a help text when one mouse over a qt component
	setHelpText();

	//Set flags and component status
	setFlags();
	setComponentStatus();	
	setComponentIcon();

	//Initializing VTK objects
	pointCloud = vtkPolyData::New();
	pointCloudMapper = vtkPolyDataMapper::New();
	mapper = vtkPolyDataMapper::New();
	actor = vtkActor::New();
	cubeAxesActor=vtkCubeAxesActor2D::New();
	axes=vtkCubeAxesActor2D::New();
	outlineActor=vtkActor::New();
	STLData=vtkPolyData::New();

	//Initializing others object and Variables
	resetMaxMinValue();
	length=0.0;
	width=0.0;
	height=0.0;
	deltaX=0.0;
	deltaY=0.0;
	num_of_points=0;

}

MainWindow::~MainWindow() //Class destructor
{
    delete ui;
	//source->Delete(); //This is how to delete vtk dynamic object
	pointCloud->Delete();
	pointCloudMapper->Delete();
	mapper->Delete();
	actor->Delete();
	ren->Delete();
	cubeAxesActor->Delete();
	axes->Delete();
	outlineActor->Delete();
	STLData->Delete();
}

void MainWindow::setHelpText(){

}

void MainWindow::setComponentStatus(){
	//flag_file_is_opened = 0, means that no file is opened yet, and otherwise
	if(flag_file_is_opened==0){
		saveButton->setEnabled(false);

		deleteButton->setEnabled(false);
		cropButton->setEnabled(false);
		filterButton->setEnabled(false);

		lineFittingButton->setEnabled(false);
		planeFittingButton->setEnabled(false);
		sphereFittingButton->setEnabled(false);
		cylinderFittingButton->setEnabled(false);

		straightnessButton->setEnabled(false);
		flatnessButton->setEnabled(false);
		sphereFormButton->setEnabled(false);
		cylindricityButton->setEnabled(false);
		holeDepthButton->setEnabled(false);
		boxDepthButton->setEnabled(false);

		STLGenerationButton->setEnabled(false);
		triangleMeshingButton->setEnabled(false);
		STLDeleteButton->setEnabled(false);
		STLCropButton->setEnabled(false);

		simulationButton->setEnabled(false);

		ui->pushButtonAxes->setEnabled(false);
	}
	else{
		saveButton->setEnabled(true);

		deleteButton->setEnabled(true);
		cropButton->setEnabled(true);
		filterButton->setEnabled(true);

		lineFittingButton->setEnabled(true);
		planeFittingButton->setEnabled(true);
		sphereFittingButton->setEnabled(true);
		cylinderFittingButton->setEnabled(true);

		straightnessButton->setEnabled(true);
		flatnessButton->setEnabled(true);
		sphereFormButton->setEnabled(true);
		cylindricityButton->setEnabled(true);
		holeDepthButton->setEnabled(true);
		boxDepthButton->setEnabled(true);

		STLGenerationButton->setEnabled(true);
		triangleMeshingButton->setEnabled(true);
		STLDeleteButton->setEnabled(true);
		STLCropButton->setEnabled(true);

		simulationButton->setEnabled(true);

		ui->pushButtonAxes->setEnabled(true);
	}
}

void MainWindow::setFlags(){
	flag_file_is_opened=0;	
	flag_axis_shown=0;
	flag_simulation_window=0;
}

void MainWindow::setComponentIcon(){
	ui->pushButtonAxes->setIcon(QIcon(":/resources/axes.PNG"));
	ui->pushButtonAxes->setToolTip("Show or hide axes and bounding box");
}

void MainWindow::on_pushButton_clicked()
{
    //ui->lineEdit->setText("Wahyudin P. Syam");
	double r,g,b;
	r=QString(ui->lineEdit->text()).toDouble();
	g=QString(ui->lineEdit2->text()).toDouble();
	b=QString(ui->lineEdit3->text()).toDouble();

	//Updating the renderer background color
	ren->SetBackground(r,g,b);
	ren->GetRenderWindow()->Render();
}

void MainWindow::on_pushButton_2_clicked()
{
    /*//READING FILE USING Qt
	//==========================================================================
    //QFile inputFile("test.txt"); //it does not work without the complete path
    QFile inputFile("F:\\MyQtApps\\QtDesktopApp\\DesktopApp\\test.txt");
    inputFile.open(QIODevice::ReadOnly);

    QTextStream in(&inputFile);
    QString line=in.readAll();
    inputFile.close();

    line.clear();

    //READING FILE USING C style
    FILE* pFile;
    char text[250];
    //std::string text;
    pFile=fopen("F:\\MyQtApps\\QtDesktopApp\\DesktopApp\\test.txt","r");
    while(fgets(text,250,pFile)){
        //std::getline( pFile, text )
        //line.append(QString::fromStdString(text));
        line.append(text);		
    }
    fclose(pFile);

    ui->textEdit->setPlainText(line);
    QTextCursor cursor=ui->textEdit->textCursor();
    cursor.movePosition(QTextCursor::Start,QTextCursor::MoveAnchor,1);
	//=====================================================================*/

	ui->textEdit->clear();

}

void MainWindow::newFile(){
    //QMessageBox::information(this,tr("Message"),tr("You pressed new"));
	ren->RemoveAllViewProps();
    ren->ResetCamera();
	ren->GetRenderWindow()->Render();
	clearDataDisplay();
	resetMaxMinValue();
}

void MainWindow::about(){
    //QMessageBox::question(this,tr("About"),tr("Copyright @ Metrology Lab. - Politecnico di Milano \n \n Wahyudin P. Syam \n February, 2015"));
	QMessageBox::information(this,tr("About"),tr("Copyright @ Metrology Lab. - Politecnico di Milano \n \n Wahyudin P. Syam \n February, 2015"));
}

//CREATING MENUS and ACTIONS (SUB-MENU)
void MainWindow::createActions(){
    newAct= new QAction(tr("&New"),this);
    newAct->setShortcuts(QKeySequence::New);
    newAct->setStatusTip(tr("Create a new file"));
    connect(newAct, SIGNAL(triggered()),this,SLOT(newFile())); //this is the linking between SIGNAL and SLOT

    aboutAct= new QAction (tr("&About"),this);
    aboutAct->setShortcuts(QKeySequence::New);
    aboutAct->setStatusTip(tr("About the software"));
    connect(aboutAct, SIGNAL(triggered()),this,SLOT(about())); //this is the linking between SIGNAL and SLOT

}

void MainWindow::createMenus(){
    fileMenu=ui->menuBar->addMenu(tr("&File")); //this one is also functioning
    //fileMenu=menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(newAct);

    helpMenu=ui->menuBar->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}


//SIGNAL and SLOT
//======================================================================================================
//SIGNAL
void MainWindow::createSignalAndSlot(){
	//connect(ui->horizontalSlider, SIGNAL(sliderPressed()),this,SLOT(setValueSlider1())); //this is the linking between SIGNAL and SLOT
	connect(ui->horizontalSlider, SIGNAL(sliderReleased()),this,SLOT(setValueSlider1())); //this is the linking between SIGNAL and SLOT
	connect(ui->horizontalSlider2, SIGNAL(sliderReleased()),this,SLOT(setValueSlider1()));
	connect(ui->horizontalSlider3, SIGNAL(sliderReleased()),this,SLOT(setValueSlider1()));
	connect(ui->pushButton_3,SIGNAL(clicked()),this,SLOT(saveOutput()));
	connect(ui->pushButtonUP,SIGNAL(clicked()),this,SLOT(upView()));
	connect(ui->pushButtonRIGHT,SIGNAL(clicked()),this,SLOT(rightView()));
	connect(ui->pushButtonBottom,SIGNAL(clicked()),this,SLOT(bottomView()));
	connect(ui->pushButtonLEFT,SIGNAL(clicked()),this,SLOT(leftView()));
	connect(ui->pushButtonCLEAR,SIGNAL(clicked()),this,SLOT(clearView()));
	connect(ui->pushButtonSHOW,SIGNAL(clicked()),this,SLOT(showOriginalData()));
	connect(ui->pushButtonAxes,SIGNAL(clicked()),this,SLOT(showHideAxes()));
}
//SLOT
void MainWindow::showHideAxes(){
	//to show and hide the axis and the bounding box

	//vtkCubeAxesActor2D *axes=vtkCubeAxesActor2D::New();

	if(flag_axis_shown==0){//to show
		//creating vtkTextProperty to set the text property of the cube axes
		vtkTextProperty *textProperty=vtkTextProperty::New();
		textProperty->SetColor(0, 0, 1);
		//textProperty->ShadowOn();
		textProperty->SetFontSize(5);

		//setting up the cube axes actor
		axes->SetViewProp(actor);
		axes->SetCamera(ren->GetActiveCamera());
		axes->SetLabelFormat("%6.4g");
		axes->SetFlyModeToClosestTriad();
		axes->ScalingOff();
		//axes->SetAxisTitleTextProperty(textProperty.GetPointer());
		//axes->SetAxisLabelTextProperty(textProperty.GetPointer());
		axes->SetAxisTitleTextProperty(textProperty);
		axes->SetAxisLabelTextProperty(textProperty);	

		//add actor to renderer
		ren->AddViewProp(axes);
		ren->AddActor(outlineActor);
		ren->GetRenderWindow()->Render();

		textProperty->Delete();	

		flag_axis_shown=1;
	}
	else{//to hide
		ren->RemoveActor2D(axes);
		ren->RemoveActor(outlineActor);
		ren->GetRenderWindow()->Render();
		flag_axis_shown=0;
	}

	//axes->Delete();
}

void MainWindow::clearView(){
	ren->RemoveAllViewProps();
    ren->ResetCamera();
	ren->GetRenderWindow()->Render();
}

void MainWindow::showOriginalData(){
	//Clear the view inside the renderer
	ren->RemoveAllViewProps();

	//Re-show the original points inside matrix m (global matrix)
	ren->ResetCamera();
	
	vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*2);
	camera->Azimuth(-45.0);
	camera->Elevation(-45.0);		
	ren->SetActiveCamera(camera);	
	
	//actor->SetVisibility(true);
	ren->AddActor(actor);
	//ren->Render();
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	
	//ui->vtkWidget->GetRenderWindow()->Render();
	
	camera->Delete();
}

void MainWindow::setValueSlider1(){
	//ui->lineEdit->setText("Wahyudin P. Syam");
	double r,g,b;
	
	double val=0.0;
	val=ui->horizontalSlider->value();
	val=val/1000;
	//ui->lineEdit->setText(QString::number(ui->horizontalSlider->value()));
	ui->lineEdit->setText(QString::number(val)); //Convert the integer value to QString format

	val=ui->horizontalSlider2->value();
	val=val/1000;
	//ui->lineEdit->setText(QString::number(ui->horizontalSlider->value()));
	ui->lineEdit2->setText(QString::number(val));

	val=ui->horizontalSlider3->value();
	val=val/1000;
	//ui->lineEdit->setText(QString::number(ui->horizontalSlider->value()));
	ui->lineEdit3->setText(QString::number(val));
	
	r=QString(ui->lineEdit->text()).toDouble();
	g=QString(ui->lineEdit2->text()).toDouble();
	b=QString(ui->lineEdit3->text()).toDouble();

	//Updating the renderer background color
	ren->SetBackground(r,g,b);
	ren->GetRenderWindow()->Render();
}

void MainWindow::saveOutput(){
	QFileDialog *fileDlg=new QFileDialog(this);
	QString path= fileDlg->getSaveFileName();
	QFile f(path);
	if(f.open(QIODevice::WriteOnly)){
	//writing the data
		QTextStream stream(&f);
		stream <<ui->textEdit->toPlainText()<<endl;
	}
	else{
		QMessageBox::information(this,tr("sava file information"),tr("Filename should be defined!"));
	}
	f.close();
}

void MainWindow::upView(){
	ren->RemoveAllViewProps(); //remove all actor in the actor list
	ren->ResetCamera();
	
	vtkCamera *camera=vtkCamera::New();

	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*2);
	//camera->SetPosition(0,0,length*4);
	//camera->Azimuth(-45.0);
	//camera->Elevation(-45.0);	//Pitch(), Azimuth() and Roll()
	//camera->SetViewUp(0,1,0);	
	ren->SetActiveCamera(camera);	
	
	ren->AddActor(actor);	
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	

	/*ui->vtkWidget->update();
	ui->vtkWidget->show();
	camera->SetPosition(0,0,length*2);
	ren->SetActiveCamera(camera);		
	ui->vtkWidget->GetRenderWindow()->Start();
	ui->vtkWidget->GetRenderWindow()->Finalize();*/

	/*//ui->vtkWidget->update();
	//ui->vtkWidget->GetRenderWindow()->Render();
	//ui->vtkWidget->repaint();
	//ui->vtkWidget->GetRenderWindow()->Start();*/
		
	camera->Delete();
}

void MainWindow::rightView(){
	ren->RemoveAllViewProps(); //remove all actor in the actor list
	ren->ResetCamera();
	
	vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*2);
	camera->Azimuth(-90.0);	
	//camera->SetViewUp(0,1,0);	
	ren->SetActiveCamera(camera);	
	
	ren->AddActor(actor);	
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	
		
	camera->Delete();
}

void MainWindow::bottomView(){
	ren->RemoveAllViewProps(); //remove all actor in the actor list
	ren->ResetCamera();
	
	vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*2);
	camera->Azimuth(180.0);	
	//camera->SetViewUp(0,1,0);	
	ren->SetActiveCamera(camera);	
	
	ren->AddActor(actor);	
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	
		
	camera->Delete();
}

void MainWindow::leftView(){
	ren->RemoveAllViewProps(); //remove all actor in the actor list
	ren->ResetCamera();
	
	vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation
	camera->SetPosition(0,0,length*2);
	camera->Azimuth(90.0);		
	//camera->SetViewUp(0,1,0);	
	ren->SetActiveCamera(camera);	
	
	ren->AddActor(actor);	
	ren->ResetCameraClippingRange();
	ren->GetRenderWindow()->Render();	
		
	camera->Delete();
}


//CREATING TOOLBAR
void MainWindow::createToolBar(){
    toolBar=new QToolBar(this);
    toolBar->setGeometry(0,0,750,20);
    toolBar->setMovable(1);
    toolBar->setFloatable(1);

    openButton=new QToolButton(this);
    //openButton->setIcon(QIcon("F:\\MyQtApps\\QtDesktopApp\\DesktopApp\\resources\\openFile.png"));
	openButton->setIcon(QIcon(":/resources/openFile.PNG"));//NOTE .png and .PNG are different (case sensitive)
	openButton->setToolTip("Open ALICONA file");
    //openButton->setGeometry(0,0,10,20);
    connect(openButton, SIGNAL(clicked()),this,SLOT(toolButton1_clicked())); //this is the linking between SIGNAL and SLOT

    saveButton=new QToolButton(this);
    //saveButton->setIcon(QIcon("F:\\MyQtApps\\QtDesktopApp\\DesktopApp\\resources\\saveFile.png"));
	saveButton->setIcon(QIcon(":/resources/saveFile.PNG"));
	saveButton->setToolTip("Save output to file");
    //saveButton->setGeometry(0,0,10,20);
    connect(saveButton, SIGNAL(clicked()),this,SLOT(toolButton2_clicked())); //this is the linking between SIGNAL and SLOT

	//DATA EDITING
	deleteButton=new QToolButton(this);
	deleteButton->setIcon(QIcon(":/resources/delete_data.PNG"));
	deleteButton->setToolTip("Delete selected data");
    deleteButton->setGeometry(0,0,10,20);
    connect(deleteButton, SIGNAL(clicked()),this,SLOT(deleteButton_clicked())); //this is the linking between SIGNAL and SLOT

	cropButton=new QToolButton(this);
	cropButton->setIcon(QIcon(":/resources/croping_data.PNG"));
	cropButton->setToolTip("Croping selected data");
    cropButton->setGeometry(0,0,10,20);
    connect(cropButton, SIGNAL(clicked()),this,SLOT(cropButton_clicked())); //this is the linking between SIGNAL and SLOT

	filterButton=new QToolButton(this);
	filterButton->setIcon(QIcon(":/resources/filtering_data.PNG"));
	filterButton->setToolTip("Plane filtering outlier data");
    filterButton->setGeometry(0,0,10,20);
    connect(filterButton, SIGNAL(clicked()),this,SLOT(filterButton_clicked())); //this is the linking between SIGNAL and SLOT


	//LS FITTING
	lineFittingButton=new QToolButton(this);
    //lineFittingButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\lineFitting.png"));
	lineFittingButton->setIcon(QIcon(":/resources/lineFitting.PNG"));
	lineFittingButton->setToolTip("Line least-square fitting");
    //saveButton->setGeometry(0,0,10,20);
    connect(lineFittingButton, SIGNAL(clicked()),this,SLOT(lineFitting_clicked())); //this is the linking between SIGNAL and SLOT

	planeFittingButton=new QToolButton(this);
    //planeFittingButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\planeFitting.png"));
	planeFittingButton->setIcon(QIcon(":/resources/planeFitting.PNG"));
    planeFittingButton->setToolTip("Plane least-square fitting");
	//saveButton->setGeometry(0,0,10,20);
    connect(planeFittingButton, SIGNAL(clicked()),this,SLOT(planeFitting_clicked())); //this is the linking between SIGNAL and SLOT

	sphereFittingButton=new QToolButton(this);
    //sphereFittingButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\sphereFitting.png"));
	sphereFittingButton->setIcon(QIcon(":/resources/sphereFitting.PNG"));
    sphereFittingButton->setToolTip("Sphere least-square fitting");
	//saveButton->setGeometry(0,0,10,20);
    connect(sphereFittingButton, SIGNAL(clicked()),this,SLOT(sphereFitting_clicked())); //this is the linking between SIGNAL and SLOT

	cylinderFittingButton=new QToolButton(this);
    //cylinderFittingButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\cylinderFitting.png"));
	cylinderFittingButton->setIcon(QIcon(":/resources/cylinderFitting.PNG"));
    cylinderFittingButton->setToolTip("Cylinder least-square fitting");
	//saveButton->setGeometry(0,0,10,20);
    connect(cylinderFittingButton, SIGNAL(clicked()),this,SLOT(cylinderFitting_clicked())); //this is the linking between SIGNAL and SLOT

	//GEOMETRIC Verification
	straightnessButton=new QToolButton(this);
    //straightnessButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\straightness.png"));
	straightnessButton->setIcon(QIcon(":/resources/straightness.PNG"));
    straightnessButton->setToolTip("Straightness");
	//saveButton->setGeometry(0,0,10,20);
    connect(straightnessButton, SIGNAL(clicked()),this,SLOT(straightness_clicked())); //this is the linking between SIGNAL and SLOT

	flatnessButton=new QToolButton(this);
    //flatnessButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\flatness.png"));
	flatnessButton->setIcon(QIcon(":/resources/flatness.PNG"));
    flatnessButton->setToolTip("Flatness");
	//saveButton->setGeometry(0,0,10,20);
    connect(flatnessButton, SIGNAL(clicked()),this,SLOT(flatness_clicked())); //this is the linking between SIGNAL and SLOT

	sphereFormButton=new QToolButton(this);
    //sphereFormButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\sphereForm.png"));
	sphereFormButton->setIcon(QIcon(":/resources/sphereForm.PNG"));
    sphereFormButton->setToolTip("Sphere form");
	//saveButton->setGeometry(0,0,10,20);
    connect(sphereFormButton, SIGNAL(clicked()),this,SLOT(sphereForm_clicked())); //this is the linking between SIGNAL and SLOT

	cylindricityButton=new QToolButton(this);
    //cylindricityButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\cylindricity.png"));
	cylindricityButton->setIcon(QIcon(":/resources/cylindricity.PNG"));
    cylindricityButton->setToolTip("Cylindricity");
	//saveButton->setGeometry(0,0,10,20);
    connect(cylindricityButton, SIGNAL(clicked()),this,SLOT(cylindricity_clicked())); //this is the linking between SIGNAL and SLOT

	//AUTOMATIC Measurement
	holeDepthButton=new QToolButton(this);
    //holeDepthButton->setIcon(QIcon("E:\\My PhD\\VTK Programming\\VTK_Project\\Belajar\\DesktopApp\\resources\\hole_depth.png"));
	holeDepthButton->setIcon(QIcon(":/resources/hole_depth.PNG"));
    holeDepthButton->setToolTip("Automatic hole-depth measurement");
	//saveButton->setGeometry(0,0,10,20);
    connect(holeDepthButton, SIGNAL(clicked()),this,SLOT(holeDepth_clicked())); //this is the linking between SIGNAL and SLOT

	boxDepthButton=new QToolButton(this);
    boxDepthButton->setIcon(QIcon(":/resources/box_depth.PNG"));
    boxDepthButton->setToolTip("Automatic square-depth measurement");
	//saveButton->setGeometry(0,0,10,20);
    connect(boxDepthButton, SIGNAL(clicked()),this,SLOT(boxDepth_clicked())); //this is the linking between SIGNAL and SLOT

	//STL PROCESSING
	STLGenerationButton=new QToolButton(this);
    STLGenerationButton->setIcon(QIcon(":/resources/STLOpen.PNG"));
    STLGenerationButton->setToolTip("Open a STL file");
	//saveButton->setGeometry(0,0,10,20);
    connect(STLGenerationButton, SIGNAL(clicked()),this,SLOT(STLGeneration_clicked())); //this is the linking between SIGNAL and SLOT

	triangleMeshingButton=new QToolButton(this);
    triangleMeshingButton->setIcon(QIcon(":/resources/STLGeneration.PNG"));
    triangleMeshingButton->setToolTip("Point tringulation (Mesh)");
	//saveButton->setGeometry(0,0,10,20);
    connect(triangleMeshingButton, SIGNAL(clicked()),this,SLOT(triangleMeshing_clicked())); //this is the linking between SIGNAL and SLOT

	STLDeleteButton=new QToolButton(this);
    STLDeleteButton->setIcon(QIcon(":/resources/delete_data_stl.PNG"));
    STLDeleteButton->setToolTip("Delete selected STL data");
	//saveButton->setGeometry(0,0,10,20);
    connect(STLDeleteButton, SIGNAL(clicked()),this,SLOT(STLDelete_clicked())); //this is the linking between SIGNAL and SLOT

	STLCropButton=new QToolButton(this);
    STLCropButton->setIcon(QIcon(":/resources/croping_data_stl.PNG"));
    STLCropButton->setToolTip("Delete selected STL data");
	//saveButton->setGeometry(0,0,10,20);
    connect(STLCropButton, SIGNAL(clicked()),this,SLOT(STLCrop_clicked())); //this is the linking between SIGNAL and SLOT



	//SIMULATION ISO15530-4
	simulationButton=new QToolButton(this);    
	simulationButton->setIcon(QIcon(":/resources/simulation.PNG"));
    simulationButton->setToolTip("Uncertainty Estimation (ISO15530-4)");
	//saveButton->setGeometry(0,0,10,20);
    connect(simulationButton, SIGNAL(clicked()),this,SLOT(simulation_clicked())); //this is the linking between SIGNAL and SLOT


    toolBar->addWidget(openButton);    
    toolBar->addWidget(saveButton);
	toolBar->addSeparator();
	toolBar->addWidget(deleteButton);
	toolBar->addWidget(cropButton);
	toolBar->addWidget(filterButton);
	toolBar->addSeparator();
	toolBar->addWidget(lineFittingButton);
	toolBar->addWidget(planeFittingButton);
	toolBar->addWidget(sphereFittingButton);
	toolBar->addWidget(cylinderFittingButton);
	toolBar->addSeparator();
	toolBar->addWidget(straightnessButton);
	toolBar->addWidget(flatnessButton);
	toolBar->addWidget(sphereFormButton);
	toolBar->addWidget(cylindricityButton);
	toolBar->addSeparator();
	toolBar->addWidget(holeDepthButton);
	toolBar->addWidget(boxDepthButton);
	toolBar->addSeparator();
	toolBar->addWidget(STLGenerationButton);
	toolBar->addWidget(triangleMeshingButton);
	toolBar->addWidget(STLDeleteButton);
	toolBar->addWidget(STLCropButton);
	toolBar->addSeparator();
	toolBar->addWidget(simulationButton);

    ui->mainToolBar->addWidget(toolBar);
    ui->mainToolBar->setParent(this);
}

//SLOT TOOLBAR
void MainWindow::toolButton1_clicked(){   

	  //THIS IS TO READ POINTS CLOUDS
	  //Opening a file using QFileDialog
	  QFileDialog *fileDlg=new QFileDialog(this);
	  QString path= fileDlg->getOpenFileName();

	  //Verify if the path has value or not
	  if(path.isEmpty()){		  
		  return;
	  }
	  
	  //coverting QString to std::string (C++ style string)
	  std::string str=path.toStdString();
	  //Converting std::string to *char (C-style string)
	  char *fileName=new char[str.length()+1]; //+1 for space of carriage return	  
	  strcpy(fileName, str.c_str());

	  clearDataDisplay();
	  ui->lineEditFilePath->setText(path);		  
	  
	  //VISUALIZATION PIPELINE
		vtkPoints *points=vtkPoints::New(); //Point Data
		vtkCellArray *strips=vtkCellArray::New(); //Cell Data, these are the two VtkObject Basic Structure

		int counterPoint=0, counter=0;
		//long counterPoint=0, counter=0;
		double point[3];		
		char chr1, chr2, chr3,str1[20];		
	
		//Setting the color base on the Z value
		vtkDoubleArray* pointColor = vtkDoubleArray::New();
		pointColor->SetName("pointColor");
		
		//Reading the point cloud
		FILE* pFile;
		pFile=fopen(fileName,"r");
		assert(pFile!=NULL); //if pFile==NULL, stop the program
				
		//counterPoint format integer
		fscanf(pFile,"%s %c %d %c %c",str1,&chr1,&counterPoint, &chr2, &chr3);//scan the number of points
		//QMessageBox::information(this,tr("Message"),"OK counterPoint");

		//counterPoint format long 
		//fscanf(pFile,"%s %c %ld %c %c",str1,&chr1,&counterPoint, &chr2, &chr3);//scan the number of points
		
		/*//counterPoint format string
		char counterPoint_string[100];
		fscanf(pFile,"%s %c %s %c %c",str1,&chr1,&counterPoint_string, &chr2, &chr3);//scan the number of points
		QMessageBox::information(this,tr("Message"),"OK counterPoint");
		counterPoint=atoi(counterPoint_string);*/

		m.resize(counterPoint,3); //re-size the matric according to number of row (points)
		num_of_points=counterPoint;
		counter=0;
		if(num_of_points<100000){
			strips->InsertNextCell(100001);
		}
		else{
			strips->InsertNextCell(counterPoint+1);
		}
		resetMaxMinValue();		
		
		while(counter<num_of_points){
		//while(counter<counterPoint){
		//while(!feof(pFile)){
			fscanf(pFile,"%lf %lf %lf",&point[0],&point[1],&point[2]);	 //without normal vector: only XYZ coordinate
			//assert(point!=NULL);

			//Reduce the magnitude of the points
			/*if(fabs(point[0])>1){
				point[0]=point[0]/1e6;
				point[1]=point[1]/1e6;
				point[2]=point[2]/1e6;
			}*/

			points->InsertPoint(counter,point);
			strips->InsertCellPoint(counter);
			
			//Inserting the data into EIGEN Matrix
			m(counter,0)=point[0];
			m(counter,1)=point[1];
			m(counter,2)=point[2];

			//Determining Max and Min value for X, Y and Z
			setMaxMinValue(point[0],point[1],point[2]);

			counter++;		

			//set the color according to the z hight
			pointColor->InsertNextValue(point[2]*1000.0);		

			//calculating deltaX and deltaY: pixel distance
			if(counter==2){
				deltaX=fabs(m(1,0)-m(0,0));
				//deltaY=fabs(m(1,1)-m(0,1));
				deltaY=deltaX;
				if(deltaX==0){ //sometimes the data is vertically arranged in the 2D matrix
					deltaY=fabs(m(1,1)-m(0,1));
					deltaX=deltaY;
				}
			}
		}
		fclose(pFile);

		//TO SOLVE SMALL DATA PROBLEM, Filling the points with fake values upto 100000 points
		int iter;
		if(num_of_points<100000){
			for(iter=num_of_points;iter<=100000;iter++){
			//for(iter=counter;iter<=100000;iter++){
				points->InsertPoint(iter,point); //inserting the value of the last point
				strips->InsertCellPoint(iter);
				pointColor->InsertNextValue(point[2]*1000.0);
			}
		}

		calcBoundingBox();
		
		//GLOBAL PolyData and DataMapper
		pointCloud->SetPoints(points);		
		pointCloud->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
		//pointCloud->SetStrips(strips); //show as triangulation surface
		pointCloud->SetVerts(strips); //show as points		
		pointCloud->Update(); //We need to update because it will be rendered not now (later)--> VTK Lazy update
		pointCloudMapper->SetInput(pointCloud);
		pointCloudMapper->Update(); //We need to update because it will be rendered not now (later) --> VTK Lazy update
		
		actor->SetMapper(pointCloudMapper);
		actor->GetProperty()->SetColor(1,0,0);

		//Defining bounding box		
		vtkPolyDataNormals *normals=vtkPolyDataNormals::New(); //local
		normals->SetInput(pointCloud);				

		vtkOutlineFilter *outline=vtkOutlineFilter::New(); //local
		outline->SetInput(normals->GetOutput());
		
		vtkPolyDataMapper *outlineMapper=vtkPolyDataMapper::New(); //local
		outlineMapper->SetInput(outline->GetOutput());

		outlineActor->SetMapper(outlineMapper); //Global
		outlineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
		
		outlineMapper->Delete();
		outline->Delete();
		normals->Delete();
				
		//LOCAL PolyData and DataMapper
		vtkPolyData *ownData = vtkPolyData::New();
		ownData->SetPoints(points);
		ownData->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
		//ownData->SetStrips(strips); //show as triangulation surface
		ownData->SetVerts(strips); //show as points
		ownData->Update();
				
		vtkPolyDataMapper *ownDataMapper= vtkPolyDataMapper::New();
		ownDataMapper->SetInput(ownData);
		
		//GRAPHICS PIPELINE
		vtkActor *ownDataActor = vtkActor::New();
		ownDataActor->SetMapper(ownDataMapper);
		ownDataActor->GetProperty()->SetColor(1.0, 0.3882, 0.2784);//The color priority is the one set inside the dataset 
		if(num_of_points<50000){//increase the poins size to be able to see them
			ownDataActor->GetProperty()->SetPointSize(5.0);
		}
		//ownDataActor->SetPosition(0,0,0);		
		//cylinderActor->RotateX(30.0);
		//cylinderActor->RotateY(-45.0);
		
		//vtkCamera
		vtkCamera *camera=vtkCamera::New();
		camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation		
		camera->SetPosition(0,0,length*4);
		camera->Azimuth(-45.0);
		camera->Elevation(-45.0);
		ren->SetActiveCamera(camera);
		/*if(fabs(point[0])>1){//for data which the origin is very far from the points
			camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation						
			camera->Azimuth(-75.0);
			camera->Elevation(-75.0);								
		}*/
		//camera->SetFocalPoint(0,1,0);
		//camera->SetPosition(0,0,1);//Default value (0,0,1)
		//camera->SetViewUp(0,1,0);	//default 0,1,0	
		//camera->ComputeViewPlaneNormal();	
		//ren->SetActiveCamera(camera);	

		//showing axes
		vtkAxesActor *axes=vtkAxesActor::New(); //Defoult axes position is on the origin of the viewport
		//axes->SetNormalizedShaftLength(0.01,0.01,0.01);
		//axes->SetNormalizedTipLength(0.01,0.01,0.01);
		//axes->SetNormalizedLabelPosition(0.01,0.01,0.01);
		axes->SetTotalLength(0.005,0.005,0.005);
		//axes->SetTotalLength(width/5,width/5,width/5);
		//axes->AddPosition(-minX, -minY, -height/2);
				
		ren->SetNearClippingPlaneTolerance(0.000001);
		ren->ResetCameraClippingRange();
		ren->AddActor(ownDataActor);
		//QMessageBox::information(this,tr("Message"),"OKKKKKKKKKBOS"+QString::number(counterPoint));
		//ren->AddActor(axes);		
		ren->GetRenderWindow()->Render();		
	  
		ownDataMapper->Delete();
		ownDataActor->Delete();
		ownData->Delete();
		points->Delete();
		strips->Delete();
		pointColor->Delete();
		camera->Delete();
		axes->Delete(); 
		
		delete [] fileName;//delete the c-style string dynamic variable

		//Print output into textEdit
		ui->textEdit->setText("Measurement: \n\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Number of points = "+QString::number(num_of_points)+"\n\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. X = "+QString::number(maxX)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. X = "+QString::number(minX)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. Y = "+QString::number(maxY)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. Y = "+QString::number(minY)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. Z = "+QString::number(maxZ)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. Z = "+QString::number(minZ)+"\n\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Pixel distance X: "+QString::number(deltaX)+"\n");
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Pixel distance Y: "+QString::number(deltaY)+"\n\n");
		//calcBoundingBox();
		ui->textEdit->setText(ui->textEdit->toPlainText()+"Bounding Box (LxWxH)= "+QString::number(length)+" x "+QString::number(width)+" x "+QString::number(height)+"\n\n");
		
		//enable tool bar button after when a file is opened
		flag_file_is_opened=1;
		setComponentStatus();
	  
      //THIS IS TO SHOW AND UPDATE QVTKWidget
	  /*// Geometry
	  source = vtkCylinderSource::New();

	  // Mapper
	  mapper = vtkPolyDataMapper::New();
	  mapper->ImmediateModeRenderingOn();
	  //mapper->SetInputConnection(source->GetOutputPort());
	  mapper->SetInput(source->GetOutput());

	  // Actor in scene
	  actor = vtkActor::New();
	  actor->SetMapper(mapper);

	  // Add Actor to renderer
	  //ren = vtkRenderer::New();
	  ren->AddActor(actor);

	  // Reset camera
	  ren->SetBackground(1,1,1);
	  ren->ResetCamera();
	  ren->GetRenderWindow()->Render();*/


	  //THIS IS IF WE WANT TO SHOW THE DATA IN SEPERATE WINDOW
	  /*vtkRenderWindow *renWin=vtkRenderWindow::New();
	  renWin->AddRenderer(ren);
	  renWin->SetSize(500,500);

	  vtkRenderWindowInteractor *iren=vtkRenderWindowInteractor::New();
	  iren->SetRenderWindow(renWin);

	  vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
	  iren->SetInteractorStyle(style);

	  iren->Initialize();
	  iren->Start();*/
	
}

void MainWindow::toolButton2_clicked(){
    //QMessageBox::information(this,tr("Message"),tr("You pressed Save Button"));
	QFileDialog *fileDlg=new QFileDialog(this);
	QString path= fileDlg->getSaveFileName();

	//Verify if the path has value or not
	if(path.isEmpty()){
		  return;
	}

	QFile f(path);
	if(f.open(QIODevice::WriteOnly)){
	//writing the data
		QTextStream stream(&f);
		stream <<ui->textEdit->toPlainText()<<endl;
	}
	else{
		QMessageBox::information(this,tr("save file information"),tr("Filename should be defined!"));
	}
	f.close();
}

void MainWindow::deleteButton_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Delete Button"));

	//==========TESTING Delete (cutting) data - half from the original==============================

	vtkSmartPointer<vtkCubeSource> cubeSource=vtkSmartPointer<vtkCubeSource>::New();
	double centerX, centerY, centerZ;
	centerX=(maxX+minX)/2;
	centerY=(maxY+minY)/2;
	centerZ=(maxZ+minZ)/2;
	cubeSource->SetCenter(centerX, centerY, centerZ);
	cubeSource->SetCenter(maxX, centerY, centerZ);
	cubeSource->SetXLength((maxX-minX)/2);
	cubeSource->SetYLength(maxY-minY);
	cubeSource->SetZLength(maxZ-minZ);
	//cubeSource->SetXLength(((maxX-minX)/2)*0.1);
	//cubeSource->SetYLength((maxY-minY)*0.1);
	//cubeSource->SetZLength((maxZ-minZ)*0.1);
	cubeSource->Update();

	//cretae the function to clipp (crop the data). Creating implicit function: a box, the size is taken from the cubeSource bounding box
	vtkSmartPointer<vtkBox> implicitCube=vtkSmartPointer<vtkBox>::New(); //This is the implicit equation to clipp the data set (polyData)
	implicitCube->SetBounds(cubeSource->GetOutput()->GetBounds());
	//implicitCube->SetXMax(((maxX-minX)/2),(maxY-minY),(maxZ-minZ));

	//vtkIdFilter
	//generate scalars or field data from point and cell ids
    //vtkIdFilter is a filter to that generates scalars or field data using cell and point ids. 
	//That is, the point attribute data scalars or field data are generated from the point ids,
	//and the cell attribute data scalars or field data are generated from the the cell ids.
	//vtkSmartPointer<vtkIdFilter> idFilter=vtkSmartPointer<vtkIdFilter>::New();
	//idFilter->SetInputConnection(pointCloud);

	//vtkPolyDataNormal
	vtkSmartPointer<vtkPolyDataNormals> normals=vtkSmartPointer<vtkPolyDataNormals>::New(); //local
	normals->SetInput(pointCloud); 

	/*//Cutting the polydata
	vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
	cutter->SetCutFunction(implicitCube);
	cutter->SetInput(normals->GetOutput());
	cutter->Update();
	
	//vtkPolyData *STLPolyData=cutter->GetOutput();*/

	//Clipping the polydata
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetClipFunction(implicitCube);
	//clipper->SetInput(pointCloud);
	clipper->SetInput(normals->GetOutput());
	clipper->InsideOutOff(); //keep data outside the implicit function (selected region)
	clipper->Update();
	
	vtkPolyData *STLPolyData=clipper->GetOutput();

	//Insert the new clipped (cropped) point into the global matrix m
	int num_of_points_in_STL=0; //NOTE: vtkIdType = int
	num_of_points_in_STL=STLPolyData->GetNumberOfPoints();
	num_of_points=num_of_points_in_STL; //num_of_points is global variable
	m.resize(num_of_points_in_STL,3);
	resetMaxMinValue();		
	ui->textEdit->setText("Number of points = "+QString::number(STLPolyData->GetNumberOfPoints() )+"\n");
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		m(i,0)=point[0];
		m(i,1)=point[1];
		m(i,2)=point[2];		
		setMaxMinValue(point[0],point[1],point[2]);
	}	
	calcBoundingBox();		

	showPoints(m);	

	//===================================ENF OF TESTING================================================
}

void MainWindow::cropButton_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Cropping Button"));

	//==========TESTING cropping (clipping) data - half from the original==============================

	vtkSmartPointer<vtkCubeSource> cubeSource=vtkSmartPointer<vtkCubeSource>::New();
	double centerX, centerY, centerZ;
	centerX=(maxX+minX)/2;
	centerY=(maxY+minY)/2;
	centerZ=(maxZ+minZ)/2;
	cubeSource->SetCenter(centerX, centerY, centerZ);
	cubeSource->SetCenter(maxX, centerY, centerZ);
	cubeSource->SetXLength((maxX-minX)/2);
	cubeSource->SetYLength(maxY-minY);
	cubeSource->SetZLength(maxZ-minZ);
	//cubeSource->SetXLength(((maxX-minX)/2)*0.1);
	//cubeSource->SetYLength((maxY-minY)*0.1);
	//cubeSource->SetZLength((maxZ-minZ)*0.1);
	cubeSource->Update();

	//cretae the function to clipp (crop the data). Creating implicit function: a box, the size is taken from the cubeSource bounding box
	vtkSmartPointer<vtkBox> implicitCube=vtkSmartPointer<vtkBox>::New(); //This is the implicit equation to clipp the data set (polyData)
	implicitCube->SetBounds(cubeSource->GetOutput()->GetBounds());
	//implicitCube->SetXMax(((maxX-minX)/2),(maxY-minY),(maxZ-minZ));

	//vtkIdFilter
	//generate scalars or field data from point and cell ids
    //vtkIdFilter is a filter to that generates scalars or field data using cell and point ids. 
	//That is, the point attribute data scalars or field data are generated from the point ids,
	//and the cell attribute data scalars or field data are generated from the the cell ids.
	//vtkSmartPointer<vtkIdFilter> idFilter=vtkSmartPointer<vtkIdFilter>::New();
	//idFilter->SetInputConnection(pointCloud);

	//vtkPolyDataNormal
	vtkSmartPointer<vtkPolyDataNormals> normals=vtkSmartPointer<vtkPolyDataNormals>::New(); //local
	normals->SetInput(pointCloud); 

	//Clipping the polydata
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetClipFunction(implicitCube);
	//clipper->SetInput(pointCloud);
	clipper->SetInput(normals->GetOutput());
	clipper->InsideOutOn();
	clipper->Update();

	vtkPolyData *STLPolyData=clipper->GetOutput();

	//Insert the new clipped (cropped) point into the global matrix m
	int num_of_points_in_STL=0; //NOTE: vtkIdType = int
	num_of_points_in_STL=STLPolyData->GetNumberOfPoints();
	num_of_points=num_of_points_in_STL; //num_of_points is global variable
	m.resize(num_of_points_in_STL,3);
	resetMaxMinValue();		
	ui->textEdit->setText("Number of points = "+QString::number(STLPolyData->GetNumberOfPoints() )+"\n");
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		m(i,0)=point[0];
		m(i,1)=point[1];
		m(i,2)=point[2];		
		setMaxMinValue(point[0],point[1],point[2]);
	}	
	calcBoundingBox();		

	showPoints(m);	

	//===================================ENF OF TESTING================================================


}

void MainWindow::filterButton_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Plane Filtering Button"));

	MatrixXd m2;
	//QMessageBox::information(this,tr("Message"),"Initial number of points= " + QString::number(num_of_points));	
	myFunction function;
	m2=function.plane_filter_3_sigma_based(m);

	num_of_points=m2.rows();
	//QMessageBox::information(this,tr("Message"),"Final number of points= " + QString::number(num_of_points));	
	m.resize(num_of_points,3);
	for(int i=0;i<num_of_points;i++){
		m(i,0)=m2(i,0);
		m(i,1)=m2(i,1);
		m(i,2)=m2(i,2);
	}

	showPoints(m);
}

void MainWindow::lineFitting_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Line Fitting Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0; //point on line and cosine direction
	myFunction function;
	function.line_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	double p1x=0.0,p1y=0.0,p1z=0.0,p2x=0.0,p2y=0.0,p2z=0.0;
	double t=0.0; //parametric scale
	//defining point1
	p1x=minX;
	t=(p1x-px)/(n1);
	p1y=py+n2*t;
	p1z=pz+n3*t;
	//defining point2
	p2x=maxX;
	t=(p2x-px)/(n1);
	p2y=py+n2*t;
	p2z=pz+n3*t;

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Fitted Line Parameters:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"1. Point on line:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"PX = "+QString::number(px)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"PY = "+QString::number(py)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"PZ = "+QString::number(pz)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"2. Cosine direction:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"X-direction = "+QString::number(n1)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Y-direction = "+QString::number(n2)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Z-direction = "+QString::number(n3)+"\n\n");

	//Show data with VTK
	vtkLineSource *lineSource=vtkLineSource::New();
	lineSource->SetPoint1(p1x,p1y,p1z);
	lineSource->SetPoint2(p2x,p2y,p2z);
	//planeSource->SetXResolution(10);
	//planeSource->SetYResolution(10);
	lineSource->Update();  //VTK is lazy update

	//we can use vykTubeFilter to represent a line, since the tube size is vary when we zoom

	vtkPolyDataMapper *lineMapper=vtkPolyDataMapper::New();
	lineMapper->SetInput(lineSource->GetOutput());

	vtkActor *lineActor=vtkActor::New();
	lineActor->SetMapper(lineMapper);
	lineActor->GetProperty()->SetLineWidth(5);

	ren->AddActor(lineActor);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	lineSource->Delete();
	lineMapper->Delete();
	lineActor->Delete();
}

void MainWindow::planeFitting_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Plane Fitting Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0; //center point and normal vector

	myFunction function;
	function.plane_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Fitted Plane Parameters:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"1. Centroid:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CX = "+QString::number(px)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CY = "+QString::number(py)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CZ = "+QString::number(pz)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"2. Normal vector:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"X-direction = "+QString::number(n1)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Y-direction = "+QString::number(n2)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Z-direction = "+QString::number(n3)+"\n\n");

	//Show data with VTK
	vtkPlaneSource *planeSource=vtkPlaneSource::New();
	planeSource->SetCenter(px,py,pz);
	planeSource->SetNormal(n1,n2,n3);
	//Set the size of the plane
	/*if(fabs(m(0,0))<=1){
		planeSource->SetOrigin(px-(length/2), py-(width/2),pz);
		planeSource->SetPoint1(length-(length/2), py-(width/2),pz);
		planeSource->SetPoint2(px-(length/2), width-(width/2),pz);
		planeSource->SetXResolution(10);
		planeSource->SetYResolution(10);
	}
	else{
		//planeSource->SetOrigin((px-(length/2)), (py-(width/2)),pz);
		planeSource->SetOrigin(minX, minY,pz);
		planeSource->SetPoint1(minX,maxY,pz);
		planeSource->SetPoint2(maxX,minY,pz);	
		//this is how to adjust how many number of rectanngle
		planeSource->SetXResolution(10);
		planeSource->SetYResolution(10);
	}*/

	planeSource->SetOrigin(minX, minY,pz);
	planeSource->SetPoint1(minX,maxY,pz);
	planeSource->SetPoint2(maxX,minY,pz);	
	//this is how to adjust how many number of rectanngle
	planeSource->SetXResolution(10);
	planeSource->SetYResolution(10);
	planeSource->Update();  //VTK is lazy update

	//vtkPolyData* plane = planeSource->GetOutput();

	vtkPolyDataMapper *planeMapper=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	planeMapper->SetInput(planeSource->GetOutput());

	vtkActor *planeActor=vtkActor::New();
	planeActor->SetMapper(planeMapper);

	ren->AddActor(planeActor);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	planeSource->Delete();
	planeMapper->Delete();
	planeActor->Delete();
}

void MainWindow::sphereFitting_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Sphere Fitting Button"));
	double px=0.0,py=0.0,pz=0.0,r=0.0, ss_residual=0.0;

	myFunction function;
	function.sphere_fitting(m,&px,&py,&pz,&r,&ss_residual);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Fitted Sphere Parameters:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Sphere center and radius:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CX = "+QString::number(px)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CY = "+QString::number(py)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CZ = "+QString::number(pz)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Radius = "+QString::number(r)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"SS Residual = "+QString::number(ss_residual)+"\n\n");

	//Show data with VTK
	vtkSphereSource *sphereSource=vtkSphereSource::New();
	sphereSource->SetCenter(px,py,pz);
	sphereSource->SetRadius(r);	
	sphereSource->Update();  //VTK is lazy update

	//vtkPolyData* plane = planeSource->GetOutput();

	vtkPolyDataMapper *sphereMapper=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	sphereMapper->SetInput(sphereSource->GetOutput());

	vtkActor *sphereActor=vtkActor::New();
	sphereActor->SetMapper(sphereMapper);

	ren->AddActor(sphereActor);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	sphereSource->Delete();
	sphereMapper->Delete();
	sphereActor->Delete();

}

void MainWindow::cylinderFitting_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Cylinder Fitting Button"));
	//QMessageBox::information(this,tr("Message"),tr("You pressed Sphere Fitting Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0,r=0.0, ss_residual=0.0;

	myFunction function;
	function.cylinder_fitting(m,&px,&py,&pz,&n1,&n2,&n3,&r,&ss_residual);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Fitted Cylinder Parameters:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"1. Cylinder point on axis and radius:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CX = "+QString::number(px)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CY = "+QString::number(py)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"CZ = "+QString::number(pz)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Radius = "+QString::number(r)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"2. Cylinder cosine direction:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"X-dir = "+QString::number(n1)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Y-dir = "+QString::number(n2)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Z-dir = "+QString::number(n3)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"SS Residual = "+QString::number(ss_residual)+"\n\n");

	//Show data with VTK
	vtkCylinderSource *cylinderSource=vtkCylinderSource::New();
	cylinderSource->SetCenter(px,py,pz);
	cylinderSource->SetRadius(r);	
	cylinderSource->SetHeight(length); //maxX-minX
	cylinderSource->Update();  //VTK is lazy update

	//vtkPolyData* plane = planeSource->GetOutput();

	vtkPolyDataMapper *cylinderMapper=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	cylinderMapper->SetInput(cylinderSource->GetOutput());

	vtkActor *cylinderActor=vtkActor::New();
	cylinderActor->SetMapper(cylinderMapper);

	ren->AddActor(cylinderActor);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	cylinderSource->Delete();
	cylinderMapper->Delete();
	cylinderActor->Delete();
}

void MainWindow::straightness_clicked(){ 
	//QMessageBox::information(this,tr("Message"),tr("You pressed Straightness Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;

	double straightness=0.0;
	myFunction function;
	straightness=function.straightness(m,&px,&py,&pz,&n1,&n2,&n3);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Straightness = "+QString::number(straightness)+"\n");

}

void MainWindow::flatness_clicked(){ 
	//QMessageBox::information(this,tr("Message"),tr("You pressed flatness Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;

	double flatness=0.0;
	myFunction function;
	flatness=function.flatness(m,&px,&py,&pz,&n1,&n2,&n3);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Flatness = "+QString::number(flatness)+"\n");
}

void MainWindow::sphereForm_clicked(){ 
	//QMessageBox::information(this,tr("Message"),tr("You pressed sphereForm Button"));
	double px=0.0,py=0.0,pz=0.0,r=0.0;

	double sphereForm=0.0;
	myFunction function;
	sphereForm=function.sphereForm(m,&px,&py,&pz,&r);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Sphere Form error= "+QString::number(sphereForm)+"\n");
}

void MainWindow::cylindricity_clicked(){ 
	//QMessageBox::information(this,tr("Message"),tr("You pressed cylindricity Button"));
	double px=0.0,py=0.0,pz=0.0,n1=0.0, n2=0.0, n3=0.0, r=0.0;

	double cylindricity=0.0;
	myFunction function;
	cylindricity=function.cylindricity(m,&px,&py,&pz,&n1,&n2,&n3,&r);

	//Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"cylindricity = "+QString::number(cylindricity)+"\n");
}
void MainWindow::holeDepth_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Holde Depth Button"));

	//1. Measuring algorithm
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;

	//Initial fitting of the plane considering the whole points
	myFunction function;
	function.plane_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	//Filtering the points based on 3-sigma rule
	double num_of_filtered_points=0.0;
	MatrixXd m2(num_of_points,3);
	m2=function.plane_filter_3_sigma_based(m);//error, can not return matrix
	//m2=function.plane_filter_3_sigma_based(m,px,py,pz,n1,n2,n3,num_of_points);	
	//function.plane_filter_3_sigma_based(m,m2,&num_of_filtered_points); //should be call by reference
	//ui->textEdit->setText(ui->textEdit->toPlainText()+"m2 points = "+QString::number(m2.rows())+"\n\n");
	//ui->textEdit->setText(ui->textEdit->toPlainText()+"filtered points = "+QString::number(num_of_filtered_points)+"\n\n");

	//refitting the plane from filtered points (m2)
	function.plane_fitting(m2,&px,&py,&pz,&n1,&n2,&n3);

	//finding a point which has the most distance to the re-fitted plane
	double px_far=0.0,py_far=0.0,pz_far=0.0;
	function.find_farthest_point(m2,m,&px_far, &py_far, &pz_far);

	//Hole depth calculation
	double hole_depth=0.0;
	hole_depth=function.dist_3D_point_to_plane(px_far,py_far,pz_far,px,py,pz,n1,n2,n3); 

	//2. Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"HOLE DEPTH Measurement Result:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Hole depth = "+QString::number(hole_depth)+"\n\n");	

	//3. Show data with VTK
	//fitted plane
	vtkPlaneSource *planeSource=vtkPlaneSource::New();
	planeSource->SetCenter(px,py,pz);
	planeSource->SetNormal(n1,n2,n3);
	//Set the size of the plane
	planeSource->SetOrigin(minX, minY,pz);
	planeSource->SetPoint1(minX,maxY,pz);
	planeSource->SetPoint2(maxX,minY,pz);	
	//this is how to adjust how many number of rectanngle
	planeSource->SetXResolution(10);
	planeSource->SetYResolution(10);
	planeSource->Update();  //VTK is lazy update

	vtkPolyDataMapper *planeMapper=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	planeMapper->SetInput(planeSource->GetOutput());

	vtkActor *planeActor=vtkActor::New();
	planeActor->SetMapper(planeMapper);

	//palne on farthest point
	vtkPlaneSource *planeSource2=vtkPlaneSource::New();
	planeSource2->SetCenter(px_far,py_far,pz_far);
	planeSource2->SetNormal(n1,n2,n3);
	//Set the size of the plane
	planeSource2->SetOrigin(minX, minY,pz_far);
	planeSource2->SetPoint1(minX,maxY,pz_far);
	planeSource2->SetPoint2(maxX,minY,pz_far);	
	//this is how to adjust how many number of rectanngle
	planeSource2->SetXResolution(10);
	planeSource2->SetYResolution(10);
	planeSource2->Update();  //VTK is lazy update

	vtkPolyDataMapper *planeMapper2=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	planeMapper2->SetInput(planeSource2->GetOutput());

	vtkActor *planeActor2=vtkActor::New();
	planeActor2->SetMapper(planeMapper2);

	ren->AddActor(planeActor);
	ren->AddActor(planeActor2);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	planeSource->Delete();
	planeMapper->Delete();
	planeActor->Delete();
	planeSource2->Delete();
	planeMapper2->Delete();
	planeActor2->Delete();
	
}

void MainWindow::boxDepth_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Hole Depth Button"));

	//1. Measuring algorithm
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;

	//Initial fitting of the plane considering the whole points
	myFunction function;
	//function.plane_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	//Filtering the points based on 3-sigma rule
	double num_of_filtered_points=0.0;
	MatrixXd m2(num_of_points,3);
	m2=function.plane_filter_3_sigma_based(m); //m2 is the filtered points

	//refitting the plane from filtered points (m2)
	function.plane_fitting(m2,&px,&py,&pz,&n1,&n2,&n3); //plane robust parameters (TOP Plane)

	//finding points on the bottom of the robust fitted plane
	MatrixXd m_bottom_plane;
	m_bottom_plane=function.find_points_on_bottom_plane(m, px, py, pz, n1, n2, n3);
	//function.find_farthest_point(m2,m,&px_bottom_plane, &py_bottom_plane, &pz_bottom_plane);

	//finding Centroid of the bottom plane
	double px_bottom_plane=0.0,py_bottom_plane=0.0,pz_bottom_plane=0.0;
	function.calculate_centroid(m_bottom_plane, &px_bottom_plane, &py_bottom_plane, &pz_bottom_plane);

	//Hole depth calculation
	double box_depth=0.0;
	box_depth=function.dist_3D_point_to_plane(px_bottom_plane,py_bottom_plane,pz_bottom_plane,px,py,pz,n1,n2,n3); 

	//2. Show the output to the text edit
	ui->textEdit->setText(ui->textEdit->toPlainText()+"BOX DEPTH Measurement Result:\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"BOX depth = "+QString::number(box_depth)+"\n\n");	

	//3. Show data with VTK
	//fitted plane
	vtkPlaneSource *planeSource=vtkPlaneSource::New();
	planeSource->SetCenter(px,py,pz);
	planeSource->SetNormal(n1,n2,n3);
	//Set the size of the plane
	planeSource->SetOrigin(minX, minY,pz);
	planeSource->SetPoint1(minX,maxY,pz);
	planeSource->SetPoint2(maxX,minY,pz);	
	//this is how to adjust how many number of rectanngle
	planeSource->SetXResolution(10);
	planeSource->SetYResolution(10);
	planeSource->Update();  //VTK is lazy update

	vtkPolyDataMapper *planeMapper=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	planeMapper->SetInput(planeSource->GetOutput());

	vtkActor *planeActor=vtkActor::New();
	planeActor->SetMapper(planeMapper);

	//Plane Bottom
	vtkPlaneSource *planeSource2=vtkPlaneSource::New();
	planeSource2->SetCenter(px_bottom_plane,py_bottom_plane,pz_bottom_plane);
	planeSource2->SetNormal(n1,n2,n3);
	//Set the size of the plane
	planeSource2->SetOrigin(minX, minY,pz_bottom_plane);
	planeSource2->SetPoint1(minX,maxY,pz_bottom_plane);
	planeSource2->SetPoint2(maxX,minY,pz_bottom_plane);	
	//this is how to adjust how many number of rectanngle
	planeSource2->SetXResolution(10);
	planeSource2->SetYResolution(10);
	planeSource2->Update();  //VTK is lazy update

	vtkPolyDataMapper *planeMapper2=vtkPolyDataMapper::New();
	//planeMapper->SetInputData(plane);
	planeMapper2->SetInput(planeSource2->GetOutput());

	vtkActor *planeActor2=vtkActor::New();
	planeActor2->SetMapper(planeMapper2);

	ren->AddActor(planeActor);
	ren->AddActor(planeActor2);
	ren->GetRenderWindow()->Render();

	//delete vtk dynamic object
	planeSource->Delete();
	planeMapper->Delete();
	planeActor->Delete();
	planeSource2->Delete();
	planeMapper2->Delete();
	planeActor2->Delete();

}

void MainWindow::STLGeneration_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed STL Generation Button"));

	//Opening a file using QFileDialog
	QFileDialog *fileDlg=new QFileDialog(this);
	QString path= fileDlg->getOpenFileName();

	//Verify if the path has value
	if(path.isEmpty()){		  
	  return;
	}

	//Initializing others object and Variables
	resetMaxMinValue();
	length=0.0;
	width=0.0;
	height=0.0;
	deltaX=0.0;
	deltaY=0.0;
	num_of_points=0;

	ren->RemoveAllViewProps(); //remove all actor in the actor list
	ren->ResetCamera();	
	 
	//coverting QString to std::string (C++ style string)
	std::string str=path.toStdString();
	//Converting std::string to *char (C-style string)
	char *fileName=new char[str.length()+1]; //+1 for space of carriage return	  
	strcpy(fileName, str.c_str());

	//Visualization Pipeline
	vtkSTLReader *STLModel= vtkSTLReader::New(); //Reader Source Object
	STLModel->SetFileName(fileName);
	STLModel->Update();

	//vtkPolyData *STLPolyData=vtkPolyData::New(); This should be before vtkSTLReader *STLModel= vtkSTLReader::New();!!!
	//STLPolyData=STLModel->GetOutput();
	vtkPolyData *STLPolyData=STLModel->GetOutput(); //only pointiting to the vtkPolyData object withour assigning memmory

	int num_of_points_in_STL=0; //NOTE: vtkIdType = int
	num_of_points_in_STL=STLPolyData->GetNumberOfPoints();
	num_of_points=num_of_points_in_STL;
	m.resize(num_of_points_in_STL,3);
	//double point[3];
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		m(i,0)=point[0];
		m(i,1)=point[1];
		m(i,2)=point[2];		
		setMaxMinValue(point[0],point[1],point[2]);
	}	
	calcBoundingBox();	

	//Filling the mfaces (the triangle): The cells
	num_of_cells=STLPolyData->GetNumberOfCells();
	mCells.resize(num_of_cells,3);
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfCells(); i++){
		vtkSmartPointer<vtkIdList> cellPointIds =vtkSmartPointer<vtkIdList>::New();
		STLPolyData->GetCellPoints(i,cellPointIds);		
		vtkIdType *cellId;
		cellId=cellPointIds->GetPointer(0);//we should start from the 0 position
		mCells(i,0)=*(cellId);//this is to access the element of the vtkIdList pointed bu cellId
		mCells(i,1)=*(cellId+1);
		mCells(i,2)=*(cellId+2);
	}

	//==============Filling Global vtkPolyData STLData: The points and cells=====================
	STLData->Reset();
	
	vtkSmartPointer<vtkPoints> points=vtkSmartPointer<vtkPoints>::New(); //Point Data
	vtkSmartPointer<vtkCellArray> cells=vtkSmartPointer<vtkCellArray>::New(); //Cell Data, these are the two Data set geometry
	
	//1. inserting point
	double point[3];
	for(int i=0;i<STLPolyData->GetNumberOfPoints();i++){		
		point[0]=m(i,0);
		point[1]=m(i,1);
		point[2]=m(i,2);
		points->InsertPoint(i,point);			
	}

	//2. inserting cells
	for(int i=0;i<STLPolyData->GetNumberOfCells();i++){		
		vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();			
		triangle->GetPointIds()->SetId ( 0, mCells(i,0));
		triangle->GetPointIds()->SetId ( 1, mCells(i,1));
		triangle->GetPointIds()->SetId ( 2, mCells(i,2) );
		cells->InsertNextCell ( triangle );
	}

	STLData->SetPoints (points);
	STLData->SetPolys (cells);

	STLData->Update();

	/*//========================== TESTING: Getting POINT and CELL ============================================
	//variables definition: NOTE vtkIdType = int
	int num_points=0;
	int num_cells=0;
	MatrixXd mPoints;
	MatrixXi mCells;

	num_points=STLPolyData->GetNumberOfPoints();
	num_cells=STLPolyData->GetNumberOfCells();
	mPoints.resize(num_points,3);
	mCells.resize(num_cells,3);
	
	//Filling the points
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		mPoints(i,0)=point[0];
		mPoints(i,1)=point[1];
		mPoints(i,2)=point[2];				
	}
	
	//Filling the cells
	//vtkSmartPointer<vtkIdList> cellPointIds =vtkSmartPointer<vtkIdList>::New();
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfCells(); i++){
		//cellPointIds->Reset();
		vtkSmartPointer<vtkIdList> cellPointIds =vtkSmartPointer<vtkIdList>::New();
		STLPolyData->GetCellPoints(i,cellPointIds);		
		vtkIdType *cellId;
		cellId=cellPointIds->GetPointer(0);//we should start from the 0 position
		mCells(i,0)=*(cellId);//this is to access the element of the vtkIdList pointed bu cellId
		mCells(i,1)=*(cellId+1);
		mCells(i,2)=*(cellId+2);

		//vtkIdType *cell_id; //THIS IS NOT WORKING, MAYBE BECAUSE "BUILTCELL()" functon is not carried out
		//vtkIdType ntupple=3;
		//STLPolyData->GetCellPoints(i,ntupple,cell_id);
		//mCells(i,0)=cells[0];
		//mCells(i,1)=cells[1];
		//mCells(i,2)=cells[2];		

		//mCells(i,0)=*cell_id;
		//mCells(i,1)=*(cell_id+1);
		//mCells(i,2)=*(cell_id+2);			
	}

	FILE* pFile2;
	char fileName2[100];

	sprintf(fileName2,"C:\\Documents and Settings\\Wahyudin\\Desktop\\AALICONA\\Point_and_Cell.txt");
	//strcat(fileName2,"-FILTERED.txt");
	//puts(fileName2);	
	pFile2=fopen(fileName2,"w");

	//Writing the points
	fprintf(pFile2,"Points: %d\n", num_points);
	for(int i=0;i<num_points;i++){	
		fprintf(pFile2,"%.9f %.9f %.9f\n",(float) mPoints(i,0), (float) mPoints(i,1), (float) mPoints(i,2));
	}  

	//writing the cells
	fprintf(pFile2,"Cells: %d \n", num_cells);
	for(int i=0;i<num_cells;i++){	
		fprintf(pFile2,"%.9f %.9f %.9f\n",(float) mCells(i,0), (float) mCells(i,1), (float) mCells(i,2));
	}  

	fclose(pFile2);
	//==================================FINISH TESTING========================================================*/

	//MAPPING
	vtkPolyDataMapper *STLModelMapper= vtkPolyDataMapper::New();
	STLModelMapper->SetInput(STLModel->GetOutput());
	STLModel->Delete();	
	//STLModelMapper->SetInput(STLModel->GetOutputPort());

	//Graphics Pipeline
	vtkActor *STLModelActor = vtkActor::New();
	STLModelActor->SetMapper(STLModelMapper);
	STLModelMapper->Delete();
	STLModelActor->GetProperty()->SetColor(0.5, 0.5, 0.5);
	//STLModelActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	//assign the STLModelMapper to actor: global vtkActor
	actor->SetMapper(STLModelMapper);
	
	//ren->RemoveAllViewProps(); //remove all actor in the actor list
	//ren->ResetCamera();	

	vtkCamera *camera=vtkCamera::New();
		camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation		
		camera->SetPosition(0,0,length*4);
		camera->Azimuth(-45.0);
		camera->Elevation(-45.0);
	
	ren->SetActiveCamera(camera);
	camera->Delete();
	ren->AddActor(STLModelActor);
	STLModelActor->Delete();
	ren->ResetCameraClippingRange(); //to make sure that our data inside the clipping range, otherwise it is not showing
	ren->GetRenderWindow()->Render();	
	//QMessageBox::information(this,tr("Message"),"OK 1");
	//STLPolyData->Delete();

	//Defining bounding box		
	vtkPolyDataNormals *normals=vtkPolyDataNormals::New(); //local
	normals->SetInput(STLPolyData);				

	vtkOutlineFilter *outline=vtkOutlineFilter::New(); //local
	outline->SetInput(normals->GetOutput());
		
	vtkPolyDataMapper *outlineMapper=vtkPolyDataMapper::New(); //local
	outlineMapper->SetInput(outline->GetOutput());

	outlineActor->SetMapper(outlineMapper); //Global
	outlineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
		
	outlineMapper->Delete();
	outline->Delete();
	normals->Delete();

	//SHOWING to TextEdit
	ui->textEdit->setText("Measurement: \n\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Number of points = "+QString::number(num_of_points)+"\n\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. X = "+QString::number(maxX)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. X = "+QString::number(minX)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. Y = "+QString::number(maxY)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. Y = "+QString::number(minY)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Max. Z = "+QString::number(maxZ)+"\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Min. Z = "+QString::number(minZ)+"\n\n");
	//ui->textEdit->setText(ui->textEdit->toPlainText()+"Pixel distance X: "+QString::number(deltaX)+"\n");
	//ui->textEdit->setText(ui->textEdit->toPlainText()+"Pixel distance Y: "+QString::number(deltaY)+"\n\n");
	ui->textEdit->setText(ui->textEdit->toPlainText()+"Bounding Box (LxWxH)= "+QString::number(length)+" x "+QString::number(width)+" x "+QString::number(height)+"\n\n");	

}

void MainWindow::triangleMeshing_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed triangle meshing Button"));
	
	/*//TESTING SHOW SMALL NUMBER OF POINTS independently, still it show error. Then
	//the problem of having points <100000 still occurs.
    //=================================================================================

	int i,j,k;
	float point[10][3];
	srand(time(NULL)); 
	for(i=0;i<10;i++){//set the coordinate
		point[i][0]=(float)(rand()%10)/10;//i%10;
		point[i][1]=(float)(rand()%10)/10;//i%10;
		point[i][2]=(float)(rand()%10)/10;
	}

	vtkPoints *points_1=vtkPoints::New();
	vtkCellArray *strips_1=vtkCellArray::New();
	strips_1->InsertNextCell(10);
	for(i=0;i<10;i++){
		points_1->InsertPoint(i,point[i]);
		strips_1->InsertCellPoint(i);
	}

	vtkDoubleArray* pointColor_1 = vtkDoubleArray::New();
	pointColor_1->SetName("pointColor");
	pointColor_1->InsertNextValue(2.7);
	pointColor_1->InsertNextValue(4.1);
	pointColor_1->InsertNextValue(5.3);
	pointColor_1->InsertNextValue(3.4);

	vtkPolyData *ownData_1 = vtkPolyData::New();
	ownData_1->SetPoints(points_1);
	ownData_1->GetPointData()->SetScalars(pointColor_1);
	ownData_1->SetVerts(strips_1); //show as points
	
	vtkPolyDataMapper *ownDataMapper_1= vtkPolyDataMapper::New();
	ownDataMapper_1->SetInput(ownData_1);

	//Graphics Pipeline
	vtkActor *ownDataActor_1 = vtkActor::New();
	ownDataActor_1->SetMapper(ownDataMapper_1);
	ownDataActor_1->GetProperty()->SetColor(1.0, 0.3882, 0.2784);
	//cylinderActor->RotateX(30.0);
	//cylinderActor->RotateY(-45.0);

	vtkRenderer *ren1_1=vtkRenderer::New();
	ren1_1->AddActor(ownDataActor_1);
	ren1_1->SetBackground(1.0,1.0,0.9);
	ren1_1->GetRenderWindow()->Render();
	//memory free
	
	ownDataMapper_1->Delete();
	ownDataActor_1->Delete();
	ren1_1->Delete();
	ownData_1->Delete();
	points_1->Delete();
	strips_1->Delete();
	pointColor_1->Delete(); */

	vtkPoints *points=vtkPoints::New();
	int i;
	double point[3];
	for(i=0;i<=num_of_points;i++){			
		point[0]=m(i,0);
		point[1]=m(i,1);
		point[2]=m(i,2);
		points->InsertPoint(i,point); //inserting the value of the last point				
	}

	vtkPolyData *meshPoints=vtkPolyData::New();
	meshPoints->SetPoints(points);
	points->Delete();	

	vtkDelaunay2D *STLMesh=vtkDelaunay2D::New();
	STLMesh->SetInput(meshPoints);
	STLMesh->SetSource(meshPoints); //This method use Constrained vtkDelaynay2D method
	//STLMesh->SetTolerance(0.0001);
	meshPoints->Delete();	

	vtkPolyDataMapper *STLMeshMapper=vtkPolyDataMapper::New();
	STLMeshMapper->SetInput(STLMesh->GetOutput());
	STLMesh->Delete();	

	vtkActor *STLMeshActor=vtkActor::New();
	STLMeshActor->SetMapper(STLMeshMapper);
	STLMeshMapper->Delete();
	

	//Clear the view inside the renderer
	ren->RemoveAllViewProps();
	//Re-show the original points inside matrix m (global matrix)
	

	//Camera view setting
	vtkCamera *camera;//=vtkCamera::New();
	camera=ren->GetActiveCamera();
	//camera->Delete();
	
	ren->ResetCamera();
	ren->SetActiveCamera(camera);		
	ren->ResetCameraClippingRange();	

	ren->AddActor(STLMeshActor);
	STLMeshActor->Delete();

	ren->GetRenderWindow()->Render();	
	
}

void MainWindow::STLDelete_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed STL Delete Button"));
	//==========TESTING Delete (cutting) data - half from the original==============================

	vtkSmartPointer<vtkCubeSource> cubeSource=vtkSmartPointer<vtkCubeSource>::New();
	double centerX, centerY, centerZ;
	centerX=(maxX+minX)/2;
	centerY=(maxY+minY)/2;
	centerZ=(maxZ+minZ)/2;
	cubeSource->SetCenter(centerX, centerY, centerZ);
	cubeSource->SetCenter(maxX, centerY, centerZ);
	cubeSource->SetXLength((maxX-minX)/2);
	cubeSource->SetYLength(maxY-minY);
	cubeSource->SetZLength(maxZ-minZ);
	cubeSource->Update();

	//cretae the function to clipp (crop the data). Creating implicit function: a box, the size is taken from the cubeSource bounding box
	vtkSmartPointer<vtkBox> implicitCube=vtkSmartPointer<vtkBox>::New(); //This is the implicit equation to clipp the data set (polyData)
	implicitCube->SetBounds(cubeSource->GetOutput()->GetBounds());
	
	//vtkIdFilter
	//generate scalars or field data from point and cell ids
    //vtkIdFilter is a filter to that generates scalars or field data using cell and point ids. 
	//That is, the point attribute data scalars or field data are generated from the point ids,
	//and the cell attribute data scalars or field data are generated from the the cell ids.
	//vtkSmartPointer<vtkIdFilter> idFilter=vtkSmartPointer<vtkIdFilter>::New();
	//idFilter->SetInputConnection(pointCloud);

	//vtkPolyDataNormal
	vtkSmartPointer<vtkPolyDataNormals> normals=vtkSmartPointer<vtkPolyDataNormals>::New(); //local
	normals->SetInput(STLData); 

	//Clipping the polydata
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetClipFunction(implicitCube);
	//clipper->SetInput(pointCloud);
	clipper->SetInput(normals->GetOutput());
	clipper->InsideOutOff(); //keep data outside the implicit function (selected region)
	clipper->Update();
	
	vtkPolyData *STLPolyData=clipper->GetOutput();

	//Insert the new clipped (cropped) point
	//1. inserting new points into the global matrix m
	int num_of_points_in_STL=0; //NOTE: vtkIdType = int
	num_of_points_in_STL=STLPolyData->GetNumberOfPoints();
	num_of_points=num_of_points_in_STL; //num_of_points is global variable
	m.resize(num_of_points_in_STL,3);
	resetMaxMinValue();		
	ui->textEdit->setText("Number of points = "+QString::number(STLPolyData->GetNumberOfPoints() )+"\n");
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		m(i,0)=point[0];
		m(i,1)=point[1];
		m(i,2)=point[2];		
		setMaxMinValue(point[0],point[1],point[2]);
	}	
	calcBoundingBox();	

	//2. inserting new cells into the global matrix mFaces
	num_of_cells=STLPolyData->GetNumberOfCells();
	mCells.resize(num_of_cells,3);
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfCells(); i++){
		vtkSmartPointer<vtkIdList> cellPointIds =vtkSmartPointer<vtkIdList>::New();
		STLPolyData->GetCellPoints(i,cellPointIds);		
		vtkIdType *cellId;
		cellId=cellPointIds->GetPointer(0);//we should start from the 0 position
		mCells(i,0)=*(cellId);//this is to access the element of the vtkIdList pointed bu cellId
		mCells(i,1)=*(cellId+1);
		mCells(i,2)=*(cellId+2);
	}

	showSTL(m,mCells);	

}

void MainWindow::STLCrop_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed STL Crop Button"));
	//==========TESTING Cropping (clipping) data - half from the original==============================

	vtkSmartPointer<vtkCubeSource> cubeSource=vtkSmartPointer<vtkCubeSource>::New();
	double centerX, centerY, centerZ;
	centerX=(maxX+minX)/2;
	centerY=(maxY+minY)/2;
	centerZ=(maxZ+minZ)/2;
	cubeSource->SetCenter(centerX, centerY, centerZ);
	cubeSource->SetCenter(maxX, centerY, centerZ);
	cubeSource->SetXLength((maxX-minX)/2);
	cubeSource->SetYLength(maxY-minY);
	cubeSource->SetZLength(maxZ-minZ);
	cubeSource->Update();

	//cretae the function to clipp (crop the data). Creating implicit function: a box, the size is taken from the cubeSource bounding box
	vtkSmartPointer<vtkBox> implicitCube=vtkSmartPointer<vtkBox>::New(); //This is the implicit equation to clipp the data set (polyData)
	implicitCube->SetBounds(cubeSource->GetOutput()->GetBounds());
	
	//vtkIdFilter
	//generate scalars or field data from point and cell ids
    //vtkIdFilter is a filter to that generates scalars or field data using cell and point ids. 
	//That is, the point attribute data scalars or field data are generated from the point ids,
	//and the cell attribute data scalars or field data are generated from the the cell ids.
	//vtkSmartPointer<vtkIdFilter> idFilter=vtkSmartPointer<vtkIdFilter>::New();
	//idFilter->SetInputConnection(pointCloud);

	//vtkPolyDataNormal
	vtkSmartPointer<vtkPolyDataNormals> normals=vtkSmartPointer<vtkPolyDataNormals>::New(); //local
	normals->SetInput(STLData); 

	//Clipping the polydata
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetClipFunction(implicitCube);
	//clipper->SetInput(pointCloud);
	clipper->SetInput(normals->GetOutput());
	clipper->InsideOutOn(); //keep data outside the implicit function (selected region)
	clipper->Update();
	
	vtkPolyData *STLPolyData=clipper->GetOutput();

	//Insert the new clipped (cropped) point
	//1. inserting new points into the global matrix m
	int num_of_points_in_STL=0; //NOTE: vtkIdType = int
	num_of_points_in_STL=STLPolyData->GetNumberOfPoints();
	num_of_points=num_of_points_in_STL; //num_of_points is global variable
	m.resize(num_of_points_in_STL,3);
	resetMaxMinValue();		
	ui->textEdit->setText("Number of points = "+QString::number(STLPolyData->GetNumberOfPoints() )+"\n");
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfPoints() ; i++){
		double point[3];
		STLPolyData->GetPoint(i,point);
		m(i,0)=point[0];
		m(i,1)=point[1];
		m(i,2)=point[2];		
		setMaxMinValue(point[0],point[1],point[2]);
	}	
	calcBoundingBox();	

	//2. inserting new cells into the global matrix mFaces
	num_of_cells=STLPolyData->GetNumberOfCells();
	mCells.resize(num_of_cells,3);
	for(vtkIdType i=0; i<STLPolyData->GetNumberOfCells(); i++){
		vtkSmartPointer<vtkIdList> cellPointIds =vtkSmartPointer<vtkIdList>::New();
		STLPolyData->GetCellPoints(i,cellPointIds);		
		vtkIdType *cellId;
		cellId=cellPointIds->GetPointer(0);//we should start from the 0 position
		mCells(i,0)=*(cellId);//this is to access the element of the vtkIdList pointed bu cellId
		mCells(i,1)=*(cellId+1);
		mCells(i,2)=*(cellId+2);
	}

	showSTL(m,mCells);	

}


void MainWindow::simulation_clicked(){
	//QMessageBox::information(this,tr("Message"),tr("You pressed Simulation Button"));

	//Show other window (widget): simulationWindow.

	//1. Modal method: user can not access the parent when the window is showing.
	/*simulationWindow simWin;
	simWin.setWindowModality(Qt::WindowModal);
	simWin.setVisible(true);
	simWin.show();*/

	//2. Modaless method
	simWin=new simulationWindow();
	simWin->setFixedSize(900,500);

	//passing all the data from this window to simWindow

	//simWin->setNoOfPoints(num_of_points);
	simWin->setGeneralData(m,minX,minY,minZ,maxX,maxY,maxZ,length,width,height,deltaX,deltaY,num_of_points);
		//simWin->setGeneralData(m,&minX,&minY,&minZ,&maxX,&maxY,&maxZ,&length,&width,&height,&deltaX,&deltaY,&num_of_points);
	
	//only send vtk actor, cubeaxesactor and outline actor
	simWin->setVTKData(actor, outlineActor, cubeAxesActor);

	//SHOWING the simulation window
	simWin->show();
	//simulationButton->setEnabled(false);
	
}


//OTHERS METHODS
void MainWindow::clearDataDisplay(){
	ui->textEdit->clear();
	ui->lineEditFilePath->clear();	
}

void MainWindow::resetMaxMinValue(){
	minX=9999999.9999;
	minY=9999999.9999;
	minZ=9999999.9999;
	maxX=-9999999.9999;
	maxY=-9999999.9999;
	maxZ=-9999999.9999;
}

void MainWindow::setMaxMinValue(double xi,double yi,double zi){
		if(maxX<xi){
			maxX=xi;
		}
		if(maxY<yi){
			maxY=yi;
		}
		if(maxZ<zi){
			maxZ=zi;
		}
		if(minX>xi){
			minX=xi;
		}
		if(minY>yi){
			minY=yi;
		}
		if(minZ>zi){
			minZ=zi;
		}

}

void MainWindow::calcBoundingBox(){
	length=maxX-minX;
	width=maxY-minY;
	height=maxZ-minZ;

}

void MainWindow::showSTL(MatrixXd m_points, MatrixXi m_faces){//The global Dataset (vtkPolyData STLData) is updated

	resetMaxMinValue();	

	//==============Filling Global vtkPolyData STLData: The points and cells=====================
	STLData->Reset();
	
	vtkSmartPointer<vtkPoints> points=vtkSmartPointer<vtkPoints>::New(); //Point Data
	vtkSmartPointer<vtkCellArray> cells=vtkSmartPointer<vtkCellArray>::New(); //Cell Data, these are the two Data set geometry
	
	//1. inserting point
	double point[3];
	for(int i=0;i<num_of_points;i++){		
		point[0]=m_points(i,0);
		point[1]=m_points(i,1);
		point[2]=m_points(i,2);
		points->InsertPoint(i,point);	

		//Determining Max and Min value for X, Y and Z
		setMaxMinValue(point[0],point[1],point[2]);
	}
	calcBoundingBox();

	//2. inserting cells
	for(int i=0;i<num_of_cells;i++){		
		vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();			
		triangle->GetPointIds()->SetId ( 0, m_faces(i,0));
		triangle->GetPointIds()->SetId ( 1, m_faces(i,1));
		triangle->GetPointIds()->SetId ( 2, m_faces(i,2) );
		cells->InsertNextCell ( triangle );
	}

	STLData->SetPoints (points);
	STLData->SetPolys (cells);

	STLData->Update();

	//=========================== GRPAHIC PIPELINE =======================================
	//GLOBAL PolyData and DataMapper
	ren->RemoveAllViewProps();
	ren->ResetCamera();
	//ren->GetRenderWindow()->Render();	
		
	pointCloudMapper->SetInput(STLData);
	pointCloudMapper->ScalarVisibilityOff();
	pointCloudMapper->Update(); //We need to update because it will be rendered not now (later) --> VTK Lazy update
		
	//global actor
	actor->SetMapper(pointCloudMapper);
	actor->GetProperty()->SetColor(1.0,0.0,0.0);
	actor->SetVisibility(1);
	//actor->Render(ren,pointCloudMapper);

	//local actor: to change the color
	vtkActor *local_actor=vtkActor::New();
	local_actor->SetMapper(pointCloudMapper);
	local_actor->GetProperty()->SetColor(0.0,1.0,0.0);

	//Defining bounding box: this is global vtkActor object		
	vtkPolyDataNormals *normals=vtkPolyDataNormals::New(); //local
	normals->SetInput(pointCloud);				

	vtkOutlineFilter *outline=vtkOutlineFilter::New(); //local
	outline->SetInput(normals->GetOutput());
		
	vtkPolyDataMapper *outlineMapper=vtkPolyDataMapper::New(); //local
	outlineMapper->SetInput(outline->GetOutput());

	outlineActor->SetMapper(outlineMapper); //Global
	outlineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
		
	outlineMapper->Delete();
	outline->Delete();
	normals->Delete();		

	//vtkCamera
	/*vtkCamera *camera=vtkCamera::New();
	camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation		
	camera->SetPosition(0,0,length*4);
	camera->Azimuth(-45.0);
	camera->Elevation(-45.0);*/
	vtkCamera *camera;
	camera=ren->GetActiveCamera();
	ren->SetActiveCamera(camera);

	//showing axes
	vtkAxesActor *axes=vtkAxesActor::New(); //Defoult axes position is on the origin of the viewport		
	axes->SetTotalLength(0.005,0.005,0.005);	
				
	//ren->SetNearClippingPlaneTolerance(0.000001);
	ren->ResetCameraClippingRange();
	//ren->AddActor(actor);		
	ren->AddActor(local_actor);		
	ren->GetRenderWindow()->Render();		
	  
	//DELETING allocaed memory of vtk objects	
	local_actor->Delete();
	axes->Delete(); 
}

void MainWindow::showPoints(MatrixXd m_points){ //The global Dataset (vtkPolyData pointCloud) is updated
	   //NOTE: This function is to show points cloud.
	   //INPUT: point cloid in nx3 matrix
	   //OUTPUT: the point clous is shown on vtkWidget

		vtkPoints *points=vtkPoints::New(); //Point Data
		vtkCellArray *strips=vtkCellArray::New(); //Cell Data, these are the two VtkObject Basic Structure

		double point[3];		
		
		//Setting the color base on the Z value
		vtkDoubleArray* pointColor = vtkDoubleArray::New();
		pointColor->SetName("pointColor");
				
		if(num_of_points<100000){
			strips->InsertNextCell(100001);
		}
		else{
			strips->InsertNextCell(num_of_points+1);
		}
		resetMaxMinValue();		

		for(int i=0;i<num_of_points;i++){
	
			//Inserting the data into point[]
			point[0]=m_points(i,0);
			point[1]=m_points(i,1);
			point[2]=m_points(i,2);

			points->InsertPoint(i,point);
			strips->InsertCellPoint(i);			
		
			//Determining Max and Min value for X, Y and Z
			setMaxMinValue(point[0],point[1],point[2]);

			//set the color according to the z hight
			pointColor->InsertNextValue(point[2]*1000.0);		

		}

		//TO SOLVE SMALL DATA PROBLEM, Filling the points with fake values upto 100000 points		
		if(num_of_points<100000){
			int iter;
			for(iter=num_of_points;iter<=100000;iter++){
			//for(iter=counter;iter<=100000;iter++){
				points->InsertPoint(iter,point); //inserting the value of the last point
				strips->InsertCellPoint(iter);
				pointColor->InsertNextValue(point[2]*1000.0);
			}
		}

		calcBoundingBox();

		//GLOBAL PolyData and DataMapper
		ren->RemoveAllViewProps();
		ren->ResetCamera();
		//ren->GetRenderWindow()->Render();	
		
		pointCloud->Reset(); //to reset the data inside the vtkPolyData dataset object (Release the data), but this only reset the data
		pointCloud->SetPoints(points);		
		//pointCloud->GetPointData()->SetScalars(pointColor);//set the point color based on the value we insert to the pointcolor object
		pointCloud->SetVerts(strips); //show as points		
		pointCloud->Update(); //We need to update because it will be rendered not now (later)--> VTK Lazy update

		pointCloudMapper->SetInput(pointCloud);
		pointCloudMapper->ScalarVisibilityOff();
		pointCloudMapper->Update(); //We need to update because it will be rendered not now (later) --> VTK Lazy update
		
		//global actor
		actor->SetMapper(pointCloudMapper);
		actor->GetProperty()->SetColor(1.0,0.0,0.0);
		actor->SetVisibility(1);
		//actor->Render(ren,pointCloudMapper);

		//local actor: to change the color
		vtkActor *local_actor=vtkActor::New();
		local_actor->SetMapper(pointCloudMapper);
		local_actor->GetProperty()->SetColor(0.0,1.0,0.0);

		//Defining bounding box: this is global vtkActor object		
		vtkPolyDataNormals *normals=vtkPolyDataNormals::New(); //local
		normals->SetInput(pointCloud);				

		vtkOutlineFilter *outline=vtkOutlineFilter::New(); //local
		outline->SetInput(normals->GetOutput());
		
		vtkPolyDataMapper *outlineMapper=vtkPolyDataMapper::New(); //local
		outlineMapper->SetInput(outline->GetOutput());

		outlineActor->SetMapper(outlineMapper); //Global
		outlineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
		
		outlineMapper->Delete();
		outline->Delete();
		normals->Delete();		

		//vtkCamera
		/*vtkCamera *camera=vtkCamera::New();
		camera->SetFocalPoint((maxX+minX)/2,(maxY+minY)/2,(maxZ+minZ)/2);//origin of the rotation		
		camera->SetPosition(0,0,length*4);
		camera->Azimuth(-45.0);
		camera->Elevation(-45.0);*/
		vtkCamera *camera;
		camera=ren->GetActiveCamera();
		ren->SetActiveCamera(camera);

		//showing axes
		vtkAxesActor *axes=vtkAxesActor::New(); //Defoult axes position is on the origin of the viewport		
		axes->SetTotalLength(0.005,0.005,0.005);	
				
		//ren->SetNearClippingPlaneTolerance(0.000001);
		ren->ResetCameraClippingRange();
		//ren->AddActor(actor);		
		ren->AddActor(local_actor);		
		ren->GetRenderWindow()->Render();		
	  
		//DELETING allocaed memory of vtk objects
		points->Delete();
		strips->Delete();
		pointColor->Delete();
		//camera->Delete();
		local_actor->Delete();
		axes->Delete(); 
}