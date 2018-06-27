#ifndef SIMULATIONWINDOW_H
#define SIMULATIONWINDOW_H

#include <QWidget>
#include "ui_simulationWindow.h"

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
using namespace std;

//EIGEN LIB
#include <Eigen/Dense>
using namespace Eigen;

class simulationWindow : public QWidget
{
	Q_OBJECT

public:
	simulationWindow(QWidget *parent = 0);
	~simulationWindow();

	//to send data from the caller windopw to this window
	void setNoOfPoints(int);
	void setGeneralData(MatrixXd,double,double,double,double,double,double,double,double,double,double,double,int);
	//void setGeneralData(MatrixXd,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,int*);
	void setVTKData(vtkActor*, vtkActor*, vtkCubeAxesActor2D*);

private slots:
	void pushButtonSimulation_clicked();
	void pushButtonCancel_clicked();

private:
	Ui::simulationWindow ui;
	//Ui::simulationWindow *ui;

	//variables
	MatrixXd m;//Original matrix to hold the opened point cloud
	double minX, minY, minZ, maxX, maxY, maxZ;
	double length, width, height;
	double deltaX, deltaY;
	int points; //number of points

	//VTK variables
	vtkPolyDataMapper* mapper;
    vtkActor* actor;
    vtkRenderer* ren;
	vtkPolyData* pointCloud;
	vtkCamera* cameraGlobal;
	vtkPolyDataMapper* pointCloudMapper;
	vtkCubeAxesActor2D* cubeAxesActor;
	vtkCubeAxesActor2D *axes;
	vtkActor *outlineActor;
	vtkPoints *pointsGlobal;
	vtkCellArray *stripsGlobal;
	vtkDoubleArray* pointColorGlobal;
		
	//create signal and slots
	void createSignalAndSlot();	
	void showData();
	void valueInitialization();

	//points animation
	void pointAnimation(MatrixXd);

	//set Flags
	int flag_animation_enabled;

};

#endif // SIMULATIONWINDOW_H
