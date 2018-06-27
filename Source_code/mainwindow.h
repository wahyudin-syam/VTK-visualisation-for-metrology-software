#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "simulationWindow.h"

#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QToolButton>
#include <QToolBar>
#include <QIcon>

//VTK Library
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkCylinderSource.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkCubeAxesActor2D.h"
#include "vtkOutlineFilter.h"
#include "vtkPolyDataNormals.h" //Normal for cell

//EIGEN LIB
#include <Eigen/Dense>
using namespace Eigen;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void newFile();
    void about();
    void toolButton1_clicked();
    void toolButton2_clicked();
	void deleteButton_clicked();
	void cropButton_clicked();
	void filterButton_clicked();
	void lineFitting_clicked();
	void planeFitting_clicked();
	void sphereFitting_clicked();
	void cylinderFitting_clicked();
	void straightness_clicked();
	void flatness_clicked();
	void sphereForm_clicked();
	void cylindricity_clicked();
	void holeDepth_clicked();
	void boxDepth_clicked();
	void STLGeneration_clicked();
	void triangleMeshing_clicked();
	void STLDelete_clicked();
	void STLCrop_clicked();
	void simulation_clicked();

	void setValueSlider1();
	void saveOutput();
	void upView();
	void rightView();
	void bottomView();
	void leftView();
	void clearView();
	void showOriginalData();
	void showHideAxes();

private:
    Ui::MainWindow *ui;

	//Methods section
	//===========================================
    //for MENUS and ACTIONS (SUB-MENU) procedure
    void createActions();
    void createMenus();
	void createSignalAndSlot();

    //for TOOLBAR
    void createToolBar();

	//for creating help text
	void setHelpText();

	//other method or function
	void setFlags();
	void setComponentIcon();
	void setComponentStatus();
	void clearDataDisplay();
	void resetMaxMinValue();
	void setMaxMinValue(double,double,double);
	void calcBoundingBox();
	void showPoints(MatrixXd);
	void showSTL(MatrixXd, MatrixXi);

	//Property section
	//==================================
    //for MENUS and ACTIONS (SUB-MENU)
    QMenu *fileMenu;
    QMenu *helpMenu;
    QAction *newAct; //under fileMenu
    QAction *aboutAct; //under helpMenu

    //for TOOLBAR
    QToolBar *toolBar;
    QToolButton *openButton;
    QToolButton *saveButton;
	QToolButton *deleteButton;
	QToolButton *cropButton;
	QToolButton *filterButton;
	QToolButton *lineFittingButton;
	QToolButton *planeFittingButton;
	QToolButton *sphereFittingButton;
	QToolButton *cylinderFittingButton;	
	QToolButton *straightnessButton;	
	QToolButton *flatnessButton;	
	QToolButton *sphereFormButton;
	QToolButton *cylindricityButton;
	QToolButton *holeDepthButton;	
	QToolButton *boxDepthButton;	
	QToolButton *STLGenerationButton;	
	QToolButton *simulationButton;
	QToolButton *triangleMeshingButton;
	QToolButton *STLDeleteButton;
	QToolButton *STLCropButton;

	//VTK variables
	vtkCylinderSource* source;
    vtkPolyDataMapper* mapper;
    vtkActor* actor;
    vtkRenderer* ren;
	vtkPolyData* pointCloud;
	vtkPolyDataMapper* pointCloudMapper;
	vtkCubeAxesActor2D* cubeAxesActor;
	vtkCubeAxesActor2D *axes;
	vtkActor *outlineActor;
	vtkPolyData* STLData;

	//Matrices
	//MatrixXd m(3,3); //Original matrix to hold the opened point cloud
	MatrixXd m;//Original matrix to hold the opened point cloud (both for Point Cloud data and STL data)
	MatrixXi mCells;
	double minX, minY, minZ, maxX, maxY, maxZ;
	double length, width, height;
	double deltaX, deltaY;
	int num_of_points;
	int num_of_cells;
	//long num_of_points;

	//FLAGS variable
	int flag_file_is_opened;
	int flag_axis_shown;
	int flag_simulation_window;

	//other WINDOW
	simulationWindow *simWin;
};

#endif // MAINWINDOW_H
