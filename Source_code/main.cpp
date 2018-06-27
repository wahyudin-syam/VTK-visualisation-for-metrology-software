#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
	
    w.setFixedSize(1200,750); //disabling the maximize button
	
	//a.setActiveWindow(w);
	//a.setMainWidget(&w);
	
    w.show();

    return a.exec();
}
