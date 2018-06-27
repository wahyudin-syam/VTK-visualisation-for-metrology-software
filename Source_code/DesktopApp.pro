#-------------------------------------------------
#
# Project created by QtCreator 2015-02-02T18:34:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DesktopApp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h \
    function.h

FORMS    += mainwindow.ui

#This is for comment
#This is additional setting when we are importing external library

#this is to include the eigen library. we can directly use it without CMAKE with MinGW compiler since
#it does not need the linking process (.lib)
#INCLUDEPATH += F:/MyQtApps/QtDesktopApp/include


#RESOURCES file
RESOURCES = resources.qrc

#OpenCV 1.0
#INCLUDEPATH += C:\Program Files\OpenCV\cv

#CONFIG( debug, debug|release ) {
#LIBS += -LC:\\Program Files\\OpenCV\\lib\
#    -lcxcored\
#    -lhighguid\
#    -lmld\
#    -lcvd\
#}
#else {
#LIBS += -LC:\\Program Files\\OpenCV\\lib\
#    -lcxcore\
#    -lhighgui\
#    -lml\
#    -lcv\
#}

#LIBS +=-LF:/MyQtApps/QtDesktopApp/DesktopApp/opencvlib\
#    -LD:/OpenCV2.1/bin

##PCL Library
#INCLUDEPATH += -LF:/PCL 1.5.1/include/pcl-1.5
#LIBS +=-LF:/PCL 1.5.1/lib
