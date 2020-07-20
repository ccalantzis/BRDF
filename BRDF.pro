TEMPLATE = app
LANGUAGE  = C++

LIBIGLDIR += ../libigl
EIGENDIR += $$LIBIGLDIR/external/eigen


linux:OPENCV = /usr/local
linux:OPENCV_INC = $$OPENCV/include/opencv4/opencv2/
linux:OPENCV_LIB = $$OPENCV/lib

QMAKE_CXXFLAGS += -std=c++11

TARGET = brdf
DEPENDPATH += . $$LIBIGLDIR/include $$EIGENDIR
INCLUDEPATH += . $$LIBIGLDIR/include $$EIGENDIR $$OPENCV_INC $$OPENCV/include/opencv4/ $OPENCV_LIB

# Lib sources
QT += core xml opengl

HEADERS += brdfdata.h
HEADERS += glutcallbacks.h
HEADERS += glut.h
HEADERS += glext.h

SOURCES += brdfdata.cpp
SOURCES += glutcallbacks.cpp
SOURCES += main.cpp

linux:LIBS += -L $$OPENCV_LIB/ -l:libopencv_core.so.4.3 -L $$OPENCV_LIB/ -l:libopencv_imgproc.so.4.3 -L $$OPENCV_LIB/ -l:libopencv_imgcodecs.so.4.3 $$OPENCV_LIB/ -l:libopencv_highgui.so.4.3 -lGLU levmar/liblevmar.a -lstdc++ -lGL -lglut -lGLU -lopenblas -lpthread
#linux:LIBS += `pkg-config --libs opencv` levmar/liblevmar.a -lstdc++ -lGL -lglut -lGLU -lopenblas -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui
