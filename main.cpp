//============================================================================
// Name        : main.cpp
// Author      : Christian Thurow
// Description : BRDF Calculation for Rapid Prototyping Project at TU-Berlin
//============================================================================

#if defined(_MSC_VER)
#include "stdafx.h"
#endif
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "brdfdata.h"
#include <GL/gl.h> //do we really need that or just glut?
#include "glut.h"
#include "glutcallbacks.h"

void Render(CBRDFdata* data, int argc, char** argv);

CBRDFdata m_brdf;
int m_width = 1920;
int m_height = 1080;

// main function
int main(int argc, char** argv)
{	
    if(argc < 3)
    {
        std::cout << "required command line arguments: path to image folder, path to obj file, path to cal file\n";
        return -1;
    }
	//data structure for brdf
	m_brdf = CBRDFdata();

    std::string image_folder_path = argv[1];
    std::string obj_path = argv[2];
    std::string cal_path = argv[3];

	//read in 3d model + calc surface normals
    bool code = m_brdf.LoadModel(obj_path);
    if(!code) return -1;
	m_brdf.m_model = 1; //0: Phong, 1: Blinn-Phong

	//read in 16 images
    code = m_brdf.LoadImages(image_folder_path);
    if(!code) return -1;
    //m_brdf.PrintImages();
    m_brdf.SubtractAmbientLight(image_folder_path); //not really important, but correct

	//optional output
	//m_brdf.PrintNormalisedImages();
	
	//read in geometry and camera infos
    code = m_brdf.LoadCameraParameters(cal_path);
    if(!code) return -1;

	//initialise led positions
	m_brdf.InitLEDs();
	//render model
    std::cout << "begin rendering" << '\n';
    Render(&m_brdf, argc, argv);

    return 0;
}

void Render(CBRDFdata* data, int argc, char** argv)
{
	//render model in opengl
	//let user interactively choose whether to show BRDF-correct lighting
	// ->on key press, activate a shader that takes into account the brdf values of each surface

	//show ground plane with origin
	//show model, in correct relation to origin
	//show camera position

    // initialize OpenGL (see glutcallbacks.c)
    Init();

	//let user rotate and translate around

    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowPosition(1000, 100);
	glutInitWindowSize(m_width, m_height);
	glutInit(&argc, argv);
    glutCreateWindow("Scanned Mesh");

    // register GLUT callbacks (see glutcallbacks.c)
    RegisterCallbacks();

    // start GLUT event loop
    glutMainLoop();
}
