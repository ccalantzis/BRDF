﻿//============================================================================
// Name        : glutcallbacks.cpp
// Author      : Christian Thurow
// Description : Rendering Engine
//============================================================================

#if defined(_MSC_VER)
#include "stdafx.h"
#endif
#pragma once
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fstream>

#include "brdfdata.h"
#include "glutcallbacks.h"

extern "C" {
#include "glut.h"
#include <GL/gl.h>
#include <GL/glx.h>
}

extern CBRDFdata m_brdf;
extern int m_width;
extern int m_height;

float color_1[4] = {1, 1, 1, 1};	//white
float color_2[4] = {1, 1, 0, 1};	//yellow
float color_3[4] = {1, 0, 0, 1};	//red

void* font = GLUT_BITMAP_8_BY_13;

long m_frameCounter[100];
int m_frame = 0;

cv::Mat m_pixelMap;

bool m_calcNow = false;
bool m_defaultView = true;
bool m_calcedOnce = false;
bool m_showMapping = false;
bool m_showShadedBRDF = false;

//camera position
float m_eyeX;
float m_eyeY;
float m_eyeZ;

float m_centerX = 0.0f;
float m_centerY = 0.0f;
float m_centerZ = 0.0f;

float m_stepLength = 1.0;

float m_glFar = 1000.0f;
double m_fps = 0.0;
//double m_glFov = 86;
//double m_glFovV = 73;
double m_glFov = 78;
double m_glFovV = 49;

GLuint m_meshListID = 0;

int m_lbutton = 0; //is left mouse button pressed
int m_lastMouseX = 0;
int m_lastMouseY = 0;

bool m_lights = true;
bool single_BRDF = false;

GLenum my_program;
GLenum my_fragment_shader;
GLenum my_vertex_shader;

//GL ARB extension for Shaders:
PFNGLCREATEPROGRAMOBJECTARBPROC		glCreateProgramObjectARB;
PFNGLCREATESHADEROBJECTARBPROC		glCreateShaderObjectARB;
PFNGLSHADERSOURCEARBPROC			glShaderSourceARB;
PFNGLCOMPILESHADERARBPROC			glCompileShaderARB;
PFNGLATTACHOBJECTARBPROC			glAttachObjectARB;
PFNGLLINKPROGRAMARBPROC				glLinkProgramARB;
PFNGLUSEPROGRAMOBJECTARBPROC		glUseProgramObjectARB;
PFNGLGETATTRIBLOCATIONARBPROC		glGetAttribLocationARB;
PFNGLVERTEXATTRIB3FPROC				glVertexAttrib3f;
PFNGLGETINFOLOGARBPROC				glGetInfoLogARB;
PFNGLGETUNIFORMLOCATIONARBPROC		glGetUniformLocationARB;
PFNGLGETOBJECTPARAMETERIVARBPROC	glGetObjectParameterivARB;
PFNGLUNIFORM3FPROC					glUniform3f;
PFNGLUNIFORM1FPROC					glUniform1f;

#if defined(CLOCK_MONOTONIC)
// From https://stackoverflow.com/questions/13542165/c-linux-tickcount
static uint32_t GetTickCount()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (uint32_t)(ts.tv_nsec / 1000000) + ((uint64_t)ts.tv_sec * 1000ull);
}
#endif

//char* GetShaderSource(char* filename)
//{
//	char* charbuffer = NULL;
//	vector<char> buffer;
//	buffer.clear();

//	std::ifstream file;
//	file.open(filename, std::ios::in | std::ios::binary);

//	if(!file.fail())
//	{
//		while(!file.eof())
//		{
//			char currChar = file.get();
//			buffer.push_back(currChar);
//		}
//		file.close();
//    }

//	//dirty hack..
//	std::vector<char>::iterator it = buffer.begin();
//	buffer.erase(it);
//	it = buffer.begin();
//	buffer.erase(it);
//	it = buffer.begin();
//	buffer.erase(it);

//	charbuffer = new char[buffer.size()];
//	for(int i=0; i<buffer.size()-1; i++)
//	{
//		charbuffer[i] = buffer.at(i);
//	}
//	charbuffer[buffer.size()-1] = '\0';

//	buffer.clear();

//	return charbuffer;
//}

void InitShader()
{
    std::string line;
    std::string file;

    std::ifstream myfile ("fragmentShader.sl");

    if (myfile.is_open())
    {
        while (! myfile.eof() )
        {
            getline (myfile,line);
            file += line; // Concatenate every lines
        }
        myfile.close();
    }

    const char *my_fragment_shader_source = file.c_str();
    //Init Shader
//	char* my_fragment_shader_source;
    //char* my_vertex_shader_source;

    // Get Vertex And Fragment Shader Sources
//	my_fragment_shader_source = GetShaderSource("fragmentShader.sl");
    //my_vertex_shader_source = GetShaderSource("vertexShader.sl");

#if !defined(_WIN_VER)
#define wglGetProcAddress(name) glXGetProcAddress((GLubyte *)name)
#endif

    glCreateProgramObjectARB = (PFNGLCREATEPROGRAMOBJECTARBPROC)wglGetProcAddress("glCreateProgramObjectARB");
    glCreateShaderObjectARB = (PFNGLCREATESHADEROBJECTARBPROC)wglGetProcAddress("glCreateShaderObjectARB");
    glShaderSourceARB = (PFNGLSHADERSOURCEARBPROC)wglGetProcAddress("glShaderSourceARB");
    glCompileShaderARB = (PFNGLCOMPILESHADERARBPROC)wglGetProcAddress("glCompileShaderARB");
    glAttachObjectARB = (PFNGLATTACHOBJECTARBPROC)wglGetProcAddress("glAttachObjectARB");
    glLinkProgramARB = (PFNGLLINKPROGRAMARBPROC)wglGetProcAddress("glLinkProgramARB");
    glUseProgramObjectARB = (PFNGLUSEPROGRAMOBJECTARBPROC)wglGetProcAddress("glUseProgramObjectARB");
    glGetAttribLocationARB = (PFNGLGETATTRIBLOCATIONARBPROC)wglGetProcAddress("glGetAttribLocationARB");
    glVertexAttrib3f = (PFNGLVERTEXATTRIB3FPROC)wglGetProcAddress("glVertexAttrib3f");
    glGetInfoLogARB = (PFNGLGETINFOLOGARBPROC)wglGetProcAddress("glGetInfoLogARB");
    glGetUniformLocationARB = (PFNGLGETUNIFORMLOCATIONARBPROC)wglGetProcAddress("glGetUniformLocationARB");
    glGetObjectParameterivARB = (PFNGLGETOBJECTPARAMETERIVARBPROC)wglGetProcAddress("glGetObjectParameterivARB");
    glUniform3f = (PFNGLUNIFORM3FPROC)wglGetProcAddress("glUniform3f");
    glUniform1f = (PFNGLUNIFORM1FPROC)wglGetProcAddress("glUniform1f");

    // Create Shader And Program Objects
    my_program = glCreateProgramObjectARB();
    my_fragment_shader = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);

    // Load Shader Sources
    glShaderSourceARB(my_fragment_shader, 1, (const GLcharARB**)&my_fragment_shader_source, NULL);

    // Compile The Shaders
    glCompileShaderARB(my_fragment_shader);

    int compiled = 0;
    glGetObjectParameterivARB(my_fragment_shader, GL_OBJECT_COMPILE_STATUS_ARB, &compiled);
    int logLength = 0;
    glGetObjectParameterivARB(my_fragment_shader, GL_OBJECT_INFO_LOG_LENGTH_ARB, &logLength);

    // Attach The Shader Objects To The Program Object
    glAttachObjectARB(my_program, my_fragment_shader);

    // Link The Program Object
    //glLinkProgramARB(my_program);

    //======VERTEX SHADER=====================

    //my_vertex_shader = glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);

    //glShaderSourceARB(my_vertex_shader, 1, (const GLcharARB**)&my_vertex_shader_source, NULL);

    //glCompileShaderARB(my_vertex_shader);

    //compiled = 0;
    //glGetObjectParameterivARB(my_vertex_shader, GL_OBJECT_COMPILE_STATUS_ARB, &compiled);
    //logLength = 0;
    //glGetObjectParameterivARB(my_vertex_shader, GL_OBJECT_INFO_LOG_LENGTH_ARB, &logLength);

    // Attach The Shader Objects To The Program Object
    glAttachObjectARB(my_program, my_vertex_shader);

    // Link The Program Object
    glLinkProgramARB(my_program);

    //free(my_fragment_shader_source);
    //free(my_vertex_shader_source);
}

void Init(void)
{
    InitShader();

    // usual shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // clear background to black and clear depth buffer
    glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // enable depth test (z-buffer)
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // enable normalization of vertex normals
    glEnable(GL_NORMALIZE);

    // initial view definitions
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // perspective projection
    gluPerspective(m_glFov, 1.0, 1.0, m_glFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    ResetCamera();
}

void DrawGroundPlane(void)
{
    glPushMatrix();
    //glColor4f(0.0, 0.0, 1.0, 1.0);

    ////square
    //glBegin(GL_QUADS);
    //	glNormal3f( 0.0f, 1.0, 0.0f);
    //	glVertex3f(-10, 0, -10);
    //	glVertex3f(10, 0, -10);
    //	glVertex3f(10, 0, 10);
    //	glVertex3f(-10, 0, 10);
    //glEnd();

    glColor4d(1.0, 1.0, 1.0, 1.0);

    //lines
    glBegin(GL_LINES);
        glVertex3d(-10, 0.01, 0);
        glVertex3d(10, 0.01, 0);
    glEnd();
    glBegin(GL_LINES);
        glVertex3d(0, 0.01, -10);
        glVertex3d(0, 0.01, 10);
    glEnd();

    glPopMatrix();
}

void DrawMesh()
{
    //Draws the mesh

    glPushMatrix();
    glTranslatef(0,0,0);

    int my_vec3R_location = -1;
    int my_vec3G_location = -1;
    int my_vec3B_location = -1;
    int my_cosPhi_location = -1;

    //if(m_showShadedBRDF && m_calcedOnce)
    //{
    //	//use the custom shader instead of fixed function OpenGL
    //	glUseProgramObjectARB(my_program);
    //	int testLoc = glGetUniformLocationARB(my_program, "test");
    //	my_vec3R_location = glGetUniformLocationARB(my_program, "brdf_vecR");
    //	my_vec3G_location = glGetUniformLocationARB(my_program, "brdf_vecG");
    //	my_vec3B_location = glGetUniformLocationARB(my_program, "brdf_vecB");
    //	my_cosPhi_location = glGetUniformLocationARB(my_program, "cosPhi");
    //}
    //else
    //	glUseProgramObjectARB(0);

    if(!m_showShadedBRDF)
    {
        if(m_meshListID <= 0)
        {
            m_meshListID = glGenLists(1);
            glNewList(m_meshListID, GL_COMPILE);

            glColor4d(1.0, 1.0, 1.0, 1.0);

            for(int i=0; i < m_brdf.m_faces.rows(); i++)
            {
                glBegin(GL_TRIANGLES);
                    glNormal3d(m_brdf.face_normals(i,0), m_brdf.face_normals(i,1), m_brdf.face_normals(i,2));
                    glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,0),0), m_brdf.m_vertices(m_brdf.m_faces(i,0),1), m_brdf.m_vertices(m_brdf.m_faces(i,0),2));
                    glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,1),0), m_brdf.m_vertices(m_brdf.m_faces(i,1),1), m_brdf.m_vertices(m_brdf.m_faces(i,1),2));
                    glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,2),0), m_brdf.m_vertices(m_brdf.m_faces(i,2),1), m_brdf.m_vertices(m_brdf.m_faces(i,2),2));

                glEnd();
            }

            glEndList();
        }
        else
            glCallList(m_meshListID);
    }
    else
    {
        for(int i=0; i < m_brdf.m_faces.rows(); i++)
        {
            //TODO: auslagern auf grafikkarte(in den fragment shader)

            //get the light source position of source 0
            Eigen::RowVector3d lpos;
            lpos << m_eyeX, m_eyeY, m_eyeZ; //same like the camera position)

            //get center of current triangle
            GLdouble objectX = 0.0;
            GLdouble objectY = 0.0;
            GLdouble objectZ = 0.0;

            for (int j = 0; j < 3; j++)
            {
                objectX += m_brdf.m_vertices(m_brdf.m_faces(i,j),0);
                objectY += m_brdf.m_vertices(m_brdf.m_faces(i,j),1);
                objectZ += m_brdf.m_vertices(m_brdf.m_faces(i,j),2);
            }

            objectX /= 3.0; objectY /= 3.0; objectZ /= 3.0;

            //calc the light direction
            Eigen::RowVector3d lightDir;
            lightDir << lpos[0] - objectX, lpos[1] - objectY, lpos[2] - objectZ;
            lightDir.normalize();

            //camera direction
            Eigen::RowVector3d viewDir;
            viewDir << m_eyeX - m_centerX, m_eyeY - m_centerY, m_eyeZ - m_centerZ;
            viewDir.normalize();

            //calc the half vector between light source vector and view vector
            Eigen::RowVector3d h = lightDir + viewDir;
            h.normalize();

            //calc cos(phi) = dot(normal, lightvector)
            double cosLN = m_brdf.face_normals(i, m_brdf.face_normals.row(i).cwiseProduct(lightDir).sum());
            double brdfB = 0.0;
            double brdfG = 0.0;
            double brdfR = 0.0;


            if(m_brdf.m_model == 1) //BLINN-PHONG!
            {
               //calc cos(thetaDash) = dot(normal, h)
               double cosNH = m_brdf.face_normals.row(i).cwiseProduct(h).sum();

               if(single_BRDF)
               {
                   brdfB = m_brdf.single_brdf(0).kd * cosLN + m_brdf.single_brdf(0).ks * (pow(cosNH, m_brdf.single_brdf(0).n));
                   brdfG = m_brdf.single_brdf(1).kd * cosLN + m_brdf.single_brdf(1).ks * (pow(cosNH, m_brdf.single_brdf(1).n));
                   brdfR = m_brdf.single_brdf(2).kd * cosLN + m_brdf.single_brdf(2).ks * (pow(cosNH, m_brdf.single_brdf(2).n));
               }
               else
               {
                   brdfB = m_brdf.brdf_surfaces(i,0).kd * cosLN + m_brdf.brdf_surfaces(i,0).ks * (pow(cosNH, m_brdf.brdf_surfaces(i,0).n));
                   brdfG = m_brdf.brdf_surfaces(i,1).kd * cosLN + m_brdf.brdf_surfaces(i,1).ks * (pow(cosNH, m_brdf.brdf_surfaces(i,1).n));
                   brdfR = m_brdf.brdf_surfaces(i,2).kd * cosLN + m_brdf.brdf_surfaces(i,2).ks * (pow(cosNH, m_brdf.brdf_surfaces(i,2).n));
               }
            }
            else if(m_brdf.m_model == 0) //PHONG!
            {

               //calc
               GLdouble scale_factor = m_brdf.face_normals(i,0) * lightDir[0] +m_brdf.face_normals(i,1) * lightDir[1] + m_brdf.face_normals(i,2) * lightDir[2];
               Eigen::RowVector3d P = -scale_factor * m_brdf.face_normals.row(i);
               Eigen::RowVector3d R = lightDir - 2*P;
               //calc cos(theta) = dot(R, viewDir)
               float cosRV = viewDir.cwiseProduct(R).sum();

               //Phong colors
               if (single_BRDF)
               {
                   brdfB = m_brdf.single_brdf(0).kd * cosLN + m_brdf.single_brdf(0).ks * ((m_brdf.single_brdf(0).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.single_brdf(0).n));
                   brdfG = m_brdf.single_brdf(1).kd * cosLN + m_brdf.single_brdf(1).ks * ((m_brdf.single_brdf(1).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.single_brdf(1).n));
                   brdfR = m_brdf.single_brdf(2).kd * cosLN + m_brdf.single_brdf(2).ks * ((m_brdf.single_brdf(2).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.single_brdf(2).n));
               }
               else
               {
                   brdfB = m_brdf.brdf_surfaces(i,0).kd * cosLN + m_brdf.brdf_surfaces(i,0).ks * ((m_brdf.brdf_surfaces(i,0).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.brdf_surfaces(i,0).n));
                   brdfG = m_brdf.brdf_surfaces(i,1).kd * cosLN + m_brdf.brdf_surfaces(i,1).ks * ((m_brdf.brdf_surfaces(i,1).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.brdf_surfaces(i,1).n));
                   brdfR = m_brdf.brdf_surfaces(i,2).kd * cosLN + m_brdf.brdf_surfaces(i,2).ks * ((m_brdf.brdf_surfaces(i,2).n + 2.0)/(2.0*CV_PI)) * (pow(cosRV, m_brdf.brdf_surfaces(i,2).n));
               }
            }

            glBegin(GL_TRIANGLES);
                glNormal3d(m_brdf.face_normals(i,0), m_brdf.face_normals(i,1), m_brdf.face_normals(i,2));

                //glUniform3f(my_vec3R_location, m_brdf.m_faces[i].brdf[0].kd, m_brdf.m_faces[i].brdf[0].ks, m_brdf.m_faces[i].brdf[0].n);
                //glUniform3f(my_vec3G_location, m_brdf.m_faces[i].brdf[1].kd, m_brdf.m_faces[i].brdf[1].ks, m_brdf.m_faces[i].brdf[1].n);
                //glUniform3f(my_vec3B_location, m_brdf.m_faces[i].brdf[2].kd, m_brdf.m_faces[i].brdf[2].ks, m_brdf.m_faces[i].brdf[2].n);
                //glUniform1f(my_cosPhi_location, cosPhi);

                glColor4d(brdfR, brdfG, brdfB, 1.0);
                glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,0),0), m_brdf.m_vertices(m_brdf.m_faces(i,0),1), m_brdf.m_vertices(m_brdf.m_faces(i,0),2));
                glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,1),0), m_brdf.m_vertices(m_brdf.m_faces(i,1),1), m_brdf.m_vertices(m_brdf.m_faces(i,1),2));
                glVertex3d(m_brdf.m_vertices(m_brdf.m_faces(i,2),0), m_brdf.m_vertices(m_brdf.m_faces(i,2),1), m_brdf.m_vertices(m_brdf.m_faces(i,2),2));
                glEnd();
        }
    }

    glPopMatrix();

    glUseProgramObjectARB(0);
}


void DrawScene()
{
    glPushMatrix();

    DrawGroundPlane();

    if(m_lights && !m_showShadedBRDF)
    {
        glShadeModel(GL_FLAT);

        glTranslatef(m_eyeX, m_eyeY, m_eyeZ);

        GLfloat LightDiffuse[] = { 1.0f, 1.0f, 1.0f, 0.7f };	// Diffuse Light Values
        GLfloat LightAmbient[] = { 1.0f, 1.0f, 1.0f, 1.0f };	// ambient light values
        GLfloat LightPosition[]= { m_eyeX, m_eyeY, m_eyeZ, 1.0f };	// Light Position

        glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);			// Setup The Ambient Light
        glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);			// Setup The Diffuse Light
        glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);		// Position The Light

        glEnable(GL_LIGHT1);
        glEnable(GL_LIGHTING);

        glTranslatef(-m_eyeX, -m_eyeY, -m_eyeZ);
    }

    DrawMesh();

    glPopMatrix();
}

//write 2d text using GLUT
//The projection matrix must be set to orthogonal before call this function.
void DrawString(std::string str, int x, int y, float color[4], void *font)
{
    glDisable(GL_TEXTURE_2D);

    glColor4fv(color);          //set text color
    glRasterPos3i(x, y, 1);     //set text position

    //loop through all characters in the string
    for(auto i = str.begin(); i!= str.end(); ++i)
    {
        glutBitmapCharacter(font, *i);
    }

    glEnable(GL_TEXTURE_2D);
}

void ResetCamera()
{
    if(!m_defaultView)
        return;

//    m_eyeX = m_brdf.GetCameraOrigin().at<double>(0, 0)/10.0;
//    m_eyeY = m_brdf.GetCameraOrigin().at<double>(0, 1)/10.0;
//    m_eyeZ = m_brdf.GetCameraOrigin().at<double>(0, 2)/10.0;
    m_eyeX = 0;
    m_eyeY = 0;
    m_eyeZ = 100;

    double ax = m_brdf.GetA().at<double>(0, 0);
    double ay = m_brdf.GetA().at<double>(0, 1);
    double az = m_brdf.GetA().at<double>(0, 2);

    //find center on z=0
    double s = m_eyeZ / az;
    m_centerX = m_eyeX - s*ax;
    m_centerY = m_eyeY - s*ay;
    m_centerZ = 0.0;

    m_centerX = 0;
    m_centerY = 0;
    m_centerZ = 0;
}

void DrawOnScreenDisplay()
{
    //show fps and position values
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT1);

    std::string fps = "fps: " + std::to_string((int)m_fps);

    std::string fov = "fovH: " + std::to_string((int)m_glFov);

    std::string fovV = "fovV: " + std::to_string((int)m_glFovV);

    std::string eyeX = "eyeX: " + std::to_string((int)m_eyeX);

    std::string eyeY = "eyeY: " + std::to_string((int)m_eyeY);

    std::string eyeZ = "eyeZ: " + std::to_string((int)m_eyeZ);

    std::string esc = "esc: exit";
    std::string w = "w: increase eyeY";
    std::string s = "s: decrease eyeY";
    std::string a = "a: increase eyeX";
    std::string d = "d: decrease eyeX";
    std::string y = "q: increase eyeZ";
    std::string x = "e: decrease eyeZ";
    std::string l = "l: toggle lights";
    std::string r = "r: reset camera";
    std::string h = "h: increase field of view horizontal";
    std::string j = "j: decrease field of view horizontal";
    std::string b = "b: increase field of view vertical";
    std::string n = "n: decrease field of view vertical";
    std::string p = "p: toggle mapping";
    std::string m = "m: toggle shaded brdf";
    std::string c = "c: calc brdf";

    glPushMatrix();

    DrawString(fps, 10, 10, color_1, font);
    DrawString(eyeZ, 10, 20, color_1, font);
    DrawString(eyeY, 10, 30, color_1, font);
    DrawString(eyeX, 10, 40, color_1, font);
    DrawString(fov, 10, 50, color_1, font);
    DrawString(fovV, 10, 60, color_1, font);

    int i=1;

    DrawString(esc, 10, m_height-10*i++, color_1, font);
    DrawString(w, 10, m_height-10*i++, color_1, font);
    DrawString(s, 10, m_height-10*i++, color_1, font);
    DrawString(a, 10, m_height-10*i++, color_1, font);
    DrawString(d, 10, m_height-10*i++, color_1, font);
    DrawString(y, 10, m_height-10*i++, color_1, font);
    DrawString(x, 10, m_height-10*i++, color_1, font);
    DrawString(p, 10, m_height-10*i++, color_1, font);
    DrawString(r, 10, m_height-10*i++, color_1, font);
    DrawString(h, 10, m_height-10*i++, color_1, font);
    DrawString(j, 10, m_height-10*i++, color_1, font);
    DrawString(b, 10, m_height-10*i++, color_1, font);
    DrawString(n, 10, m_height-10*i++, color_1, font);
    if(m_showShadedBRDF)
        DrawString(m, 10, m_height-10*i++, color_3, font);
    else
        DrawString(m, 10, m_height-10*i++, color_2, font);

    if(m_lights)
        DrawString(l, 10, m_height-10*i++, color_2, font);
    else
        DrawString(l, 10, m_height-10*i++, color_1, font);
    DrawString(c, 10, m_height-10*i++, color_1, font);
    if(single_BRDF)
        DrawString("1: single BRDF function", 10, m_height-10*i++, color_3, font);
    else
        DrawString("1: single BRDF function", 10, m_height-10*i++, color_1, font);

    glPopMatrix();
}

void CalculateFrameRate(uint32_t newVal)
{
    m_frameCounter[m_frame] = newVal;
    m_frame++;
    if(m_frame >= 100)
        m_frame = 0;

    if(m_frame == 0)
    {
        float average_ms_per_frame = (float)(m_frameCounter[99] - m_frameCounter[0])/100;
        m_fps = 1000.0f / average_ms_per_frame;
    }
}

// This creates an asymmetric frustum.
// It converts to 6 params (l, r, b, t, n, f) for glFrustum()
// from given 4 params (fovy, aspect, near, far)
// taken into account the principal point of the camera
// from the intrinsic camera parameters given
void MakeFrustum(double fovY, double aspectRatio, double front, double back)
{
    const double DEG2RAD = 3.14159265 / 180;

    double tangent = tan(fovY/2 * DEG2RAD);   // tangent of half fovY
    double height = front * tangent;          // half height of near plane
    double width = height * aspectRatio;      // half width of near plane

    // params: left, right, bottom, top, near, far
    //offsets
    double cx = m_brdf.GetCX();
    double cy = m_brdf.GetCY();
    double offset_y = 2.0*(m_height/2.0 - cy)/ m_height;
    double offset_x = 2.0*(m_width/2.0  - cx)/ m_width;
    glFrustum(-width+offset_x, width+offset_x, -height-offset_y, height-offset_y, front, back);
    //glFrustum(-width, width, -height, height, front, back);
}


void DrawMapping()
{
    if(!m_showMapping)
        return;

    glPushMatrix();

    std::string sign = ".";
    for(int x=0; x < m_width; x++)
        for(int y=0; y < m_height; y++)
        {
            unsigned int curr = m_pixelMap.at<unsigned int>(y, x);
            if(curr > 0)
                DrawString(sign, x, y, color_1, font);
        }
    glPopMatrix();
}

// display callback for GLUT
void Display_(void)
{
    // clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glClearColor(0.0, 0.0, 0.0, 1.0f);

    //initial view definitions
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double aspect = m_glFov / m_glFovV;

    //gluPerspective(45, aspect, 1.0, m_glFar);
    MakeFrustum(m_glFovV, aspect, 1.0, m_glFar);

    // switch to opengl modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    double ox = m_brdf.GetO().at<double>(0, 0);
    double oy = m_brdf.GetO().at<double>(0, 1);
    double oz = m_brdf.GetO().at<double>(0, 2);
    //gluLookAt(m_eyeX, m_eyeY, m_eyeZ, m_centerX, m_centerY, m_centerZ, -ox, -oy, -oz);
    //gluLookAt(m_eyeX, m_eyeY, m_eyeZ, m_centerX, m_centerY, m_centerZ, 0,1,0);
    gluLookAt(0, 0, 50, 0, 0, 0, 0,1,0);

    //draw the scene
    DrawScene();

    if(m_calcNow)
    {
        //map each pixel with surface on object
        std::cout << "map pixels to 3d model" << std::endl;
        m_pixelMap = m_brdf.CalcPixel2SurfaceMapping();

        //calc the actual BRDF
        std::cout << "begin calculation" << std::endl;
        if(single_BRDF)
        {
            m_brdf.CalcBRDFEquation_SingleBRDF(m_pixelMap);
        }
        else
        {
            m_brdf.CalcBRDFEquation(m_pixelMap);
        }

        std::cout << "done calculation" << std::endl;

        m_calcedOnce = true;
        m_calcNow = false;
    }

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //set to 2D orthogonal projection
    glMatrixMode(GL_PROJECTION);        //switch to projection matrix
    glLoadIdentity();                   //reset projection matrix
    gluOrtho2D(0, m_width, 0, m_height);//set to orthogonal projection

    //on screen infos
    DrawOnScreenDisplay();
    DrawMapping();
    // display back buffer
    glutSwapBuffers();

    CalculateFrameRate(GetTickCount());
}

// reshape-Callback for GLUT
void Reshape(int w, int h)
{
    // reshaped window aspect ratio
    if(h <= 0)
        h = 1;

    float aspect = (float) w / (float) h;
    m_width = w;
    m_height = h;

    // viewport
    glViewport(0,0, (GLsizei) m_width, (GLsizei) m_height);

    // clear background and depth buffer
    glClearColor(0.1,0.1,0.1,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // restore view definition after window reshape
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // perspective projection
    gluPerspective(m_glFov, aspect, 1.0, m_glFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Display_();
}

// keyboard callback
void Keyboard(unsigned char key, int x, int y)
{
    // rotate selected node around
    // x,y and z axes with keypresses
    switch( key )
    {
        case 27: //Escape
            exit(0);
            break;
        case 'l':
            m_lights = !m_lights;
            break;
        case 'a':
            m_eyeX -= m_stepLength;
            break;
        case 'd':
            m_eyeX += m_stepLength;
            break;
        case 'q':
            m_eyeZ -= m_stepLength;
            break;
        case 'e':
            m_eyeZ += m_stepLength;
            break;
        case 'w':
            m_eyeY -= m_stepLength;
            break;
        case 's':
            m_eyeY += m_stepLength;
            break;
        case 'h':
            m_glFov += 0.5f;
            break;
        case 'j':
            m_glFov -= 0.5f;
            break;
        case 'b':
            m_glFovV += 0.5f;
            break;
        case 'n':
            m_glFovV -= 0.5f;
            break;
        case 'p':
            m_showMapping = !m_showMapping;
            break;
        case 'm':
            m_showShadedBRDF = !m_showShadedBRDF;
            break;
        case 'r':
            ResetCamera();
            break;
        case 'c':
            m_calcNow = true;
            break;
        case '1':
            single_BRDF = !single_BRDF;
            break;

        default:
            break;
    }
}

// the right button mouse menu
void MouseMenu(int id)
{
    //switch (id)
    //{
    //	case 1: exit(0);
    //	default:
    //		break;
    //}
}

// mouse motion
void MouseMotion(int x, int y)
{
    // rotate selected node when left mouse button is pressed
    if(m_lbutton)
    {
        m_centerY -= (float) (y-m_lastMouseY);
        m_centerX += (float) (x-m_lastMouseX);
        m_lastMouseX = x;
        m_lastMouseY = y;
    }
}

// mouse callback
void Mouse(int btn, int state, int x, int y)
{
    if(btn == GLUT_LEFT)
    {
        if(state == GLUT_UP)
        {
            m_lbutton = 0;
        }
        else if(state == GLUT_DOWN)
        {
            m_lbutton = 1;
            m_lastMouseX = x;
            m_lastMouseY = y;
        }
    }
    else if(btn == GLUT_WHEEL_UP)
    {
        m_glFov += 2.0f;
        if(m_glFov > 179.0f)
            m_glFov = 179.0f;
    }
    else if(btn == GLUT_WHEEL_DOWN)
    {
        m_glFov -= 2.0f;
        if(m_glFov < 1.0f)
            m_glFov = 1.0f;
    }
}

// register callbacks with GLUT
void RegisterCallbacks(void)
{
    glutDisplayFunc(Display_);
    glutIdleFunc(Display_);
    glutKeyboardFunc(Keyboard);
    glutReshapeFunc(Reshape);
    glutMotionFunc(MouseMotion);
    glutMouseFunc(Mouse);
    //glutCreateMenu(MouseMenu);
        //glutAddMenuEntry("quit",1);
    //glutAttachMenu(GLUT_RIGHT_BUTTON);
    return;
}
