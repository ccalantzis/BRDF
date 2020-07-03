//============================================================================
// Name        : brdfdata.cpp
// Author      : Christian Thurow
// Description : contains functions for the CBRDFdata class
//============================================================================

#if defined(_MSC_VER)
#include "stdafx.h"
#endif
#include "brdfdata.h"
#include <iostream>
#include <fstream>
#include "glut.h"
#include "levmar/levmar.h"
#include <igl/readOBJ.h>
#include <Eigen/Core>
#include <igl/barycentric_coordinates.h>
#include <fstream>
#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <igl/point_mesh_squared_distance.h>
#include <igl/voxel_grid.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/highgui/highgui.hpp>

bool CBRDFdata::LoadImages()
{
	m_images.clear();
	for(int i=0; i<m_numImages; i++)
	{
        string path = "img/timber/";
		char num[4];
		snprintf(num, sizeof(num), "%d", i+1);
		string extension = ".png";
		
		path += num;
		path += extension;

        //IplImage* newImg = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_COLOR);
        cv::Mat newImg = cv::imread(path);
        if(newImg.empty())
			return false;

		if(m_width <= -1 || m_height <= -1)
		{
            m_width = newImg.size().width;
            m_height = newImg.size().height;
		}

		m_images.push_back(newImg);
	}
	return true;
}

//puts out all input images onto the screen
void CBRDFdata::PrintImages()
{
	int i=0;
    for(vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++, i++)
    {
		string name = "image: ";
		char num[4];
		snprintf(num, sizeof(num), "%d", i+1);
		name += num;
        cv::namedWindow(name.c_str(), cv::WINDOW_AUTOSIZE);
        cv::imshow(name.c_str(), (*it));
		cv::waitKey(0);
    }
}

////faulty
//void CBRDFdata::NormaliseImages()
//{
//	for(vector<IplImage*>::iterator it = m_images.begin(); it != m_images.end(); it++)
//	{
//		//find darkest pixel in channel
//		int min_r = 0, min_g = 0, min_b = 0;
		
//		for(int x = 0; x < (*it)->width; x++)
//			for(int y = 0; y < (*it)->height; y++)
//			{
//				CvScalar pix = cvGet2D((*it), y, x);
//				if(pix.val[0] < min_r)
//					min_r = pix.val[0];
//				if(pix.val[1] < min_g)
//					min_g = pix.val[1];
//				if(pix.val[2] < min_b)
//					min_b = pix.val[2];
//			}


//		//subtract darkest pixel from all pixels in image
//		CvScalar dark;
//		dark.val[0] = min_r;
//		dark.val[1] = min_g;
//		dark.val[2] = min_b;

//		IplImage* darkImage;
//		CvSize size;
//		size.height = (*it)->height;
//		size.width = (*it)->width;
//		darkImage = cvCreateImage(size, (*it)->depth, 3);
//		cvSet(darkImage, dark);

//		cvSub((*it), darkImage, (*it));
//    }
//}

void CBRDFdata::PrintNormalisedImages()
{
	int i=0;
    for(vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++, i++)
    {
		string name = "normed image: ";
		char num[4];
		snprintf(num, sizeof(num)," %d", i+1);
		name += num;
        cv::namedWindow(name.c_str(), cv::WINDOW_AUTOSIZE);
        cv::imshow(name.c_str(), (*it));
		cv::waitKey(0);
	}
}

bool CBRDFdata::LoadDarkImage()
{
    string path = "img/timber/";
	string name = "dark";
	string extension = ".png";
		
	path += name;
	path += extension;

    m_dark = cv::imread(path);
    if(m_dark.dims == 0)
		return false;

	return true;
}

void CBRDFdata::SubtractAmbientLight()
{
    if(m_dark.empty())
		return;
	
    for(vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++)
    {
        cv::subtract((*it), m_dark, (*it));
	}
}

void CBRDFdata::LoadCameraParameters(std::string filename)
{
	std::vector<char> buffer;
	ReadInFile(filename, &buffer);

	int i=0;
	for(std::vector<char>::iterator it = buffer.begin(); it != buffer.end() && (*it != NULL); ++it)
	{
		char curr = *it;
		if(curr == '<')
		{
			it++;
			string currentParameter = "";
			for(; it != buffer.end() && (*it) != '>'; ++it) //read in parameter name
			{
				currentParameter += (*it);
			}
			string currentValue = "";
			if((*it) == '>') //parameter detected
				it++;

			//read in value
			for(; it != buffer.end() && (*it) != '<'; ++it)
			{
				currentValue += (*it);
			}
			if((*it) == '<') //end of value reached
				it++;

			WriteValue(currentParameter, currentValue);

			//go to end of current line
			for(; it != buffer.end() && (*it) != '>'; ++it) //read in parameter name
			{}
			if((*it) == '>') //detected end of line
				it++;
		}
	}
}

void CBRDFdata::WriteValue(std::string parameter, std::string value)
{
	double val = atof(value.c_str());

	if(parameter == "cx")
        m_principal_point.at<double>(0) = val;

	if(parameter == "cy")
        m_principal_point.at<double>(1) = val;

	if(parameter == "f")
		m_focalLength = val;

	if(parameter == "sx")
		m_pixelSizeX = val;

	if(parameter == "nx")
        m_n.at<double>(0) = val;

	if(parameter == "ny")
        m_n.at<double>(1) = val;

	if(parameter == "nz")
        m_n.at<double>(2) = val;

	if(parameter == "ox")
        m_o.at<double>(0) = val;

	if(parameter == "oy")
        m_o.at<double>(1) = val;

	if(parameter == "oz")
        m_o.at<double>(2) = val;

	if(parameter == "ax")
        m_a.at<double>(0) = val;

	if(parameter == "ay")
        m_a.at<double>(1) = val;

	if(parameter == "az")
        m_a.at<double>(2) = val;

	if(parameter == "px")
        m_p.at<double>(0) = val;

	if(parameter == "py")
        m_p.at<double>(1) = val;

	if(parameter == "pz")
        m_p.at<double>(2) = val;

}

//returns 0 if file could not be read, 1 otherwise
bool CBRDFdata::ReadInFile(std::string filename, std::vector<char>* buffer)
{
	buffer->clear();

	std::ifstream file;
	file.open(filename, std::ios::in | ios::binary);

	if(!file.fail())
	{ 
		while(!file.eof())
		{
			char currChar;
			file.get(currChar);
			buffer->push_back(currChar);
		}
		file.close();
    } 
	else
       return false;

	return true;
}

void CBRDFdata::ScaleMesh()
{
    Eigen::RowVector3d max_vertex = m_vertices.colwise().maxCoeff();
    Eigen::RowVector3d min_vertex = m_vertices.colwise().minCoeff();

    double diffX = max_vertex(0) - min_vertex(0);
    double diffY = max_vertex(1) - min_vertex(1);
    double diffZ = max_vertex(2) - min_vertex(2);

    //in the original code they used 10. But they weren't calculating the min and max values correctly.
    //the min might not be less than 0, guys. and on that note, the max might not be greater than 0.
    double scaleFactor = 8.0;

    m_vertices.col(0) /= diffX;
    m_vertices.col(1) /= diffY;
    m_vertices.col(2) /= diffZ;

    m_vertices.col(0) *= scaleFactor;
    m_vertices.col(1) *= scaleFactor;
    m_vertices.col(2) *= scaleFactor;
}

void CBRDFdata::LoadModel(std::string filename)
{
	//load 3d mesh from file
	//calc surface normals of object, if not already done..
	//give every surface a number, if they don't already have one(needed for later mapping with pixels) -> number of index
	//choose appropriate data structure, remember: we need to be able to store the BRDF infos as well for each surface/triangle

    igl::readOBJ(filename, m_vertices, m_faces);

    face_normals = CalcFaceNormals(m_vertices, m_faces);
    brdf_surfaces.resize(m_faces.rows(), 3);

    ScaleMesh();
}

Eigen::MatrixXd CBRDFdata::CalcFaceNormals(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F)
{
    Eigen::MatrixXd FN;
    FN.resize(F.rows(), 3);
    for (int rowCount_F=0; rowCount_F <F.rows(); ++rowCount_F)
    {
        int vertex1_index = F(rowCount_F,0);
        int vertex2_index = F(rowCount_F,1);
        int vertex3_index = F(rowCount_F,2);
        Eigen::RowVector3d vertex1 = V.block<1,3>(vertex1_index, 0);
        Eigen::RowVector3d vertex2 = V.block<1,3>(vertex2_index, 0);
        Eigen::RowVector3d vertex3 = V.block<1,3>(vertex3_index, 0);
        Eigen::RowVector3d edge1 = vertex2 - vertex1;
        Eigen::RowVector3d edge2 = vertex3 - vertex1;
        Eigen::RowVector3d face_normal = edge1.cross(edge2);
        face_normal.normalize();
        FN.row(rowCount_F) = face_normal;
    }
    return FN;
}

int CBRDFdata::GetNumFaces(vector<char*>* linesInFile)
{
	int numFaces = 0;

	//go to first occurance of f
	std::vector<char*>::iterator it;
	for(it = linesInFile->begin(); it != linesInFile->end() && (*it != NULL); it++)
	{
		if((*it)[0] == 'f')
			break;
	}
	std::string currLine = (*it);
	for(; it != linesInFile->end() && (*it != NULL); it++)
	{
		if((*it)[0] != 'f')
		{
			break;
		}
	}
	currLine = (*it);
	if((*it)[0] == '#')
	{
		std::string numVertices = "";
		for(int i=0; i<strlen(*it); i++)
		{
			if((*it)[i] > 47 && (*it)[i] < 58)
				numVertices += (*it)[i];
		}
		numFaces = atoi(numVertices.c_str());
	}

	return numFaces;
}

int CBRDFdata::GetNumVertices(vector<char*>* linesInFile)
{
	int numVerts = 0;
	//go to first occurance of v 
	std::vector<char*>::iterator it;
	for(it = linesInFile->begin(); it != linesInFile->end() && (*it != NULL); it++)
	{
		if((*it)[0] == 'v')
		{
			break;
		}
	}
	std::string currLine = (*it);
	for(; it != linesInFile->end() && (*it != NULL); it++)
	{
		if((*it)[0] != 'v')
		{
			break;
		}
	}
	currLine = (*it);
	if((*it)[0] == '#')
	{
		std::string numVertices = "";
		for(int i=0; i<strlen(*it); i++)
		{
			if((*it)[i] > 47 && (*it)[i] < 58)
				numVertices += (*it)[i];
		}
		numVerts = atoi(numVertices.c_str());
	}
	

	return numVerts;
}

void CBRDFdata::SaveValuesToSurface(int currentSurface, cv::Mat brdf, int colorChannel) //BGR
{
	//saves the brdf information stored in brdf to the corresponding surface on the model
	//attention: mind the color channel.. so we will need that information 3x at the surface..
	//brdf matrix contains parameters: kd, ks and n - in that order

    brdf_surfaces(currentSurface, colorChannel).kd = brdf.at<double>(0);
    brdf_surfaces(currentSurface, colorChannel).ks = brdf.at<double>(1);
    brdf_surfaces(currentSurface, colorChannel).n = brdf.at<double>(2);
}

cv::Mat CBRDFdata::GetCameraOrigin()
{
	return m_p;
}

cv::Mat CBRDFdata::GetA()
{
	return m_a;
}

cv::Mat CBRDFdata::GetO()
{
	return m_o;
}

cv::Mat CBRDFdata::GetN()
{
	return m_n;
}

double CBRDFdata::GetCX()
{
    return m_principal_point.at<double>(0);
}

double CBRDFdata::GetCY()
{
    return m_principal_point.at<double>(1);
}

cv::Mat CBRDFdata::CalcPixel2SurfaceMapping()
{
    cv::Mat map(m_height, m_width, CV_32S);
    map = cv::Scalar(0.0);

	//create map: pixel of image to surface on model(triangle?!) -> 0 means pixel not on model; val_x >0 means belongs to surface x
	//return matrix has size of input image and contains zeros for all pixels that don't contain the object of interest, otherwise the number to
	//the surface that pixel points
	//this map should be the same for all images, so we only have to calc it ones

    for(int i=0; i<m_faces.rows(); i++)
	{
		GLdouble winX = 0.0;
		GLdouble winY = 0.0;
		GLdouble winZ = 0.0;

		GLdouble objectX = 0.0;
		GLdouble objectY = 0.0;
		GLdouble objectZ = 0.0;
		
		for (int j = 0; j < 3; j++) //get center of current triangle
		{
            objectX += m_vertices(m_faces(i,j),0);
            objectY += m_vertices(m_faces(i,j),1);
            objectZ += m_vertices(m_faces(i,j),2);
		}
		
		objectX /= 3.0; objectY /= 3.0; objectZ /= 3.0;

		GLdouble model_view[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

		GLdouble projection[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projection);

		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		int errorValue = gluProject(objectX, objectY, objectZ, model_view, projection, viewport, &winX, &winY, &winZ);

		if(winY >=0 && winX >=0)
            map.at<int>(winY, winX) = i;
	}

	return map;
}

void CBRDFdata::InitLEDs()
{
	//we will need the absolute positions of the leds here hard coded!
	//we measured:
	//coordinate origin: x,y = 0; z = 11,5cm
	//led 1,2,3,4: radius from coordinate origin: 30,5cm, height of the leds from zero: 36,5cm
	//led 5,6,7,8: --------------------------------||---, height ------------------||-- 26,0cm
	//led 9,10,11,12: -----------------------------||---, height ------------------||-- 15,0cm
	//led 13,14,15,16: ----------------------------||---, height ------------------||--  4,5cm

    m_led.resize(m_numImages, 3);

	for (int i = 0; i < m_numImages; i++)
	{		
		//y coordinate
		switch (i / 4)
		{
			case 0: 
                m_led(i,1) = 365.0 - 115.0;
				break;
			case 1: 
                m_led(i,1) = 260.0 - 115.0;
				break;
			case 2: 
                m_led(i,1) = 150.0 - 115.0;
				break;
			default: 
                m_led(i,1) = 45.0 - 115.0;
				break;
		}
		
		//x+z coordinates
		switch (i % 4)
		{
		case 0:
            m_led(i,0) = 305.0 * sin(6.0/33.0*CV_PI*0.5);
            m_led(i,2) = 305.0 * cos(6.0/33.0*CV_PI*0.5);
			break;
		case 1:
            m_led(i,0) = 305.0 * sin(13.0/33.0*CV_PI*0.5);
            m_led(i,2) = 305.0 * cos(13.0/33.0*CV_PI*0.5);
			break;
		case 2:
            m_led(i,0) = 305.0 * sin(20.0/33.0*CV_PI*0.5);
            m_led(i,2) = 305.0 * cos(20.0/33.0*CV_PI*0.5);
			break;
		default:
            m_led(i,0) = 305.0 * sin(27.0/33.0*CV_PI*0.5);
            m_led(i,2) = 305.0 * cos(27.0/33.0*CV_PI*0.5);
			break;
		}
	}

	//for (int i = 0; i < m_numImages; i++)
	//{
    //	cout << i << ": x:" << m_led[i][0] << " y:" << m_led[i][1] << " z:" << m_led[i][2] << endl;
	//}
}

cv::Mat CBRDFdata::GetCosRV(int currentSurface)
{
    cv::Mat theta(1, m_numImages, CV_64F);
	//matrix theta contains angle for led1 in position 0 and angle for led16 in position 15(m_numImages-1)

	//TODO: check if the below code works!
	for (int i = 0; i < m_numImages; i++)
	{
        triangle surface;
        surface.row(0) = m_vertices.row(m_faces(currentSurface,0));
        surface.row(1) = m_vertices.row(m_faces(currentSurface,1));
        surface.row(2) = m_vertices.row(m_faces(currentSurface,2));
		
		//center of triangle
		double x = 0;
		double y = 0;
		double z = 0;
		
		for(int j = 0; j < 3; j++)
		{
            x += surface(j,0);
            y += surface(j,1);
            z += surface(j,2);
		}
		
		x /= 3.0; y /= 3.0; z /= 3.0;

		//observer vector
        double Vx = m_p.at<double>(0) - x;
        double Vy = m_p.at<double>(1) - y;
        double Vz = m_p.at<double>(2) - z;

		double length = sqrt(Vx*Vx + Vy*Vy + Vz*Vz);

		Vx /= length;
		Vy /= length;
		Vz /= length;

		//-1 * light vector, this orientation is needed for the formula below
        double Lx = x - m_led(i,0);
        double Ly = x - m_led(i,1);
        double Lz = x - m_led(i,2);
			
		length = sqrt(Lx*Lx + Ly*Ly + Lz*Lz);
			
		Lx /= length;
		Ly /= length;
		Lz /= length;
		
		//need reflexion vector, see http://wiki.delphigl.com/index.php/Reflexion
		//P = (N*L) * N
		//R = -2 * (N*L) * N + L = -2 * P + L

        double scale_factor = face_normals(currentSurface,0) * Lx + face_normals(currentSurface,1) * Ly + face_normals(currentSurface,2) * Lz;
        double Px = scale_factor * face_normals(currentSurface,0);
        double Py = scale_factor * face_normals(currentSurface,1);
        double Pz = scale_factor * face_normals(currentSurface,2);

		double Rx = Lx - 2*Px;
		double Ry = Ly - 2*Py;
		double Rz = Lz - 2*Pz;

		//reflexion vector * observer vector = theta
		//it actually return cosTheta! but thats ok, we only need cosTheta!
		double angle = Rx * Vx + Ry * Vy + Rz * Vz;
		//double actualAngle = acos(angle)*180.0/CV_PI;
        theta.at<double>(i) = angle;
	}

	return theta;
}

cv::Mat CBRDFdata::GetCosLN(int currentSurface)
{
    cv::Mat phi(1, m_numImages, CV_64F);
	//matrix phi contains angle for led1 in position 0 and angle for led16 in position 15(m_numImages-1)

	//TODO: check if the below code works!
	for (int i = 0; i < m_numImages; i++)
	{
        triangle surface;
        surface.row(0) = m_vertices.row(m_faces(currentSurface,0));
        surface.row(1) = m_vertices.row(m_faces(currentSurface,1));
        surface.row(2) = m_vertices.row(m_faces(currentSurface,2));
		
		//center of triangle
		double x = 0;
		double y = 0;
		double z = 0;
		
		for(int j = 0; j < 3; j++)
		{
            x += surface(j,0);
            y += surface(j,1);
            z += surface(j,2);
		}
		
		x /= 3.0; y /= 3.0; z /= 3.0;

		//light vector
        double Lx = m_led(i,0) - x;
        double Ly = m_led(i,1) - y;
        double Lz = m_led(i,2) - z;
			
		double length = sqrt(Lx*Lx + Ly*Ly + Lz*Lz);
			
		Lx /= length;
		Ly /= length;
		Lz /= length;
			
		//light vector * normal vector = phi
		//it actually return cosPhi! but thats ok, we only need cosPhi!
        double angle = Lx * face_normals(currentSurface,0) + Ly * face_normals(currentSurface,1) + Lz * face_normals(currentSurface,2);
        //double actualAngle = acos(angle)*180.0/CV_PI;
        phi.at<double>(i) = angle;
	}

	return phi;
}

//calcs the product of surface normal and H
cv::Mat CBRDFdata::GetCosNH(int currentSurface)
{
    cv::Mat theta_dash(1, m_numImages, CV_64F);
	//matrix theta_dash contains angle for led1 in position 0 and angle for led16 in position 15(m_numImages-1)

	for (int i = 0; i < m_numImages; i++)
	{
        triangle surface;
        surface.row(0) = m_vertices.row(m_faces(currentSurface,0));
        surface.row(1) = m_vertices.row(m_faces(currentSurface,1));
        surface.row(2) = m_vertices.row(m_faces(currentSurface,2));

		//center of triangle
		double x = 0;
		double y = 0;
		double z = 0;
		
		for(int j = 0; j < 3; j++)
		{
            x += surface(j,0);
            y += surface(j,1);
            z += surface(j,2);
		}
		
		x /= 3.0; y /= 3.0; z /= 3.0;

		//|light vector + observer vector| = half vector
        double Hx = m_led(i,0) - 2*x + m_p.at<double>(0);
        double Hy = m_led(i,1) - 2*y + m_p.at<double>(1);
        double Hz = m_led(i,2) - 2*z + m_p.at<double>(2);
			
		double length = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
			
		Hx /= length;
		Hy /= length;
		Hz /= length;
			
		//half vector * normal vector = theta
        //it actually return cosTheta'! but thats ok, we only need cosTheta'!
        double angle = Hx * face_normals(currentSurface,0) + Hy * face_normals(currentSurface,1) + Hz * face_normals(currentSurface,2);
        //double actualAngle = acos(angle)*180.0/CV_PI;
        theta_dash.at<double>(i) = angle;
	}

	return theta_dash;
}

cv::Mat CBRDFdata::GetIntensities(int x, int y, int colorChannel) //BGR
{
    cv::Mat I = cv::Mat(1, m_numImages, CV_64F);
	//matrix I contains image Intensities for iamge1 position 0 and intensity for image16 in position 15(m_numImages-1)
	//this function just gets the values of one color channel!
	int num = 0;
    for(vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++)
    {
        cv::Scalar i = (*it).at<double>((*it).size().height-1-y, x); //because ogl screen y-coordinate is inverted!
		double currIntensity = i.val[colorChannel]/255.0;
        I.at<double>(num++) = currIntensity;
	}
	return I;
}

struct extraData
{
	double* angles;
	int modelInfo;
};

/* model to be fitted to measurements */
void BRDFFunc(double *p, double *x, int m, int n, void *data)
{
	register int i;

	extraData* incommingData = (extraData*)data;
	double* angles = incommingData->angles;
	int model = incommingData->modelInfo;

	for(int i=0; i<n; i++)
	{
		double currCosPhi = angles[i];
		if(model == 0) //PHONG!
		{
			double currCosTheta = angles[i+n*2];
			x[i] = p[0]*currCosPhi + ((p[2] + 2.0)/2.0*CV_PI)*p[1]*(pow(currCosTheta,p[2]));
		}
		else if(model == 1) //BLINN-PHONG!
		{		
			double currCosThetaDash = angles[i+n];
			x[i] = p[0]*currCosPhi + p[1]*(pow(currCosThetaDash,p[2]));
		}
	}	
}

cv::Mat CBRDFdata::SolveEquation(cv::Mat phi, cv::Mat thetaDash, cv::Mat theta, cv::Mat I)
{
    cv::Mat brdf = cv::Mat(1, 3, CV_64F);
	//solve equation I = kd*cos(phi) + ks*cos^n(phi) with 16 sets of values
	//returns the resulting parameters kd, ks and n, in that order in brdf
	//phi: contains 16 values
	//I:   contains 16 values

	double p[3] = {0.5, 1.0, 1.0};
	//double p[3] = {1.1, 0.65, 0.2};
	double* x = new double[m_numImages];
	for(int i=0; i<m_numImages; i++)
	{
        x[i] = I.at<double>(i);
	}

	extraData* data = new extraData();

    data->angles = new double[phi.cols + thetaDash.cols + theta.cols];
	int j = 0;
	for(; j<m_numImages; j++)
	{
        data->angles[j] = phi.at<double>(j);
	}
	
	for(int a=0; j<m_numImages*2; j++, a++)
	{
        data->angles[j] = thetaDash.at<double>(a);
	}

	for(int a=0; j<m_numImages*3; j++, a++)
	{
        data->angles[j] = theta.at<double>(a);
	}
	
	data->modelInfo = m_model;

	int m = 3; //parameters
	int n = m_numImages; //measurements
	int itmax = 2000;
	double opts[LM_OPTS_SZ];
	double info[LM_INFO_SZ];
	/* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
	opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used

	int error = dlevmar_dif(BRDFFunc, p, x, m, n, itmax, opts, info, NULL, NULL, data);
	if(error == -1)
		cout << "Error in SolveEquation(..)" << endl;

//#ifdef _DEBUG
//
//	cout << "Levenberg-Marquardt returned in " << info[5] << "iter, reason " << info[6] << ", sumsq " << info[1] << "[" << info[0] << "g]" << endl;
//	cout << "Best fit parameters: "<< p[0] << ", " <<  p[1] << ", "  << p[2] << ", " << endl;
//
//#endif

    brdf.at<double>(0) = p[0];
    brdf.at<double>(1) = p[1];
    brdf.at<double>(2) = p[2];

	delete[] data->angles;
	delete data;

	return brdf;
}

void CBRDFdata::CalcBRDFEquation(cv::Mat pixelMap)
{
	int shizzle = 0;
	double min_kd = -100.0;
	double max_kd = 1000.0;
	double avg_kd = 0.0;
	double min_ks = -100.0;
	double max_ks = 1000.0;
	double avg_ks = 0.0;
	double min_n = -100.0;
	double max_n = 1000.0;
	double avg_n = 0.0;
	unsigned int count_kd = 0;
	unsigned int count_ks = 0;
	unsigned int count_n = 0;

	//for each pixel do:
	for(int x=0; x < m_width; x++)
		for(int y=0; y < m_height; y++)
		{
            int currentSurface = pixelMap.at<int>(y, x);

			shizzle++;

			if(currentSurface > 0) //pixel corresponds to a surface on the model
			{				
                cv::Mat phi(1, m_numImages, CV_64F);
                phi = GetCosLN(currentSurface);
                cv::Mat thetaDash(1, m_numImages, CV_64F);
                thetaDash = GetCosNH(currentSurface);
                cv::Mat theta(1, m_numImages, CV_64F);
                theta = GetCosRV(currentSurface);
				for(int colorChannel=0; colorChannel<3; colorChannel++) //do the calculation once for each color-channel
				{
					//build vector I
                    cv::Mat I = GetIntensities(x, y, colorChannel); //BGR
				
					//solve equation with 16 sets of values
                    cv::Mat brdf = SolveEquation(phi, thetaDash, theta, I);
				
					//when complete write values to surface in model
					SaveValuesToSurface(currentSurface, brdf, colorChannel); //BGR

                    if(brdf.at<double>(0) > 0 && brdf.at<double>(0) < 1000)
					{
                        avg_kd += brdf.at<double>(0);
						count_kd++;
					}
					
                    if(brdf.at<double>(1) > 0 && brdf.at<double>(1) < 1000)
					{
                        avg_ks += brdf.at<double>(1);
						count_ks++;
					}
				
                    if(brdf.at<double>(2) > 0 && brdf.at<double>(2) < 1000)
					{
                        avg_n += brdf.at<double>(2);
						count_n++;
					}
					
				}
			}

			//progress display
			double percent = (double)shizzle*100.0 / (double)(m_width*m_height);
			if ((int)shizzle % 100 == 0)
				cout << (int)percent << "% done\r";
		}

	cout << "100% done\n";

	//output statistics about brdf values:

	cout << /*"kd_min: " << min_kd << ", kd_max: " << max_kd << */", kd_avg: " << avg_kd/count_kd << endl;
	cout << /*"ks_min: " << min_ks << ", ks_max: " << max_ks << */", ks_avg: " << avg_ks/count_ks << endl;
	cout << /*"n_min: " << min_n << ", n_max: " << max_n  << */", n_avg: " << avg_n/count_n << endl;
}
