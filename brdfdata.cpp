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

bool CBRDFdata::LoadImages(std::string image_folder_path)
{
	m_images.clear();
    std::string extension = ".png";

    for(int i=1; i<=m_numImages; i++)
	{
        std::string path = image_folder_path + std::to_string(i) + extension;

        cv::Mat newImg = cv::imread(path, cv::IMREAD_COLOR);
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
    for(std::vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++, i++)
    {
        std::string name = "image: ";
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
    for(std::vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++, i++)
    {
        std::string name = "normed image: ";
		char num[4];
		snprintf(num, sizeof(num)," %d", i+1);
		name += num;
        cv::namedWindow(name.c_str(), cv::WINDOW_AUTOSIZE);
        cv::imshow(name.c_str(), (*it));
		cv::waitKey(0);
	}
}

bool CBRDFdata::LoadDarkImage(std::string image_folder_path)
{
    std::string path = image_folder_path + "dark.png";

    m_dark = cv::imread(path);
    if(m_dark.dims == 0)
		return false;

	return true;
}

void CBRDFdata::SubtractAmbientLight()
{
    if(m_dark.empty())
		return;
	
    for(std::vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++)
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
            std::string currentParameter = "";
			for(; it != buffer.end() && (*it) != '>'; ++it) //read in parameter name
			{
				currentParameter += (*it);
			}
            std::string currentValue = "";
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
    file.open(filename, std::ios::in | std::ios::binary);

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

//checking that the specified point is inside the voxel grid
bool is_point_within_range(const Eigen::RowVector3d &point, const Eigen::RowVector3d &vg_min, const Eigen::RowVector3d &vg_max)
{
    for(int i = 0; i <3; ++i)
    {
        if(point(i) < vg_min(i) || point(i) > vg_max(i))
        {
            return false;
        }
    }
    return true;
}

Eigen::MatrixXd get_voxel_cubes(const Eigen::MatrixXd &cube_centres, const double &cube_side_length)
{
    Eigen::MatrixXd cube_V(cube_centres.rows()*2,3);
    double chsl = cube_side_length/2;
    for (int i = 0; i < cube_centres.rows(); ++i)
    {
        auto tcv = cube_centres.row(i); //centre of cube
        cube_V.row(2*i) << tcv(0) - chsl, tcv(1) - chsl, tcv(2) - chsl;
        cube_V.row(2*i + 1) << tcv(0) + chsl, tcv(1) + chsl, tcv(2) + chsl;
    }
    return cube_V;
}

int get_voxel_given_point(const Eigen::RowVector3d &point, const Eigen::RowVector3d &vg_min, const Eigen::RowVector3d &vg_max,
                          const Eigen::Matrix<int, 1, 3> &side_lengths, const double &cube_side_length)
{
    if(!is_point_within_range(point, vg_min, vg_max)) return -1;
    int sum1 = std::floor((std::abs((point(0))-vg_min(0)))/cube_side_length)+1;
    int sum2 = side_lengths(0)*(std::floor((std::abs(point(1)-vg_min(1)))/cube_side_length));
    int sum3 = side_lengths(0)*side_lengths(1)*(std::floor((std::abs(point(2)-vg_min(2)))/cube_side_length));
    int voxel = sum1 + sum2 + sum3;
    return voxel;
}

//this method takes a triangle and returns the voxels that the triangle is present in
std::vector<int> triangle_to_voxels(const Eigen::Matrix<double,3,3> &triangle, const Eigen::RowVector3d vg_min,
                                    const Eigen::RowVector3d vg_max, const Eigen::Matrix<int, 1, 3> &side_cubes,
                                    const double &cube_side_length)
{
    std::vector<int> cubes;
    //min and max coordinates of the triangle
    Eigen::RowVector3d min = triangle.colwise().minCoeff();
    Eigen::RowVector3d max = triangle.colwise().maxCoeff();
    Eigen::RowVector3d max_xy_min_z;
    max_xy_min_z << max(0), max(1), min(2);
    int min_voxel = get_voxel_given_point(min, vg_min, vg_max, side_cubes, cube_side_length);
    int max_voxel = get_voxel_given_point(max, vg_min, vg_max, side_cubes, cube_side_length);
    int max_xy_min_z_voxel = get_voxel_given_point(max_xy_min_z, vg_min, vg_max, side_cubes, cube_side_length);
    if(min_voxel == -1 || max_voxel == -1 || max_xy_min_z_voxel == -1)
    {
        std::cout << "triangle out of range\n";
        std::vector<int> empty;
        empty.push_back(-1);
        return empty;
    }
    if(min_voxel != max_voxel){
        int mult = std::floor((max_xy_min_z_voxel - min_voxel)/(side_cubes(0)));
        int add = (max_xy_min_z_voxel - min_voxel)%side_cubes(0);
        int z_mult = std::floor((max_voxel - min_voxel)/(side_cubes(0)*side_cubes(1)));
        for(int i = 0; i <= z_mult; ++i){
            for(int j = 0; j <= mult; ++j)
            {
                for(int k = 0; k <= add; ++k)
                {
                    cubes.push_back(min_voxel + j*side_cubes(0) + k + i*side_cubes(0)*side_cubes(1));
                }
            }
        }
    }
    else
    {
        cubes.push_back(min_voxel);
    }
    return cubes;
}

//A vector of vectors where each vector represents a voxel and the values inside represent the triangles inside that voxel
std::vector<std::vector<int>> voxel_to_triangle_mapping(const Eigen::MatrixXd &voxel_cubes, const Eigen::MatrixXd &V,
                                                   const Eigen::MatrixXi &F, const Eigen::Matrix<int, 1, 3> &side_cubes,
                                                   const double &cube_side_length)
{
    std::vector<std::vector<int>> mapping;
    Eigen::RowVector3d vg_min = voxel_cubes.colwise().minCoeff();
    Eigen::RowVector3d vg_max = voxel_cubes.colwise().maxCoeff();
    for(int i = 0; i < side_cubes.prod(); ++i)
    {
        std::vector<int> v;
        mapping.push_back(v);
    }

    for(int i = 0; i < F.rows(); ++i)
    {
        Eigen::Matrix<double,3,3> triangle;
        triangle << V.row(F.row(i)(0)), V.row(F.row(i)(1)), V.row(F.row(i)(2));
        std::vector<int> voxels = triangle_to_voxels(triangle, vg_min, vg_max, side_cubes, cube_side_length);
        for(int voxel : voxels)
        {
            mapping[voxel].push_back(i);
        }
    }
    return mapping;
}

//find the triangle that 'point' lies on, given the triangles in a voxel
int get_triangle_given_point(const Eigen::RowVector3d &point, const std::vector<std::vector<int>> &mapping,
                             const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXd &voxel_cubes,
                             const Eigen::Matrix<int, 1, 3> &side_lengths, const double &cube_side_length)
{
    Eigen::MatrixXd point_d;
    point_d = point;
    Eigen::RowVector3d vg_min = voxel_cubes.colwise().minCoeff();
    Eigen::RowVector3d vg_max = voxel_cubes.colwise().maxCoeff();
    int voxel = get_voxel_given_point(point, vg_min, vg_max, side_lengths, cube_side_length);
    if(voxel == -1) return 0;
    std::vector<int> triangles = mapping[voxel];
    for(int i = 0; i < triangles.size(); ++i)
    {
        int F_index = triangles[i];
        Eigen::MatrixXd bc;
        igl::barycentric_coordinates(point, V.row(F.row(F_index)(0)), V.row(F.row(F_index)(1)), V.row(F.row(F_index)(2)), bc);
        if(bc(0) >=0 && bc(1) >= 0 && bc(2) >= 0)
        {
            Eigen::VectorXd sqrD;
            Eigen::VectorXi I;
            Eigen::MatrixXd C;
            Eigen::Matrix<int,1,3> Fd;
            Fd = F.row(F_index);
            igl::point_mesh_squared_distance(point_d, V, Fd, sqrD, I, C);
            if(sqrD.sum() < 1e-2)
            {
                return F_index;
            }
        }
    }
}

//cv::Mat CBRDFdata::CalcPixel2SurfaceMapping()
//{
//    Eigen::MatrixXd GV;
//    Eigen::Matrix<int,1,3> sides;
//    int cubes_longest_side = 10;
//    Eigen::AlignedBox<double, 3> box(m_vertices.colwise().minCoeff(), m_vertices.colwise().maxCoeff());
//    igl::voxel_grid(box, cubes_longest_side, 0, GV, sides);
//    double csl = (GV.row(0)-GV.row(1)).norm(); //cube side length
//    std::cout << "csl: " << csl << '\n';
//    auto voxel_cubes = get_voxel_cubes(GV, csl);
//    std::vector<std::vector<int>> v_t_map = voxel_to_triangle_mapping(voxel_cubes, m_vertices, m_faces, sides, csl);
//    std::cout << "v_t_map size: " << v_t_map.size() << '\n';

//    Eigen::RowVector3d min_point = voxel_cubes.colwise().minCoeff();
//    Eigen::RowVector3d max_point = voxel_cubes.colwise().maxCoeff();
//    std::cout << "sides: " << sides << '\n';
//    std::cout << "min_point: " << min_point << '\n';
//    std::cout << "max_point: " << max_point << '\n';
//    min_point = m_vertices.colwise().minCoeff();
//    max_point = m_vertices.colwise().maxCoeff();
//    std::cout << "min_point: " << min_point << '\n';
//    std::cout << "max_point: " << max_point << '\n';

//    cv::Mat map(m_height, m_width, CV_32S);
//    map = cv::Scalar(0.0);

//    for(int x=0; x < m_width; x++)
//    {
//        for(int y=0; y < m_height; y++)
//        {
//            GLdouble winX = 0.0;
//            GLdouble winY = 0.0;
//            GLdouble winZ = 0.0;

//            GLdouble objX = 0.0;
//            GLdouble objY = 0.0;
//            GLdouble objZ = 0.0;

//            GLdouble model_view[16];
//            glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

//            GLdouble projection[16];
//            glGetDoublev(GL_PROJECTION_MATRIX, projection);

//            GLint viewport[4];
//            glGetIntegerv(GL_VIEWPORT, viewport);
////            GLdouble z;
////            glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_DOUBLE, &z);
//            //std::cout << "z: " << z << '\n';
////            double w = 1;
////            Eigen::RowVector4d view_point;
////            view_point << x,y,z,w;
//            //int _ = gluUnProject(win_point(0), win_point(1), win_point(2), model_view, projection, viewport, &objX, &objY, &objZ);
//            int _ = gluUnProject(x, y, 0.91, model_view, projection, viewport, &objX, &objY, &objZ);
//            Eigen::RowVector3d point;
//            point << objX, objY, objZ;
//            int triangle = get_triangle_given_point(point, v_t_map, m_vertices, m_faces, voxel_cubes, sides, csl);
////            map.at<int>(winY, winX) = triangle;
//            std::cout << "object: " << objX << ", " << objY << ", " << objZ << '\n';
//            std::cout << "window: " << x << ", " << y << ", " << 0.91 << '\n';
//            std::cout << "triangle: " << triangle << '\n';

////        GLdouble z;
////        glReadPixels (viewX, viewY, 1, 1, GL_DEPTH_COMPONENT, GL_DOUBLE, &z);
////        glm::vec4 viewport = glm::vec4(0, 0, m_width, m_height);
////        glm::vec3 wincoord = glm::vec3(x, m_height - y - 1, z);
////        glm::vec3 objcoord = glm::unProject(wincoord, view, projection, viewport);

////        int error = gluUnProject(x, y, z, model_view, projection, viewport, &objX, &objY, &objZ);

////        Eigen::RowVector3d point;
////        point << objcoord.x, objcoord.y, objcoord.z;
////        std::cout << point << '\n';
////        int triangle = get_triangle_given_point(point, v_t_map, V, F, voxel_cubes, sides, csl);
////        std::cout << triangle << '\n';
////        cvSetReal2D(map, y, x, triangle);
//        }
//    }
//}

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

        std::cout << "object: " << objectX << ", " << objectY << ", " << objectZ << '\n';
        std::cout << "window: " << winX << ", " << winY << ", " << winZ << '\n';

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
    for(std::vector<cv::Mat>::iterator it = m_images.begin(); it != m_images.end(); it++)
    {
        cv::Scalar i = (*it).at<cv::Scalar>((*it).size().height-1-y, x); //because ogl screen y-coordinate is inverted!
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
        std::cout << "Error in SolveEquation(..)" << '\n';

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
                cv::Mat phi = GetCosLN(currentSurface);
                cv::Mat thetaDash = GetCosNH(currentSurface);
                cv::Mat theta = GetCosRV(currentSurface);
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
//			double percent = (double)shizzle*100.0 / (double)(m_width*m_height);
//			if ((int)shizzle % 100 == 0)
//                std::cout << (int)percent << "% done\r";
		}

    std::cout << "100% done\n";

	//output statistics about brdf values:

    std::cout << /*"kd_min: " << min_kd << ", kd_max: " << max_kd << */", kd_avg: " << avg_kd/count_kd << '\n';
    std::cout << /*"ks_min: " << min_ks << ", ks_max: " << max_ks << */", ks_avg: " << avg_ks/count_ks << '\n';
    std::cout << /*"n_min: " << min_n << ", n_max: " << max_n  << */", n_avg: " << avg_n/count_n << '\n';
}
