//============================================================================
// Name        : brdfdata.h
// Author      : Christian Thurow
// Description : defines the BRDF data structure
//============================================================================

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

using namespace std;
typedef Eigen::RowVector3d vertex;

struct brdfSurface
{
	double kd;
	double ks;
	double n;
};

struct triangle
{
	vertex m_point[3];
	vertex m_normal;
	int m_numFaces;
	brdfSurface brdf[3]; //BGR
};

class CBRDFdata
{
private:

	int m_numImages;
    vector<cv::Mat> m_images;
    int m_height; //resolution of the input images
	int m_width; //resolution of the input images
    cv::Mat m_dark;
    cv::Mat m_principal_point;
	double m_focalLength;
	double m_pixelSizeX;
    cv::Mat m_n;
    cv::Mat m_o;
    cv::Mat m_a;
    cv::Mat m_p;
    cv::Mat m_ledPositions;
	vertex* m_vertices;
	vertex* m_led;

public:
	int m_model; //0: Phong, 1: Blinn-Phong
    triangle* m_faces;

	CBRDFdata()
	{
		m_height = -1;
		m_width = -1;
		m_numImages = 16;
		m_images.clear();
        m_principal_point = cv::Mat(1, 2, CV_32F);
		m_focalLength = 0.0;
		m_pixelSizeX = 0.0;
        m_n = cv::Mat(1, 3, CV_32F); //betrag von n = 1!
        m_o = cv::Mat(1, 3, CV_32F); //betrag von o = 1!
        m_a = cv::Mat(1, 3, CV_32F); //betrag von a = 1!
        m_p = cv::Mat(1, 3, CV_32F);
		//o steht senkrecht auf n
		//o steht senkrecht auf a
		//n steht senkrecht auf a
        m_ledPositions = cv::Mat(16, 3, CV_32F);
//		m_vertices = NULL;
//		m_faces = NULL;
//		m_led = NULL;
	};

	//~CBRDFdata();

	bool Cleanup();
	bool LoadImages();
	void NormaliseImages();
	void PrintImages();
	void PrintNormalisedImages();
	void SubtractAmbientLight();
	bool LoadDarkImage();
	void LoadCameraParameters(std::string filename);
	bool ReadInFile(std::string filename, std::vector<char>* buffer);
	void WriteValue(std::string parameter, std::string value);
	void LoadModel(std::string filename);
    cv::Mat CalcPixel2SurfaceMapping();
    void CalcBRDFEquation(cv::Mat pixelMap);
    cv::Mat GetCosNH(int currentSurface);
    cv::Mat GetCosRV(int currentSurface);
    cv::Mat GetCosLN(int currentSurface);
    cv::Mat GetIntensities(int x, int y, int colorChannel);
    cv::Mat SolveEquation(cv::Mat phi, cv::Mat thetaDash, cv::Mat theta, cv::Mat I);
    void SaveValuesToSurface(int currentSurface, cv::Mat brdf, int colorChannel);
	bool ReadInFileAsLines(std::string filename, std::vector<char*>* buffer);
	int GetNumVertices(vector<char*>* linesInFile);
	void WriteVertices(vector<char*>* linesInFile);
	int GetNumFaces(vector<char*>* linesInFile);
	void WriteFaces(vector<char*>* linesInFile);
	void CalcFaceNormals();
	void ScaleMesh();
    cv::Mat GetCameraOrigin();
    cv::Mat GetA();
    cv::Mat GetO();
    cv::Mat GetN();
	double GetCX();
	double GetCY();
	void InitLEDs();
};
