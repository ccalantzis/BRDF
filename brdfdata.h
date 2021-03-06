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

typedef Eigen::RowVector3d vertex;
typedef Eigen::Matrix<double,3,3> triangle;

struct brdfSurface
{
	double kd;
	double ks;
	double n;
};


class CBRDFdata
{
private:

	int m_numImages;
    std::vector<cv::Mat> m_images;
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

public:
	int m_model; //0: Phong, 1: Blinn-Phong
    Eigen::MatrixXi m_faces;
    Eigen::MatrixXd m_vertices;
    Eigen::MatrixXd face_normals;
    Eigen::MatrixXd vertex_normals;
    Eigen::Matrix<brdfSurface, Eigen::Dynamic, 3> brdf_surfaces;
    Eigen::MatrixXd m_led;
    Eigen::Matrix<brdfSurface, 1, 3> single_brdf;


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
	};

	//~CBRDFdata();

	bool Cleanup();
    bool LoadImages(std::string image_folder_path);
	void NormaliseImages();
	void PrintImages();
    void SubtractAmbientLight(std::string image_folder_path);
    bool LoadDarkImage(std::string image_folder_path);
    bool LoadCameraParameters(std::string filename);
	bool ReadInFile(std::string filename, std::vector<char>* buffer);
	void WriteValue(std::string parameter, std::string value);
    bool LoadModel(std::string filename);
    cv::Mat CalcPixel2SurfaceMapping();
    void CalcBRDFEquation_SingleBRDF(const cv::Mat &pixelMap);
    void CalcBRDFEquation(const cv::Mat &pixelMap);
    Eigen::RowVectorXd GetCosNH(int currentSurface);
    Eigen::RowVectorXd GetCosRV(int currentSurface);
    Eigen::RowVectorXd GetCosLN(int currentSurface);
    Eigen::RowVectorXd GetIntensities_FromPixel(int x, int y, int colorChannel);
    cv::Mat SolveEquation_SingleBRDF(const Eigen::MatrixXd &phi, const Eigen::MatrixXd &thetaDash, const Eigen::MatrixXd &theta, const Eigen::MatrixXd &I);
    cv::Mat SolveEquation(Eigen::RowVectorXd phi, Eigen::RowVectorXd thetaDash, Eigen::RowVectorXd theta, Eigen::RowVectorXd I);
    void SaveValuesToSurface(int currentSurface, cv::Mat brdf, int colorChannel);
	bool ReadInFileAsLines(std::string filename, std::vector<char*>* buffer);
    Eigen::MatrixXd CalcFaceNormals(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
    Eigen::MatrixXd CalcVertexNormals(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
    void ScaleMesh();
    cv::Mat GetCameraOrigin();
    cv::Mat GetA();
    cv::Mat GetO();
    cv::Mat GetN();
	double GetCX();
	double GetCY();
	void InitLEDs();
};
