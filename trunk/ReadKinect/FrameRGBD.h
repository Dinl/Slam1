#ifndef Frame_RGBD
#define Frame_RGBD

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class FrameRGBD{
public:

	//Id del Frame
	std::string frameId;
	std::string frameName;
	std::string frameNubeName;
	std::string frameImagenName;
	std::string frameDepthnName;

	//Descriptores y keypoints 2D
	cv::Mat descriptores;
	std::vector<cv::KeyPoint> keypoints;


	//Constructor y destructor
	FrameRGBD(std::string id);
	FrameRGBD(std::string id, std::string path);
	~FrameRGBD();
	

	//Metodos
	void calcularSURF();
	void rgb2cloud(int x, int y);
	void guardar();
	bool leer(std::string path);
	void visualizar();

	//Metodos de SET y GET
	void setNube(pcl::PointCloud<PointT>::ConstPtr cloud);
	void setImagenRGB(cv::Mat &img);
	void setImagenDEPTH(cv::Mat &img);
	pcl::PointCloud<PointT>::ConstPtr getNube();
	cv::Mat getImagenRGB();
	cv::Mat getImagenDEPTH();

private:
	//VARIABLES DEL FRAME
	//La nube de PCL con informacion de color
	pcl::PointCloud<PointT>::ConstPtr Nube;
	//Cuadro con la imagen RGB
	cv::Mat ImagenRGB;
	cv::Mat ImagenRGB_gray;
	cv::Mat ImagenDEPTH_32F;
	cv::Mat ImagenDEPTH_8U;

	//Variables para SURF
	int minHessian;
	cv::SurfDescriptorExtractor extractor;

};

/********************************************************************************
*	Constructor 1: Solo con el identificador
*	
*	TODO: Hacer que sea en carpeta
********************************************************************************/
FrameRGBD::FrameRGBD(std::string id){
	FrameRGBD::frameId = id;
	frameName = "cuadro_"+frameId;
	frameNubeName = frameName + "_nube.pcd";
	frameImagenName = frameName + "_imagen.jpg";
	frameDepthnName = frameName + "_depth.jpg";

	minHessian = 400;
}

//Constructor 2: Identificador + Ruta relativa path
FrameRGBD::FrameRGBD(std::string id, std::string path){
	FrameRGBD::frameId = id;
	frameName = "cuadro_"+frameId;
	frameNubeName = path + frameName + "/" + frameName + "_nube.pcd";
	frameImagenName = path + frameName + "/" + frameName + "_imagen.jpg";
	frameDepthnName = path + frameName + "/" + frameName + "_depth.jpg";

	minHessian = 400;
}

FrameRGBD::~FrameRGBD(){

}

//Metodo que visualiza el Frame
void FrameRGBD::visualizar(){
	//Mostrar primero la imagen
	cv::namedWindow( "Imagen", cv::WINDOW_AUTOSIZE );
	cv::imshow( "Display window", ImagenRGB);
	cv::waitKey();

	//Mostrar segundo la nube
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("OpenNI viewer"));
	viewer->showCloud(FrameRGBD::Nube);

}

//Metodo que guarda el Frame en el disco
void FrameRGBD::guardar(){
	//Guardar la nube
	if (pcl::io::savePCDFile(frameNubeName, *Nube, true) != 0)
		PCL_ERROR("Problem saving %s.\n", frameNubeName.c_str());

	//Guardar la imagen RGB
	if (cv::imwrite(frameImagenName,ImagenRGB) == 0)
		PCL_ERROR("Problem saving %s.\n", frameImagenName.c_str());

	//Guardar la imagen RGB
	if (cv::imwrite(frameDepthnName,ImagenDEPTH_8U) == 0)
		PCL_ERROR("Problem saving %s.\n", frameDepthnName.c_str());
}

//Metodo que lee el frame del disco
bool FrameRGBD::leer(std::string path){
	pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	bool error = 0;

	//Cargar la nube de puntos
	if(pcl::io::loadPCDFile<PointT>(frameNubeName,*tempCloud) != 0){
		PCL_ERROR("Problem reading %s.\n", frameNubeName.c_str());
		error = 1;
	}
	//Remover los NAN para evitar errores
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*tempCloud,*tempCloud, indices);
	Nube = tempCloud;

	//Cargar la imagen RGB
	ImagenRGB = cv::imread(frameImagenName,CV_LOAD_IMAGE_COLOR);
	if(!ImagenRGB.data){
		PCL_ERROR("Problem reading %s.\n", frameImagenName.c_str());
		error = 1;
	}

	//Cargar la imagen DEPTH
	ImagenDEPTH_8U = cv::imread(frameDepthnName,CV_LOAD_IMAGE_COLOR);
	if(!ImagenDEPTH_8U.data){
		PCL_ERROR("Problem reading %s.\n", frameDepthnName.c_str());
		error = 1;
	}

	return error;
}

void FrameRGBD::calcularSURF(){
	//Crear el objeto SURF
	cv::SurfFeatureDetector detector(minHessian);

	//Calular los keypoints
	detector.detect( ImagenRGB, keypoints );

	//Calular los descriptores 2D
	extractor.compute( ImagenRGB, keypoints, descriptores);
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la Nube
*	img	->	imagen RGB, CV_8UC3
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
void FrameRGBD::setNube(pcl::PointCloud<PointT>::ConstPtr cloud){
	Nube = cloud;
}

void FrameRGBD::rgb2cloud(int x, int y){
	int pcl_index, rgb_h, rgb_w;
	rgb_h = 240;
	rgb_w = 320;
	pcl_index = ((rgb_w* ImagenRGB.rows) + rgb_h);
	std::cout << "(x,y,z) = " << Nube->at(pcl_index) << "\n";
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen RBG y una copia a gris
*	img	->	imagen RGB, CV_8UC3
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
void FrameRGBD::setImagenRGB(cv::Mat &img){
	//Copiar la imagen
	img.copyTo(ImagenRGB);

	//Hallar la imagen equivalente en grises
	cv::cvtColor(ImagenRGB, ImagenRGB_gray, cv::COLOR_RGB2GRAY);
	equalizeHist(ImagenRGB_gray, ImagenRGB_gray);
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen DEPTH 
*	img	->	imagen gris, CV_32FC1
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
void FrameRGBD::setImagenDEPTH(cv::Mat &img){

	//Copiar la imagen a 32F y 8U
	img.copyTo(ImagenDEPTH_32F);
	img.convertTo(ImagenDEPTH_8U,CV_8UC1);

	//Ecualizar la imagen
	equalizeHist(ImagenDEPTH_8U, ImagenDEPTH_8U);
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la nube
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
pcl::PointCloud<PointT>::ConstPtr FrameRGBD::getNube(){
	return Nube;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen DEPTH 
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
cv::Mat FrameRGBD::getImagenRGB(){
	return ImagenRGB_gray;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen DEPTH 
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
cv::Mat FrameRGBD::getImagenDEPTH(){
	return ImagenDEPTH_8U;
}

#endif