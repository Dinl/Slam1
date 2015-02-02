#ifndef Loader
#define Loader
#define GPU 0

#include <stdio.h>
#include <iostream>
#include <tchar.h>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/nonfree/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "FrameRGBD.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class loader{
public:

	//Vinculador con el kinect
	pcl::io::OpenNI2Grabber* openniGrabber;

	//Visualizador de la nube
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer;

	//Objetos que almacenan temporalmente en el cargador la nube global y las imagenes
	pcl::PointCloud<PointT>::ConstPtr global_cloud;
	cv::Mat global_rgbFrame,global_depthFrame;

	//Objeto FRAME global
	FrameRGBD* global_frame;

	//Variables de estado
	bool isCloud, isImage, isDepth, isKinect, isFile, isFull, isRun;
	int filesSaved, filesReaded;

	//Metodos para iniciar y detener la lectura desde disco o KINECT
	//TODO: Verificar si realmente se necesitan
	void start();
	void stop();

	//Metodo de descarga y lectura del frame actual
	FrameRGBD download();
	bool read();

	//Constructores y destructor
	loader();
	loader(std::string path, bool KINECT);
	~loader();

private:
	//Metodos de callback que se invocan en cada frame via KINECT
	void cloud_Callback(const pcl::PointCloud<PointT>::ConstPtr& cloud);
	void image_callback (const boost::shared_ptr<pcl::io::Image>& image);
	void dephtImage_callback (const boost::shared_ptr<pcl::io::DepthImage>& image);

	//Variable MUTEX para control de hilos
	boost::mutex mtx_;

	//Variable con la ruta actual de frame
	std::string framepath;
};

//Constructor 1: Por defecto se abre el KINECT
loader::loader(){
	loader("", true);
}

//Constructor 2: Se selecciona entre el Kinect y el disco, con una ruta relativa PATH
loader::loader(std::string path, bool KINECT){

	//Inicializar variables
	isCloud = false;
	isImage = false;
	isDepth = false;
	isFile = false;
	isFull = false;
	isRun = false;
	filesSaved = 0;
	filesReaded = 0;
	framepath = path;

	boost::shared_ptr<pcl::visualization::CloudViewer> v (new pcl::visualization::CloudViewer("OpenNI viewer"));
	viewer = v;

	if(KINECT){
		openniGrabber = new pcl::io::OpenNI2Grabber();
		if (openniGrabber != 0){
			boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f_cloud = boost::bind(&loader::cloud_Callback, this, _1);
			boost::function<void (const boost::shared_ptr<pcl::io::Image>&) > i_cloud = boost::bind (&loader::image_callback, this, _1);
			boost::function<void (const boost::shared_ptr<pcl::io::DepthImage>&) > di_cloud = boost::bind (&loader::dephtImage_callback, this, _1);

			openniGrabber->registerCallback(f_cloud);
			openniGrabber->registerCallback(i_cloud);
			openniGrabber->registerCallback(di_cloud);

			//Inicializar variables
			isKinect = true;
		}
		else
			PCL_ERROR("Problem creating the openniGrabber \n");

	}
	else
		isKinect = false;





}

//Destructor
loader::~loader(){

}

void loader::start(){
	//Iniciar el grabador
	if(isKinect)
		openniGrabber->start();
	
	isRun = true;

}

void loader::stop(){
	isRun = false;
	//Detener el grabador
	if(isKinect)		
		openniGrabber->stop();

}

//Metodo que devuelve el FRAME actual
FrameRGBD loader::download(){
	//Bloquear el mutex mientras de crea el FRAME
	mtx_.lock();

		//Crear el frame con el id consecutivo
		std::stringstream idStream;
		idStream << filesSaved++;
		FrameRGBD frame(idStream.str(),framepath);

		//Copiar nube
		frame.setNube(global_cloud);
		//Copiar imagen RGB en el frame
		frame.setImagenRGB(global_rgbFrame);
		//Copiar imagen DEPTH
		frame.setImagenDEPTH(global_depthFrame);

		//Pasar el frame creado al global
		global_frame = &frame;

	//Desbloquear el mutex
	mtx_.unlock();

	//Devolver el FRAME creado en este metodo
	return *global_frame;
}

bool loader::read(){
	mtx_.lock();

	//Crear el frame con el id consecutivo
	std::stringstream idStream;
	idStream << filesReaded++;
	FrameRGBD frame(idStream.str(), framepath);

	//Si no hay error al leer, se carga la nueva imagen
	bool error = frame.leer(framepath);
	if(!error){
		global_cloud = frame.getNube();
		isCloud = true;

		frame.getImagenRGB().copyTo(global_rgbFrame);
		frame.getImagenDEPTH().copyTo(global_depthFrame);
		isImage = true;
		isDepth = true;

		global_frame = &frame;
	}

	mtx_.unlock();

	return error;

}

void loader::cloud_Callback(const pcl::PointCloud<PointT>::ConstPtr& cloud){
	mtx_.lock();
	if(!isCloud){
		global_cloud = cloud;
		isCloud = true;
	}
	mtx_.unlock();
}

void loader::image_callback (const boost::shared_ptr<pcl::io::Image>& image){
	mtx_.lock();
	int altura = image->getHeight();
	int ancho = image->getWidth();
	
	cv::Mat tempImage,bgrImage,grayImage,grayImageeq,result; 
	if (image->getEncoding() == pcl::io::Image::RGB)
		tempImage= cv::Mat(altura, ancho, CV_8UC3, const_cast<void *>(image->getData())); 
	else{
		tempImage = cv::Mat(altura, ancho, CV_8UC3); 
		image->fillRGB(image->getWidth(), image->getHeight(), tempImage.data,tempImage.step); 
	}

	//cv::cvtColor(tempImage, bgrImage, cv::COLOR_RGB2BGR);
	
	#if GPU
		result = gpu_surf(grayImageeq);
	#endif

	if(!isImage){
		global_rgbFrame = tempImage;
		isImage = true;
	}
	mtx_.unlock();
}

void loader::dephtImage_callback (const boost::shared_ptr<pcl::io::DepthImage>& image){
	mtx_.lock();
	int altura = image->getHeight();
	int ancho = image->getWidth();
	
	cv::Mat tempImage = cv::Mat(altura, ancho, CV_32FC1); 
	image->fillDepthImage(ancho,altura,(float*) tempImage.data,tempImage.step);
	
	
	#if GPU
		result = gpu_surf(grayImageeq);
	#endif

	if(!isDepth){
		global_depthFrame = tempImage;
		isDepth = true;
	}

	mtx_.unlock();
}

#endif