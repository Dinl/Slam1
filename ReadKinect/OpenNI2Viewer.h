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
	//Metodos de callback que se invocan en cada frame via KINECT
	pcl::io::OpenNI2Grabber* openniGrabber;
	void cloud_Callback(const pcl::PointCloud<PointT>::ConstPtr& cloud);
	void image_callback (const boost::shared_ptr<pcl::io::Image>& image);
	void dephtImage_callback (const boost::shared_ptr<pcl::io::DepthImage>& image);
	//Visualizador
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer;


	//Metodos para iniciar y detener la lectura desde disco o KINECT
	void start();
	void stop();
	//Metodo de descarga y lectura del frame actual
	FrameRGBD download();
	bool read();

	//Constructores y destructor
	loader();
	loader(std::string path, bool KINECT);
	~loader();

	//Variables de estado
	bool isCloud, isImage, isDepth, isKinect, isFile, isFull, isRun;
	int filesSaved, filesReaded;
	std::string framepath;
	//Objetos que almacenan temporalmente en el cargador la nube global y las imagenes
	pcl::PointCloud<PointT>::ConstPtr global_cloud;
	cv::Mat global_rgbFrame,global_depthFrame;
	//Objeto FRAME global
	FrameRGBD* global_frame;
private:
	//Variable MUTEX para control de hilos
	boost::mutex mtx_;
};

//Constructor 1: Por defecto se abre el KINECT
loader::loader(){
	loader("", true);
}

//Constructor 2: Se selecciona entre el Kinect y el disco, con una ruta relativa PATH
loader::loader(std::string path, bool KINECT){

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


}

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

FrameRGBD loader::download(){
	mtx_.lock();

	//Crear el frame con el id consecutivo
	std::stringstream idStream;
	idStream << filesSaved++;
	FrameRGBD frame(idStream.str());
	//Copiar nube e imagen al frame
	frame.Nube = global_cloud;
	global_rgbFrame.copyTo(frame.Imagen);

	global_frame = &frame;

	mtx_.unlock();

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
		global_cloud = frame.Nube;
		isCloud = true;
		frame.Imagen.copyTo(global_rgbFrame);
		isImage = true;

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
	cv::cvtColor(tempImage, grayImage, cv::COLOR_RGB2GRAY);
	equalizeHist(grayImage, grayImageeq);

	#if GPU
		result = gpu_surf(grayImageeq);
	#endif

	if(!isImage){
		global_rgbFrame = grayImageeq;
		isImage = true;
	}
	mtx_.unlock();
}

void loader::dephtImage_callback (const boost::shared_ptr<pcl::io::DepthImage>& image){
	mtx_.lock();
	int altura = image->getHeight();
	int ancho = image->getWidth();
	
	cv::Mat bgrImage,grayImage,grayImageeq,result; 
	cv::Mat tempImage = cv::Mat(altura, ancho, CV_32FC1); 
	image->fillDepthImage(ancho,altura,(float*) tempImage.data,tempImage.step);
	tempImage.convertTo(grayImage,CV_8UC1);
	equalizeHist(grayImage, grayImageeq);
	#if GPU
		result = gpu_surf(grayImageeq);
	#endif

	cv::namedWindow("DepthImage", cv::WINDOW_AUTOSIZE );
	imshow("DepthImage",tempImage);
	cvWaitKey(3);
	mtx_.unlock();
}

#endif