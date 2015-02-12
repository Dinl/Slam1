#ifndef Loader_H
#define Loader_H

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
#include "opencv2/highgui/highgui.hpp"

#include "Loader.h"
#include "FrameRGBD.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class loader{
public:

	//Constructor 1: Por defecto se abre el KINECT
	loader(){
		loader("", true);
	}

	//Constructor 2: Se selecciona entre el Kinect y el disco, con una ruta relativa PATH
	loader(std::string path, bool KINECT){
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
	~loader(){

	}

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
	bool remember();
	bool see();

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



#endif