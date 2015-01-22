// ReadKinect.cpp: define el punto de entrada de la aplicación de consola.
//
#define GPU 0

#include "stdafx.h"
#include "OpenNI2Viewer.h"
#include "FrameRGBD.h"

#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>

#include <boost/chrono.hpp>

#include "pcl/io/openni2/openni.h"


boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
pcl::io::OpenNI2Grabber* openniGrabber;                               // OpenNI grabber that takes data from the device.

unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(true), saveDepth(true), noColor(false);                                // Program control.
bool isCloud(false), isImage(false), isDepth(false);

#if GPU
cv::gpu::GpuMat G_rgbFrame;
#else
cv::Mat G_rgbFrame;
#endif

PointCloud<PointXYZRGBA>::ConstPtr global_cloud;
cv::Mat global_rgbFrame,global_depthFrame;

// This function is called every time the device has new data.
void cloud_Callback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud){
	if(!isCloud){
		global_cloud = cloud;
		isCloud = true;
	}
}

#if GPU
Mat gpu_surf(cv::Mat rgbFrame){
	G_rgbFrame.upload(rgbFrame);

	cv::gpu::GpuMat src_keypoints_gpu, src_descriptors_gpu;
	cv::gpu::SURF_GPU surf;

	surf(G_rgbFrame, cv::gpu::GpuMat(), src_keypoints_gpu, src_descriptors_gpu, false);

	std::vector<cv::KeyPoint> src_keypoints;
	std::vector<float> src_descriptors;
	std::vector<std::vector<cv::DMatch>> matches;

	surf.downloadKeypoints(src_keypoints_gpu, src_keypoints);
	surf.downloadDescriptors(src_descriptors_gpu, src_descriptors);

	Mat img_keypoints_1;
	drawKeypoints( rgbFrame, src_keypoints, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	return rgbFrame;
}
#endif

void image_callback (const boost::shared_ptr<pcl::io::Image>& image){

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



}

void dephtImage_callback (const boost::shared_ptr<pcl::io::DepthImage>& image){

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

	namedWindow("DepthImage", WINDOW_AUTOSIZE );
	imshow("DepthImage",tempImage);
	cvWaitKey(3);

}



// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>  createViewer(){
	boost::shared_ptr<visualization::CloudViewer> v (new visualization::CloudViewer("OpenNI viewer"));
	return (v);
}
 
int main(int argc, char** argv){

	G_rgbFrame.create(1, 1, CV_8U);
	

	openniGrabber = new pcl::io::OpenNI2Grabber();
	if (openniGrabber == 0)
		return -1;

	boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f_cloud = boost::bind(&cloud_Callback, _1);
	boost::function<void (const boost::shared_ptr<pcl::io::Image>&) > i_cloud = boost::bind (&image_callback, _1);
	boost::function<void (const boost::shared_ptr<pcl::io::DepthImage>&) > di_cloud = boost::bind (&dephtImage_callback, _1);

	openniGrabber->registerCallback(f_cloud);
	openniGrabber->registerCallback(i_cloud);
	openniGrabber->registerCallback(di_cloud);
 
	viewer = createViewer();

	openniGrabber->start();
	
	// Main loop.
	while (!viewer->wasStopped()){

		if(isCloud && isImage){
			//Crear el frame con el id consecutivo
			stringstream idStream;
			idStream << filesSaved++;
			FrameRGBD frame(idStream.str());
			//Copiar nube e imagen al frame
			frame.Nube = global_cloud;
			global_rgbFrame.copyTo(frame.Imagen);
			//Guardar el frame
			frame.guardar();
			//Falta el LEER

			//Mostrar la nube y copiarla en el frame			
			viewer->showCloud(global_cloud);			
			isCloud = false;

			//Mostrar la imagen y copiarla en el frame			
			namedWindow("Image", WINDOW_AUTOSIZE );
			imshow("Image",global_rgbFrame);
			cvWaitKey(3);			
			isImage = false;
		}
		//Dejar descansar el sistema 10 milisegundos
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
 
	openniGrabber->stop();
}
