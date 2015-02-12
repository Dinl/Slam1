#include "stdafx.h"
#include "Loader.h"

/********************************************************************************
*	Metodo publico del LOADER para inciar la captura, sea desde Kinect o desde disco
*
*	TODO: Considerar deprecar
********************************************************************************/
void loader::start(){
	//Iniciar el grabador
	if(isKinect)
		openniGrabber->start();
	
	isRun = true;

}

/********************************************************************************
*	Metodo publico del LOADER para detener la captura, sea desde Kinect o desde disco
*
*	TODO: Considerar deprecar
********************************************************************************/
void loader::stop(){
	isRun = false;
	//Detener el grabador
	if(isKinect)		
		openniGrabber->stop();

}

/********************************************************************************
*	Metodo publico del LOADER que devuelve el FRAME actual
*
*	TODO: Toca modificar para que solo devuelva el FRAME, pasar contenido a otra funcion
********************************************************************************/
FrameRGBD loader::download(){
	return *global_frame;
}

/********************************************************************************
*	Metodo publico del LOADER que cree frames desde el kinect
*
*	TODO: Se lee por consecutivo, mejorar esto
			crear la condicion false
********************************************************************************/
bool loader::see(){
	//Bloquear el mutex mientras de crea el FRAME
	mtx_.lock();

		//Crear el frame con el id consecutivo
		std::stringstream idStream;
		idStream << filesSaved++;
		FrameRGBD *frame = new FrameRGBD(idStream.str(),framepath);

		//Copiar nube
		frame->setNube(global_cloud);
		//Copiar imagen RGB en el frame
		frame->setImagenRGB(global_rgbFrame);
		//Copiar imagen DEPTH
		frame->setImagenDEPTH(global_depthFrame);

		//Pasar el frame creado al global
		global_frame = frame;

	//Desbloquear el mutex
	mtx_.unlock();

	return true;
}

/********************************************************************************
*	Metodo publico del LOADER que lee desde disco los cuadros
*
*	TODO: Se lee por consecutivo, mejorar esto
			Cambiar el nombre de la funcion a remember
********************************************************************************/
bool loader::remember(){
	mtx_.lock();

	//Crear el frame con el id consecutivo
	std::stringstream idStream;
	idStream << filesReaded++;
	FrameRGBD *frame = new FrameRGBD(idStream.str(),framepath);

	//Si no hay error al leer, se carga la nueva imagen
	bool error = frame->leer(framepath);
	if(!error){
		global_cloud = frame->getNube();
		isCloud = true;

		frame->getImagenRGB().copyTo(global_rgbFrame);
		frame->getImagenDEPTH().copyTo(global_depthFrame);
		isImage = true;
		isDepth = true;

		global_frame = frame;
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
	cv::Mat tempImageROI;
	cv::Mat tempImageROIResize = cv::Mat(altura, ancho, CV_32FC1);
	image->fillDepthImage(ancho,altura,(float*) tempImage.data,tempImage.step);
	
	if(!isDepth){
		global_depthFrame = tempImage;
		isDepth = true;
	}

	mtx_.unlock();
}