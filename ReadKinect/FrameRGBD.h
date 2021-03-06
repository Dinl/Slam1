#ifndef Frame_RGBD_H
#define Frame_RGBD_H

#define GPU 0

#include "stdafx.h"
#define NOMINMAX
#include <windows.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#if GPU
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>
#endif

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class FrameRGBD{
public:

	//Id del Frame
	std::string frameId;
	std::string frameName;
	std::string framePath;
	std::string frameNubeName;
	std::string frameKeyNubeName;
	std::string frameImagenName;
	std::string frameImagenGrisName;
	std::string frameDepthn8Name;
	std::string frameDepthn32Name;
	std::string frameInfoName;

	/********************************************************************************
	*	Constructor 1: Solo con el identificador
	*	
	*	TODO: Hacer que sea en carpeta
	********************************************************************************/
	FrameRGBD(std::string id){
		frameId = id;
		framePath = "";
		frameName = "cuadro_"+frameId;
		frameNubeName = frameName + "/" + frameName + "_nube.pcd";
		frameKeyNubeName = frameName + "/" + frameName + "_keyNube.pcd";
		frameImagenName = frameName + "/" + frameName + "_imagen.jpg";
		frameImagenGrisName = frameName + "/" + frameName + "_imagen_grises.jpg";
		frameDepthn8Name = frameName + "/" + frameName + "_depth8.jpg";
		frameDepthn32Name = frameName + "/" + frameName + "_depth32.xml";
		frameInfoName = frameName + "/" + frameName + "_info.xml";

		minHessian = 400;
	}
	/********************************************************************************
	*	Constructor 1: Identificador + Ruta relativa path
	*	
	*	
	********************************************************************************/
	FrameRGBD(std::string id, std::string path){
		frameId = id;
		framePath = path;
		frameName = "cuadro_"+frameId;
		frameNubeName = path + "/" + frameName + "/" + frameName + "_nube.pcd";
		frameKeyNubeName = path + "/" + frameName + "/" + frameName + "_keyNube.pcd";
		frameImagenName = path + "/" + frameName + "/" + frameName + "_imagen.jpg";
		frameImagenGrisName = path + "/" + frameName + "/" + frameName + "_imagen_grises.jpg";
		frameDepthn8Name = path + "/" + frameName + "/" + frameName + "_depth8.jpg";
		frameDepthn32Name = path + "/" + frameName + "/" + frameName + "_depth32.xml";
		frameInfoName = path + "/" + frameName + "/" + frameName + "_info.xml";

		minHessian = 400;
	}

	/********************************************************************************
	*	Destructor
	*	
	********************************************************************************/
	~FrameRGBD(){

	}
	

	//Metodos
	virtual void guardar();
	virtual bool leer(std::string path);
	virtual void visualizar();

	//Metodos de SET y GET
	virtual void setNube(pcl::PointCloud<PointT>::ConstPtr cloud);
	virtual void setImagenRGB(cv::Mat &img);
	virtual void setImagenDEPTH(cv::Mat &img);
	virtual pcl::PointCloud<PointT>::ConstPtr getNube();
	virtual pcl::PointCloud<PointT>::ConstPtr getKeyNube();
	virtual cv::Mat getImagenRGB();
	virtual cv::Mat getImagenDEPTH();
	virtual cv::Mat getDescriptors();
	virtual std::vector<cv::KeyPoint> getKeypoints();

private:
	//VARIABLES DEL FRAME
	//La nube de PCL con informacion de color
	pcl::PointCloud<PointT>::ConstPtr Nube;
	pcl::PointCloud<PointT>::Ptr keypointNube;
	//Cuadro con la imagen RGB
	cv::Mat ImagenRGB;
	cv::Mat ImagenDEPTH_32F;
	cv::Mat ImagenRGB_gray;
	cv::Mat ImagenDEPTH_8U;

	//Si esta habilitado, se crean GPUMAT
	#if GPU
		cv::gpu::GpuMat G_matRGB;
		cv::gpu::GpuMat G_matDEPTH;
	#endif
	
	//Variables para SURF
	int minHessian;
	#if GPU
		cv::gpu::SURF_GPU extractor;
	#else
		cv::SurfDescriptorExtractor extractor;
	#endif

	//Descriptores y keypoints 2D
	cv::Mat descriptores;
	cv::vector<cv::KeyPoint> keypoints_rgb, keypoints_depth, keypoints;

	//Metodos privados
	virtual bool dirExists(const std::string& dirName_in);
	virtual void calcularSURF();
	virtual int rgb2cloud(int x, int y);
};

#endif