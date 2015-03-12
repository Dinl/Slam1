#ifndef Room_H_
#define Room_H_

#include <stdio.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>

#include "alineadorCERES.h"
#include "FrameRGBD.h"
#include "SANN.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudPtrT;

class Room{
public:
	
	/********************************************************************************
	*	Constructor 1: Inicializar la nube
	*	
	********************************************************************************/
	Room(){
		//Inicializar las nubes
		PointCloudPtrT cloud_total (new PointCloudT);
		global_cloud = cloud_total;

		//Inicializar los parametros de la camara
		intrinseca = new double[6];
		intrinseca[0] = 535.501896;	//Fx
		intrinseca[1] = 537.504906;	//Fy
		intrinseca[2] = 330.201700;	//CenterX
		intrinseca[3] = 248.2017;	//CenterY
		intrinseca[4] = 0.119773;	//K1
		intrinseca[5] = -0.369037;	//K2
	}

	/********************************************************************************
	*	Destructor 1: 
	*	
	********************************************************************************/
	~Room(){


	}


	//Metodo de ICP propio de PCL
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//Metodo de ICP por features propio de PCL

	//Filtro
	pcl::VoxelGrid<PointT> voxelFilter;

	//Metodo de alineacion ICP
	bool alinearICP(FrameRGBD &Frame);

	//Metodo de alineacion SANN
	bool alinearCERES(FrameRGBD &Frame);

	//Metodos GET y SET
	virtual pcl::PointCloud<PointT>::ConstPtr getGlobalCloud();

private:
	//Nube que representa todo el ambiente
	PointCloudPtrT global_cloud, globalKeyCloud;
	cv::Mat descriptores_globales;
	std::vector<cv::KeyPoint> keypoints_globales;

	//Matriz que representa los valores de la camara
	double *intrinseca;
};

#endif
