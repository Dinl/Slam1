#ifndef Room_H_
#define Room_H_

#include <stdio.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

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
		PointCloudPtrT cloud_total (new PointCloudT);
		global_cloud = cloud_total;
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
	bool alinearSANN(FrameRGBD &Frame);

	//Metodos GET y SET
	virtual pcl::PointCloud<PointT>::ConstPtr getGlobalCloud();

private:
	//Nube que representa todo el ambiente
	PointCloudPtrT global_cloud;
	cv::Mat descriptores_globales;

};

#endif
