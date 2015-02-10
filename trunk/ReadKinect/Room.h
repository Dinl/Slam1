#include <stdio.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

//#include "FrameRGBD.hpp"

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

	//Metodos GET y SET
	virtual pcl::PointCloud<PointT>::ConstPtr getGlobalCloud();

private:
	//Nube que representa todo el ambiente
	PointCloudPtrT global_cloud;

};


//Metodo que alinea el frame con el mapa global con el metodo ICP de PCL
bool Room::alinearICP(FrameRGBD &Frame){

	//Primero copiar el mapa global
	//TODO: que la copia sea selectiva a espacios especificos
	PointCloudPtrT copy_global_cloud;
	if(global_cloud->empty())
		copy_global_cloud = Frame.getKeyNube();
	else
		copy_global_cloud = global_cloud;

	//Crear una nube que contendra temporalmente el resultado
	PointCloudT Final;

	//Configurar el ICP
	icp.setInputSource(Frame.getKeyNube());
	icp.setInputTarget(copy_global_cloud);
	icp.setMaximumIterations(10);
	icp.setMaxCorrespondenceDistance(0.05);
	//Realizar la alineacion
	icp.align(Final);

	
	//Si converge, entonces se copia el resultado a la nube original
	if(icp.hasConverged()){
		PointCloudT Merge = Final + *global_cloud;
		PointCloudT::ConstPtr MergePtr (new PointCloudT(Merge));
		PointCloudT::ConstPtr filtrado;

		voxelFilter.setInputCloud(MergePtr);
		voxelFilter.setLeafSize (0.01f, 0.01f, 0.01f);
		voxelFilter.filter(Final);

		PointCloudT::ConstPtr deep_copy (new PointCloudT(Final));

		global_cloud = deep_copy;

		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	}
	else{
		std::cout << "has NOT converged"  << std::endl;
	}

	return true;
}

pcl::PointCloud<PointT>::ConstPtr Room:: getGlobalCloud(){
	return global_cloud;
}