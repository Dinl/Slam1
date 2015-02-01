#include <stdio.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include "FrameRGBD.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudPtrT;

class Room{
public:
	//Nube que representa todo el ambiente
	PointCloudPtrT global_cloud;

	//Metodo de ICP propio de PCL
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//Filtro
	pcl::VoxelGrid<PointT> voxelFilter;

	//Alinear 
	bool alinearICP(FrameRGBD &Frame);

	Room();
	~Room();
private:

};

//Constructor por defecto: 
Room::Room(){
	//Inicializar la nube
	PointCloudPtrT cloud_total (new PointCloudT);
	global_cloud = cloud_total;
}

//Destructor
Room::~Room(){

}

//Metodo que alinea el frame con el mapa global
bool Room::alinearICP(FrameRGBD &Frame){

	//Primero copiar el mapa global
	//TODO: que la copia sea selectiva a espacios especificos
	PointCloudPtrT copy_global_cloud;
	if(global_cloud->empty())
		copy_global_cloud = Frame.getNube();
	else
		copy_global_cloud = global_cloud;

	//Crear una nube que contendra temporalmente el resultado
	PointCloudT Final;

	//Configurar el ICP
	icp.setInputSource(Frame.getNube());
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