#include <stdio.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>

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

	//Alinear 
	bool alinear(FrameRGBD &Frame);

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
bool Room::alinear(FrameRGBD &Frame){

	//Primero copiar el mapa global
	//TODO: que la copia sea selectiva a espacios especificos
	PointCloudPtrT copy_global_cloud;
	if(global_cloud->empty())
		copy_global_cloud = Frame.Nube;
	else
		copy_global_cloud = global_cloud;

	//Crear una nube que contendra temporalmente el resultado
	PointCloudT Final;

	//Configurar el ICP
	icp.setInputSource(Frame.Nube);
	icp.setInputTarget(copy_global_cloud);
	//Realizar la alineacion
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//Si converge, entonces se copia el resultado a la nube original
	if(icp.hasConverged()){
		PointCloudT::ConstPtr deep_copy (new PointCloudT(Final));
		global_cloud = deep_copy;
	}

	return true;
}