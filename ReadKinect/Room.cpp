#include "stdafx.h"
#include "Room.h"


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
		std::cout << "Matrix:" << icp.getFinalTransformation() << "\n\n";
	}
	else{
		std::cout << "has NOT converged"  << std::endl;
	}

	return true;
}

//Metodo que alinea el frame con el mapa global con el metodo SANN propuesto
bool Room::alinearSANN(FrameRGBD &Frame){
	SANN bestMatcher;

	return true;
}

pcl::PointCloud<PointT>::ConstPtr Room:: getGlobalCloud(){
	return global_cloud;
}