#include "stdafx.h"
#include "Room.h"


/********************************************************************************
*	Metodo publico del Room que alinea el frame con el mapa global con el 
*	metodo ICP de PCL
*
*	TODO: Calcular mejor los parametros del ICP para que sea una comparacion justa
********************************************************************************/
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

/********************************************************************************
*	Metodo publico del Room que alinea el frame con el mapa global con el metodo
*	SANN propuesto
*
********************************************************************************/
bool Room::alinearCERES(FrameRGBD &Frame){
	
	//1. COPIAR EL MAPA GLOBAL
	//Si el mapa global no tiene datos, se toma el frame como mapa global junto 
	//con sus descriptores y keypoints
	//TODO: que la copia sea selectiva a espacios especificos
	PointCloudPtrT copy_global_cloud;
	PointCloudPtrT copy_global_key_cloud;
	PointCloudT Final;
	if(global_cloud->empty()){
		copy_global_cloud = Frame.getNube();
		copy_global_key_cloud = Frame.getKeyNube();
		Frame.getDescriptors().copyTo(descriptores_globales);
		keypoints_globales.swap(Frame.getKeypoints());
	}
	else{
		copy_global_cloud = global_cloud;
		copy_global_key_cloud = globalKeyCloud;
	}

	//2. DECLARAR VARIABLES PARA ALINEAR
	//Declaracion de variables de clasificador (Alternar entre SANN y FLANN para desarrollo)
	//SANN bestMatcher;
	cv::FlannBasedMatcher bestMatcher;

	std::vector<cv::DMatch> matches, matchesFilter;
	cv::Mat descriptores_frame;

	ceres::Problem problem3D;
	double *extrinseca;
	extrinseca = new double[6];
	extrinseca[0] = 0;
	extrinseca[1] = 0;
	extrinseca[2] = 0;
	extrinseca[3] = 0;
	extrinseca[4] = 0;
	extrinseca[5] = 0;

	//3. OBTENER NUBES, imagenes, KEYPOINTS Y DESCRIPTORES
	PointCloudPtrT frame_cloud = Frame.getNube();
	Frame.getDescriptors().copyTo(descriptores_frame);
	std::vector<cv::KeyPoint> keypoints_frame(Frame.getKeypoints());
	cv::Mat imagen_frame;
	Frame.getImagenRGB().copyTo(imagen_frame);

	//4. MATCH!
	//Todo: pasar a un metodo aparte que halle match
	//Hallar los descriptores similares
	bestMatcher.match(descriptores_globales, descriptores_frame,matchesFilter);

	//Halla min/max de las distancias correspondientes a los puntos match
	float min = 0, max = 0;
	for(int i=0; i < matchesFilter.size(); i++)
	if(matchesFilter[i].distance <= min)
		min = matchesFilter[i].distance;
	else if(matchesFilter[i].distance >= max)
		max = matchesFilter[i].distance;

	//Hallar el limite para el mejor % de distancias
	float limite = (max-min)*0.1 + min;
	for(int i=0; i < matchesFilter.size(); i++)
		if(matchesFilter[i].distance <= limite)
			matches.push_back(matchesFilter[i]);


	return true;
}

pcl::PointCloud<PointT>::ConstPtr Room:: getGlobalCloud(){
	return global_cloud;
}