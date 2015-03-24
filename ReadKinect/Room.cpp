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
*	Metodo privado que filtra los match de acuerdo al angulo entre los keypoints
*	prefiriendo la tendencia
*
*	TODO: Optimizar
********************************************************************************/
void Room::anguloFilter(std::vector<cv::KeyPoint> keypoints_scene1, 
						std::vector<cv::KeyPoint> keypoints_scene2, 
						std::vector< cv::DMatch > &matchesFilter, 
						std::vector<cv::DMatch> &matches){
	//Hallar todos los angulos
	std::vector<float> angulos;
	for(int i=0; i < matchesFilter.size(); i++){
		//Obtener el punto en la imagen original
		double *P1;
		P1 = new double[2];
		P1[0] = keypoints_scene1[matchesFilter[i].queryIdx].pt.x;
		P1[1] = keypoints_scene1[matchesFilter[i].queryIdx].pt.y;

		//Obtener el punto en la imagen destino
		double *P2;
		P2 = new double[2];
		P2[0] = keypoints_scene2[matchesFilter[i].trainIdx].pt.x;
		P2[1] = keypoints_scene2[matchesFilter[i].trainIdx].pt.y;

		double dx = P2[0]-P1[0];
		double dy = P2[1]-P1[1];
		float tangente = dx != 0 ? std::abs(dy/dx) : 0;

		angulos.push_back(tangente);
	}

	//Hallar el promedio de todos los angulos
	float anguloPromedio = 0;
	for(int i=0; i < matchesFilter.size(); i++){
		anguloPromedio += std::abs(angulos[i]);
	}
	anguloPromedio = anguloPromedio/matchesFilter.size();

	//filtrar los que esten en la direccion del angulo correcto
	std::vector< cv::DMatch > matchesAngleFilter;
	for(int i=0; i < matchesFilter.size(); i++){
		if(std::abs(angulos[i]) < 0.07){
			matchesAngleFilter.push_back(matchesFilter[i]);
		}
	}

	matchesAngleFilter.swap(matches);
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
	PointCloudPtrT frame_key_cloud = Frame.getKeyNube();
	Frame.getDescriptors().copyTo(descriptores_frame);
	std::vector<cv::KeyPoint> keypoints_frame(Frame.getKeypoints());
	cv::Mat imagen_frame;
	Frame.getImagenRGB().copyTo(imagen_frame);

	//4. MATCH!
	//Todo: pasar a un metodo aparte que halle match
	//Hallar los descriptores similares
	bestMatcher.match(descriptores_globales, descriptores_frame,matchesFilter);

	//Seccion de filtros
	anguloFilter(keypoints_globales, keypoints_frame, matchesFilter, matches);


	//5. CREAR EL PROBLEMA 3D
	for(int i=0; i < matches.size(); i++){
		//Obtener el punto en la imagen original
		double *P1;
		P1 = new double[2];
		P1[0] = keypoints_globales[matches[i].queryIdx].pt.x;
		P1[1] = keypoints_globales[matches[i].queryIdx].pt.y;

		//Obtener el correspondiente punto en la nube
		double *C1;
		C1 = new double[3];
		C1[0] = copy_global_key_cloud->at(matches[i].queryIdx).x;
		C1[1] = copy_global_key_cloud->at(matches[i].queryIdx).y;
		C1[2] = copy_global_key_cloud->at(matches[i].queryIdx).z;

		
		/**********************************************************************/
		//Obtener el punto en el frame
		double *P2;
		P2 = new double[2];
		P2[0] = keypoints_frame[matches[i].trainIdx].pt.x;
		P2[1] = keypoints_frame[matches[i].trainIdx].pt.y;

		//Obtener el correspondiente punto en la nube
		double *C2;
		C2 = new double[3];
		C2[0] = frame_key_cloud->at(matches[i].trainIdx).x;
		C2[1] = frame_key_cloud->at(matches[i].trainIdx).y;
		C2[2] = frame_key_cloud->at(matches[i].trainIdx).z;

		if(C1[0] == C1[0] && C1[1] == C1[1] && C1[2] == C1[2] && C2[0] == C2[0] && C2[1] == C2[1] && C2[2] == C2[2]){
			ceres::CostFunction* cost_function = alineador3D::Create(C1[0], C1[1], C1[2]);
			ceres::LossFunction* lost_function = new ceres::HuberLoss(1.0);
			problem3D.AddResidualBlock(cost_function, lost_function, extrinseca, C2);
		}
	}
	ceres::Solver::Options options;
	options.max_num_iterations = 100;				//Por definir
	options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = false;
	options.function_tolerance = 1e-16;
	options.gradient_tolerance = 1e-32;
	options.parameter_tolerance = 1e-16;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem3D, &summary);
	std::cout << summary.FullReport() << "\n";

	//Crear las escenas finales
	pcl::PointCloud<PointT>::Ptr scene1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr scene2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr sceneT(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr sceneSum(new pcl::PointCloud<PointT>);

	//Realizar la alineacion
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	double R[9];
	ceres::AngleAxisToRotationMatrix(extrinseca,R);

	transform_1 (0,0) = R[0];
	transform_1 (0,1) = R[3];
	transform_1 (0,2) = R[6];
	transform_1 (0,3) = extrinseca[3];

	transform_1 (1,0) = R[1];
	transform_1 (1,1) = R[4];
	transform_1 (1,2) = R[7];
	transform_1 (1,3) = extrinseca[4];

	transform_1 (2,0) = R[2];
	transform_1 (2,1) = R[5];
	transform_1 (2,2) = R[8];
	transform_1 (2,3) = extrinseca[5];

	transform_1 (3,0) = 0;
	transform_1 (3,1) = 0;
	transform_1 (3,2) = 0;
	transform_1 (3,3) = 1;
	
	std::cout << transform_1 << "\n";

	pcl::transformPointCloud (*scene2, *sceneT, transform_1);

	*scene1 = *copy_global_cloud;
	*scene2 = *frame_cloud;
	*sceneSum = *scene1 + *sceneT;

	//Finalmente pasar la nube de copia a la nube global
	global_cloud = sceneSum;
	globalKeyCloud = copy_global_key_cloud;

	return true;
}

pcl::PointCloud<PointT>::ConstPtr Room:: getGlobalCloud(){
	return global_cloud;
}