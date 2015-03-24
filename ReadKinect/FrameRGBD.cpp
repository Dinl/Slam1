#include "stdafx.h"
#include "FrameRGBD.h"

/********************************************************************************
*	Metodo publico que visualiza el Frame
*
*	TODO: Actualizar con argumento para que muestre imagenes o nubes
********************************************************************************/
void FrameRGBD::visualizar(){
	//Mostrar primero la imagen
	cv::namedWindow( "Imagen", cv::WINDOW_AUTOSIZE );
	cv::imshow( "Display window", ImagenRGB);
	cv::waitKey();

	//Mostrar segundo la nube
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("OpenNI viewer"));
	viewer->showCloud(FrameRGBD::Nube);

}

/********************************************************************************
*	Metodo publico que guarda el Frame en el disco
*		Se guarda en el disco:
*		Nube PCD
*		Nube keypoints PCD
*		Imagen RGB 8UC3
*		Imagen RGB 8UC1
*		Imagen DEPTH 32F
*		Imagen DEPTH 8U
*		Keypoints vector
*		Descriptores
*
********************************************************************************/
void FrameRGBD::guardar(){
	//Crear la carpeta del frame
	std::string a = ""+ framePath;
	std::string folderCreateCommand;
	if(dirExists(a))
		folderCreateCommand = "mkdir " + framePath +"\\"+frameName;
	else
		folderCreateCommand = "mkdir -p " + framePath +"\\"+frameName;
	
	system(folderCreateCommand.c_str());

	//Guardar la nube
	if (pcl::io::savePCDFile(frameNubeName, *Nube, true) != 0)
		PCL_ERROR("Problem saving %s.\n", frameNubeName.c_str());

	//Guardar la nube de keypoints
	if (pcl::io::savePCDFile(frameKeyNubeName, *keypointNube, true) != 0)
		PCL_ERROR("Problem saving %s.\n", frameNubeName.c_str());

	//Guardar la imagen RGB
	if (cv::imwrite(frameImagenName,ImagenRGB) == 0)
		PCL_ERROR("Problem saving %s.\n", frameImagenName.c_str());

	//Guardar la imagen RGB Grises
	if (cv::imwrite(frameImagenGrisName,ImagenRGB_gray) == 0)
		PCL_ERROR("Problem saving %s.\n", frameImagenGrisName.c_str());

	//Guardar la imagen DEPTH 8U
	if (cv::imwrite(frameDepthn8Name,ImagenDEPTH_8U) == 0)
		PCL_ERROR("Problem saving %s.\n", frameDepthn8Name.c_str());

	//Guardar la imagen DEPTH 32F
	cv::FileStorage fs1(frameDepthn32Name, cv::FileStorage::WRITE);
	fs1 << "DEPTH32F" << ImagenDEPTH_32F;
	fs1.release();

	//Guardar los keypoints y los descriptores
	cv::FileStorage fs2(frameInfoName, cv::FileStorage::WRITE);
	cv::write(fs2, "keypoints", keypoints);
	fs2 << "descriptors" << descriptores;
	fs2.release();
}

/********************************************************************************
*	Metodo que lee el frame del disco
*		Se guarda en el disco:
*		Nube PCD
*		Nube keypoints PCD
*		Imagen RGB 8UC3
*		Imagen RGB 8UC1
*		Imagen DEPTH 32F
*		Imagen DEPTH 8U
*		Keypoints vector
*		Descriptores
*
********************************************************************************/
bool FrameRGBD::leer(std::string path){
	pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<PointT>::Ptr tempKeyCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	bool error = 0;

	//Cargar la nube de puntos
	if(pcl::io::loadPCDFile<PointT>(frameNubeName,*tempCloud) != 0){
		PCL_ERROR("Problem reading %s.\n", frameNubeName.c_str());
		error = 1;
	}
	Nube = tempCloud;

	//Cargar la nube de key puntos
	if(pcl::io::loadPCDFile<PointT>(frameKeyNubeName,*tempKeyCloud) != 0){
		PCL_ERROR("Problem reading %s.\n", frameNubeName.c_str());
		error = 1;
	}
	keypointNube = tempKeyCloud;

	//Cargar la imagen RGB 
	ImagenRGB = cv::imread(frameImagenName,CV_LOAD_IMAGE_COLOR);
	if(!ImagenRGB.data){
		PCL_ERROR("Problem reading %s.\n", frameImagenName.c_str());
		error = 1;
	}

	//Cargar la imagen RGB Grises
	ImagenRGB_gray = cv::imread(frameImagenGrisName,CV_LOAD_IMAGE_GRAYSCALE);
	if(!ImagenRGB_gray.data){
		PCL_ERROR("Problem reading %s.\n", frameImagenGrisName.c_str());
		error = 1;
	}
	
	//Cargar la imagen DEPTH 8U
	ImagenDEPTH_8U = cv::imread(frameDepthn8Name,CV_LOAD_IMAGE_GRAYSCALE);
	if(!ImagenDEPTH_8U.data){
		PCL_ERROR("Problem reading %s.\n", frameDepthn8Name.c_str());
		error = 1;
	}

	//Cargar la imagen DEPTH 32F
	cv::FileStorage fs1(frameDepthn32Name, cv::FileStorage::READ);
	fs1["DEPTH32F"] >> ImagenDEPTH_32F;
	fs1.release();

	//Cargar los keypoints y los descriptores
	cv::FileStorage fs2(frameInfoName, cv::FileStorage::READ);
	cv::FileNode fs2Node = fs2["keypoints"];
	read(fs2Node, keypoints);
	fs2["descriptors"] >> descriptores;
	fs2.release();

	return error;
}


/********************************************************************************
*	Metodo privado del FRAMERGBD para calcular los keyoints y descriptores SURF
*
*	TODO: Incluir el metodo GPU
********************************************************************************/
void FrameRGBD::calcularSURF(){
	//Crear el objeto SURF
	cv::SurfFeatureDetector detector(minHessian);

	//Calular los keypoints
	detector.detect( ImagenRGB_gray, keypoints_rgb);
	detector.detect( ImagenDEPTH_8U, keypoints_depth);

	//Pasar ambos keypoints al general
	for(int i=0; i<keypoints_rgb.size(); i++){
		int x = keypoints_rgb.at(i).pt.x;
		int y = keypoints_rgb.at(i).pt.y;
		int indice = rgb2cloud(x,y);

		float cx = Nube->at(x,y).x;
		float cy = Nube->at(x,y).y;
		float cz = Nube->at(x,y).z;

		if(cx==cx && cy==cy && cz==cz){
			pcl::PointXYZRGBA punto = Nube->at(x,y);
			keypointNube->push_back(punto);
			keypoints.push_back(keypoints_rgb[i]);
		}
	}

	for(int i=0; i<keypoints_depth.size(); i++){
		int x = keypoints_depth.at(i).pt.x;
		int y = keypoints_depth.at(i).pt.y;
		int indice = rgb2cloud(x,y);

		float cx = Nube->at(x,y).x;
		float cy = Nube->at(x,y).y;
		float cz = Nube->at(x,y).z;
		if(cx==cx && cy==cy && cz==cz){
			pcl::PointXYZRGBA punto = Nube->at(x,y);
			keypointNube->push_back(punto);
			keypoints.push_back(keypoints_depth[i]);
		}

	}

	std::cout << "Keypoints RGB: " << keypoints_rgb.size() << " Keypoints DEPTH: " << keypoints_depth.size() << "\n";

	//Calular los descriptores 2D
	extractor.compute(ImagenRGB_gray, keypoints, descriptores);

	std::cout << "Descriptors: " <<  descriptores.rows << "\n";
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la Nube
*	img	->	imagen RGB, CV_8UC3
*
*	TODO: Establecer el metodo para el calculo de SURF
********************************************************************************/
void FrameRGBD::setNube(pcl::PointCloud<PointT>::ConstPtr cloud){
	
	//Nube temporal
	pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	*tempCloud = *cloud;

	//Remover los NAN para evitar errores - Se deshabilita debido que al borrar los NAN se pierde
	//la referencia con los pixeles de la imagen original
	//std::vector<int> indices;
	//pcl::removeNaNFromPointCloud(*tempCloud,*tempCloud, indices);

	//Pasar la nube filtrada al objeto local
	Nube = tempCloud;

	//Nube temporal para los keypoints
	pcl::PointCloud<PointT>::Ptr tempKeyCloud (new pcl::PointCloud<PointT>);
	keypointNube = tempKeyCloud;

}

/********************************************************************************
*	Metodo privado del FRAMERGBD convertir un punto XY de la imagen
*	en el punto XYZ de la nube de puntos
*
********************************************************************************/
int FrameRGBD::rgb2cloud(int x, int y){
	int pcl_index = ((ImagenRGB.cols * (y-1)) + x);
	//std::cout << "(x,y,z) = " << Nube->at(pcl_index) << "\n";
	return pcl_index;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen RBG y una copia a gris
*	img	->	imagen RGB, CV_8UC3
*
********************************************************************************/
void FrameRGBD::setImagenRGB(cv::Mat &img){
	//Copiar la imagen
	img.copyTo(ImagenRGB);

	//Hallar la imagen equivalente en grises
	cv::cvtColor(ImagenRGB, ImagenRGB_gray, cv::COLOR_RGB2GRAY);
	equalizeHist(ImagenRGB_gray, ImagenRGB_gray);

	//Verificar si ya se ha cargado DEPTH y RGB, y calcular SURF
	if(ImagenDEPTH_8U.data && ImagenRGB_gray.data)
		calcularSURF();
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para establecer la imagen DEPTH 
*	img	->	imagen gris, CV_32FC1
*
********************************************************************************/
void FrameRGBD::setImagenDEPTH(cv::Mat &img){

	//Copiar la imagen a 32F y 8U
	img.copyTo(ImagenDEPTH_32F);
	img.convertTo(ImagenDEPTH_8U,CV_8UC1);

	//Ecualizar la imagen
	equalizeHist(ImagenDEPTH_8U, ImagenDEPTH_8U);

	//Verificar si ya se ha cargado DEPTH y RGB, y calcular SURF
	if(ImagenDEPTH_8U.data && ImagenRGB_gray.data)
		calcularSURF();

}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener la nube
*
********************************************************************************/
pcl::PointCloud<PointT>::ConstPtr FrameRGBD::getNube(){
	return Nube;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener la nube de keypoints
*
********************************************************************************/
pcl::PointCloud<PointT>::ConstPtr FrameRGBD::getKeyNube(){
	return keypointNube;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener la imagen RGB 
*
********************************************************************************/
cv::Mat FrameRGBD::getImagenRGB(){
	return ImagenRGB_gray;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener la imagen DEPTH 
*
********************************************************************************/
cv::Mat FrameRGBD::getImagenDEPTH(){
	return ImagenDEPTH_8U;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener los descriptores SURF
*
********************************************************************************/
cv::Mat FrameRGBD::getDescriptors(){
	return descriptores;
}

/********************************************************************************
*	Metodo publico del FRAMERGBD para obtener los keypoints SURF
*
********************************************************************************/
std::vector<cv::KeyPoint> FrameRGBD::getKeypoints(){
	return keypoints;
}

bool FrameRGBD::dirExists(const std::string& dirName_in){
  DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
  if (ftyp == INVALID_FILE_ATTRIBUTES)
    return false;  //something is wrong with your path!

  if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
    return true;   // this is a directory!

  return false;    // this is not a directory!
}