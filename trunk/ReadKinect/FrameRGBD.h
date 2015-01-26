#ifndef Frame_RGBD
#define Frame_RGBD

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class FrameRGBD{
public:

	//Id del Frame
	std::string frameId;
	std::string frameName;
	std::string frameNubeName;
	std::string frameImagenName;

	//La nube de PCL con informacion de color
	pcl::PointCloud<PointT>::ConstPtr Nube;
	//Cuadro con la imagen RGB
	cv::Mat Imagen;


	//Constructor y destructor
	FrameRGBD(std::string id);
	FrameRGBD(std::string id, std::string path);
	~FrameRGBD();
	

	//Metodos
	void visualizar();
	void guardar();
	bool leer(std::string path);

private:
	

};

//Constructor 1: Solo con el identificador
FrameRGBD::FrameRGBD(std::string id){
	FrameRGBD::frameId = id;
	frameName = "cuadro_"+frameId;
	frameNubeName = frameName + "_nube.pcd";
	frameImagenName = frameName + "_imagen.jpg";
}

//Constructor 2: Identificador + Ruta relativa path
FrameRGBD::FrameRGBD(std::string id, std::string path){
	FrameRGBD::frameId = id;
	frameName = "cuadro_"+frameId;
	frameNubeName = path + frameName + "_nube.pcd";
	frameImagenName = path + frameName + "_imagen.jpg";
}

FrameRGBD::~FrameRGBD(){

}

//Metodo que visualiza el Frame
void FrameRGBD::visualizar(){
	//Mostrar primero la imagen
	cv::namedWindow( "Imagen", cv::WINDOW_AUTOSIZE );
	cv::imshow( "Display window", FrameRGBD::Imagen);
	cv::waitKey();

	//Mostrar segundo la nube
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("OpenNI viewer"));
	viewer->showCloud(FrameRGBD::Nube);

}

//Metodo que guarda el Frame en el disco
void FrameRGBD::guardar(){
	//Guardar la nube
	if (pcl::io::savePCDFile(frameNubeName, *Nube, true) != 0)
		PCL_ERROR("Problem saving %s.\n", frameNubeName.c_str());

	//Guardar la imagen
	if (cv::imwrite(frameImagenName,Imagen) == 0)
		PCL_ERROR("Problem saving %s.\n", frameImagenName.c_str());
}

//Metodo que lee el frame del disco
bool FrameRGBD::leer(std::string path){
	pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	bool error = 0;

	if(pcl::io::loadPCDFile<PointT>(frameNubeName,*tempCloud) != 0){
		PCL_ERROR("Problem reading %s.\n", frameNubeName.c_str());
		error = 1;
	}
	//Remover los NAN para evitar errores
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*tempCloud,*tempCloud, indices);
	Nube = tempCloud;

	Imagen = cv::imread(frameImagenName,CV_LOAD_IMAGE_COLOR);
	if(!Imagen.data){
		PCL_ERROR("Problem reading %s.\n", frameImagenName.c_str());
		error = 1;
	}

	return error;
}

#endif