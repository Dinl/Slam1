#ifndef Frame_RGBD
#define Frame_RGBD

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/console/print.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace pcl;

class FrameRGBD{
public:

	//Id del Frame
	string frameId;
	string frameName;
	string frameNubeName;
	string frameImagenName;

	//La nube de PCL con informacion de color
	PointCloud<PointXYZRGBA>::ConstPtr Nube;
	//Cuadro con la imagen RGB
	cv::Mat Imagen;


	//Constructor y destructor
	FrameRGBD(string id);
	~FrameRGBD();
	

	//Metodos
	void visualizar();
	void guardar();

private:
	

};

FrameRGBD::FrameRGBD(string id){
	FrameRGBD::frameId = id;
	frameName = "cuadro_"+frameId;
	frameNubeName = frameName + "_nube.pcd";
	frameImagenName = frameName + "_imagen.jpg";
}

FrameRGBD::~FrameRGBD(){

}

void FrameRGBD::visualizar(){
	//Mostrar primero la imagen
	cv::namedWindow( "Imagen", cv::WINDOW_AUTOSIZE );
	cv::imshow( "Display window", FrameRGBD::Imagen);
	cv::waitKey();

	//Mostrar segundo la nube
	boost::shared_ptr<visualization::CloudViewer> viewer (new visualization::CloudViewer("OpenNI viewer"));
	viewer->showCloud(FrameRGBD::Nube);

}

void FrameRGBD::guardar(){
	//Guardar la nube
	if (io::savePCDFile(frameNubeName, *Nube, true) != 0)
		PCL_ERROR("Problem saving %s.\n", frameNubeName.c_str());

	//Guardar la imagen
	if (cv::imwrite(frameImagenName,Imagen) == 0)
		PCL_ERROR("Problem saving %s.\n", frameImagenName.c_str());
}

#endif