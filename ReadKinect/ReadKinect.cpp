// ReadKinect.cpp: define el punto de entrada de la aplicación de consola.
//

#include "stdafx.h"
#include "OpenNI2Viewer.h"
#include "Room.h"

bool KINECT = false;
loader cargador("grabacion2/", KINECT);
Room habitacion;


//Hilo para obtener los frames y seguir con el proceso
void obtenerFrames(){
	if(KINECT){
		while (!cargador.viewer->wasStopped())
			if(cargador.isCloud && cargador.isImage){
				FrameRGBD frame = cargador.download();
				//habitacion.alinear(frame);
				frame.guardar();
			}
	}
	else{
		while (!cargador.viewer->wasStopped())
			if(!cargador.isCloud && !cargador.isImage){
				//Leer el caargador y verificar lectura
				bool error = cargador.read();
			
				//Si no hay error, continuar con el proceso, sino salir
				if(!error){
					//Descargar el frame actual para procesar
					FrameRGBD frame = cargador.download();
					habitacion.alinearICP(frame);
				}				
				else{
					PCL_WARN("Fin de la lectura!");
					break;
				}
			}
	}
	
}

//Hilo para mantener vivo el openNI
void loopKinect(){
	while (!cargador.viewer->wasStopped()){
		if(cargador.isCloud && cargador.isImage){
			//Mostrar la nube y copiarla en el frame			
			//cargador.viewer->showCloud(cargador.global_cloud);
			cargador.viewer->showCloud(habitacion.global_cloud);	
			cargador.isCloud = false;

			//Mostrar la imagen y copiarla en el frame			
			cv::namedWindow("Image", cv::WINDOW_AUTOSIZE );
			imshow("Image",cargador.global_rgbFrame);
			cvWaitKey(1);			
			cargador.isImage = false;
		}
	}
}

int main(int argc, char** argv){

	//Iniciar el cargador
	cargador.start();
	
	//Crear 2 hilos, para obtener imagenes y para mostrar
	boost::thread* loopHilo = new boost::thread(loopKinect);
	boost::thread* obtenerHilo = new boost::thread(obtenerFrames);
	loopHilo->join();
	obtenerHilo->join();

	//Detener el cargador
	cargador.stop();

	return 0;
}
