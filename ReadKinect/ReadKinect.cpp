// ReadKinect.cpp: define el punto de entrada de la aplicación de consola.
//

#include "stdafx.h"
#include "Room.h"
#include "Loader.h"

bool KINECT = false;
loader cargador("grabacion1", KINECT);
Room habitacion;

/********************************************************************************
*	Hilo para obtener los frames y seguir con el proceso
*
********************************************************************************/
void obtenerFrames(){
	//Si se va a usar el Kinect entonces usar la funcion see, en caso contrario se
	//usa remember
	if(KINECT){
		while (!cargador.viewer->wasStopped())
			if(cargador.isCloud && cargador.isImage && cargador.isDepth){
				cargador.see();
				FrameRGBD frame = cargador.download();
				//habitacion.alinear(frame);
				frame.guardar();
			}
	}
	else{
		while (!cargador.viewer->wasStopped())
			if(!cargador.isCloud && !cargador.isImage){
				//Leer el caargador y verificar lectura
				bool error = cargador.remember();
			
				//Si no hay error, continuar con el proceso, sino salir
				if(!error){
					//Descargar el frame actual para procesar
					FrameRGBD frame = cargador.download();
					//habitacion.alinearICP(frame);

					habitacion.alinearCERES(frame);
				}				
				else{
					PCL_WARN("Fin de la lectura!");
					break;
				}
			}
	}
	
}

/********************************************************************************
*	Hilo para mantener vivo el openNI
*
********************************************************************************/
void loopKinect(){
	while (!cargador.viewer->wasStopped()){
		if(cargador.isCloud && cargador.isImage){
			//Mostrar la nube y copiarla en el frame	
			if(KINECT)
				cargador.viewer->showCloud(cargador.global_cloud);
			else
				cargador.viewer->showCloud(habitacion.getGlobalCloud());
			//cargador.viewer->showCloud(habitacion.global_cloud);	
			cargador.isCloud = false;

			//Mostrar la imagen y copiarla en el frame			
			cv::namedWindow("Image", cv::WINDOW_AUTOSIZE );
			imshow("Image",cargador.global_rgbFrame);
			cvWaitKey(1);			
			cargador.isImage = false;
		}
	}
}

/********************************************************************************
*	MAIN
*
*	No recibe ningun argumento!
********************************************************************************/
int main(int argc, char** argv){

	//Verificar si se usa GPU, para inciar las librerias
	//CAUTION: DEMORADO!!! (~2min)
	#if GPU
		cv::gpu::GpuMat G_initMat;
		G_initMat.create(1, 1, CV_8U);
	#endif

	//Iniciar el cargador
	cargador.start();
	
	//Crear 2 hilos, para obtener/calcular imagenes y para mostrar
	boost::thread* loopHilo = new boost::thread(loopKinect);
	boost::thread* obtenerHilo = new boost::thread(obtenerFrames);
	loopHilo->join();
	obtenerHilo->join();

	//Detener el cargador
	cargador.stop();

	return 0;
}
