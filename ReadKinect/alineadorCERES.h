#ifndef alineadorCERES_H_
#define alineadorCERES_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>

/***********************************************************************************
	*	Clase T para realizar la alineacion de observaciones entre 2 nubes
	*
************************************************************************************/
class alineador3D{
public:
	
	/***********************************************************************************
	*	Constructor:
	*	Se crea con el punto XYZ de la primera nube 
	***********************************************************************************/
	alineador3D(double original_x, double original_y, double original_z){
		P1_x = original_x;
		P1_y = original_y;
		P1_z = original_z;
	};

	/***********************************************************************************
	*	Operador
	*	Es el operador quien recibe y calcula los residuos para cada punto
	*
	*	Textrinseca -> Array 6 double: [Rx Ry Rz X Y Z] rotaciones en el sentido de euler
	*	punto3D		-> Punto XYZ correspondiente al punto2D
	*
	*	El proceso consiste en:
	*	
	*	punto2D -> (intrinseca) -> punto2d3d -> (extrinseca) -> transformedpoint3d
	*	-> (intrinseca) -> transformedPunto2d
	*	residuo = original - transformedPunto2d
	***********************************************************************************/
	template <typename T>
	bool operator()(const T* const Textrinseca, const T* const punto3D, T* residuos) const {
		
		//Obtener el punto transformado en rotacion y traslacion
		T transformedPoint3D[3];
		ceres::AngleAxisRotatePoint(Textrinseca, punto3D, transformedPoint3D);

		transformedPoint3D[0] += Textrinseca[3];
		transformedPoint3D[1] += Textrinseca[4];
		transformedPoint3D[2] += Textrinseca[5];

		//Obtener la distancia residual
		residuos[0] = T(P1_x) - transformedPoint3D[0];
		residuos[1] = T(P1_y) - transformedPoint3D[1];
		residuos[2] = T(P1_z) - transformedPoint3D[2];

		return true;
	}

	static ceres::CostFunction* Create(const double observed_x, const double observed_y, const double observed_z) {
		return (new ceres::AutoDiffCostFunction<alineador3D, 3, 6, 3>(new alineador3D(observed_x, observed_y, observed_z)));
	}

private:
	double P1_x, P1_y, P1_z;
};

#endif