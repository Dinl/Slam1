#ifndef alineadorCERES_H_
#define alineadorCERES_H_

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>

class alineador3D{
public:
	
	//Constructor, el punto de la primera imagen 
	alineador3D(double original_x, double original_y, double original_z){
		P1_x = original_x;
		P1_y = original_y;
		P1_z = original_z;
	};

	/***********************************************************************************
	*	Operador de costo utilizado por CERES
	*	Tmatrix -> Ingresa como un array de 6 valores:
	*				Tmatrix[0] -> Angulo yaw
	*				Tmatrix[1] -> Angulo pitch
	*				Tmatrix[2] -> Angulo roll
	*				Tmatrix[3] -> Traslacion en X
	*				Tmatrix[4] -> Traslacion en Y
	*				Tmatrix[5] -> Traslacion en Z
	*				Tmatrix[6] -> distancia focal X
	*				Tmatrix[7] -> distancia focal Y
	*				Tmatrix[8] -> Centro de la imagen en X
	*				Tmatrix[9] -> Centro de la imagen en Y
	*				Tmatrix[10] -> K1
	*				Tmatrix[11] -> K2
	*
	*	punto -> Ingresa como un array de 3 valores con XYZ del destino
	*				punto[0] -> X
	*				punto[1] -> Y
	*				punto[2] -> Z
	*
	*	residuos -> Array de 3 valores con los residuos en XYZ usado por ceres
	*
	************************************************************************************/

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