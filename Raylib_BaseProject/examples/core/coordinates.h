/*******************************************************************************************
* Coordonn�es Polaires, Cylindriques et Sph�riques
* ******************************************************************************************/
#include <raymath.h>
#include <math.h>
#include <float.h>
#include <vector>

#define EPSILON 1.e-6f

#ifndef STRUCTS
#define STRUCTS

struct Polar {
	float rho;
	float theta;
};

struct Cylindrical {
	float rho;
	float theta;
	float y;
};

struct Spherical {
	float rho;
	float theta;
	float phi;
};

#endif

Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive);
Vector2 PolarToCartesian(Polar polar);
Cylindrical CartesianToCylindrical(Vector3 cart);
Vector3 CylindricalToCartesien(Cylindrical cyl);
Spherical CartesianToSpherical(Vector3 cart);
Vector3 SphericalToCartesian(Spherical sph);
