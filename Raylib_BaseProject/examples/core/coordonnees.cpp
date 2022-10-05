/*******************************************************************************************
* Fonctions de conversion des différents types de coordonnées				
* ******************************************************************************************/

#include "coordonnees.h"

Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive = true)
{
	Polar polar = { Vector2Length(cart),atan2f(cart.y,cart.x) };
	if (keepThetaPositive && polar.theta < 0)polar.theta += 2 * PI;
	return polar;
}

Vector2 PolarToCartesian(Polar polar)
{
	return Vector2Scale({ cosf(polar.theta),sinf(polar.theta) }, polar.rho);
}

Cylindrical CartesianToCylindric(Vector3 cart)
{
	Cylindrical cyl;
	cyl.y = cart.y;
	cyl.rho = sqrtf(powf(cart.x, 2) + powf(cart.z, 2));

	if (cyl.rho < EPSILON) cyl.theta = 0;
	else {
		cyl.theta = asinf(cart.x / cyl.rho);
		if (cart.z < 0) cyl.theta = PI - cyl.theta;
	}
	return cyl;
}

Vector3 CylindricToCartesien(Cylindrical cyl)
{
	return { cyl.rho * sinf(cyl.theta), cyl.y, cyl.rho * cosf(cyl.theta) };
}

Spherical CartesianToSpherical(Vector3 cart)
{
	Spherical sph;
	sph.rho = sqrtf(powf(cart.x, 2) + powf(cart.y, 2) + powf(cart.z, 2));

	if (sph.rho < EPSILON) {
		sph.theta = 0.0f;
		sph.phi = 0.0f;
	}
	else {
		sph.phi = acosf(cart.y / sph.rho);
		if ((sph.phi < EPSILON) || (sph.phi > PI - EPSILON)) sph.theta = 0.0f;
		else {
			sph.theta = asinf(cart.x / (sph.rho * sinf(sph.phi)));

			if (cart.z < 0.0f) sph.theta = PI - sph.theta;
		}
	}
	return sph;
}

Vector3 SphericalToCartesian(Spherical sph)
{
	return { sph.rho * sinf(sph.phi) * sinf(sph.theta), sph.rho * cosf(sph.phi), sph.rho * sinf(sph.phi) * cosf(sph.theta) };
}
