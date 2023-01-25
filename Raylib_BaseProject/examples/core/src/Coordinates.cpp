#include "Coordinates.hpp"

/*************************************************************************************************************************************
*					Fonctions de conversion de coordonnées cartésiennes en coordonnées polaires (et inversement)					 *
**************************************************************************************************************************************/
Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive = true)
{
	Polar polar = { Vector2Length(cart), atan2f(cart.y, cart.x) };
	// Si l'utilisateur souhaite avoir un angle positif et que l'angle actuel est négatif, on l'ajoute à 2*PI pour l'avoir en positif
	if (keepThetaPositive && polar.theta < 0) polar.theta += 2 * PI;
	return polar;
}

Vector2 PolarToCartesian(Polar polar)
{
	return Vector2Scale({ cosf(polar.theta),sinf(polar.theta) }, polar.rho);
}

/*************************************************************************************************************************************
*					Fonctions de conversion de coordonnées cartésiennes en coordonnées cylindriques (et inversement)				 *
**************************************************************************************************************************************/
Cylindrical CartesianToCylindrical(Vector3 cart)
{
	Cylindrical cyl;
	cyl.y = cart.y;
	cyl.rho = sqrtf(powf(cart.x, 2) + powf(cart.z, 2)); // RHO² = X² + Z²


	if (cyl.rho < EPSILON) cyl.theta = 0;
	else {
		cyl.theta = asinf(cart.x / cyl.rho);
		if (cart.z < 0) cyl.theta = PI - cyl.theta;
	}
	return cyl;
}

Vector3 CylindricalToCartesien(Cylindrical cyl)
{
	return { cyl.rho * sinf(cyl.theta), cyl.y, cyl.rho * cosf(cyl.theta) };
}

/*************************************************************************************************************************************
*					Fonctions de conversion de coordonnées cartésiennes en coordonnées sphériques (et inversement)					 *
**************************************************************************************************************************************/
Spherical CartesianToSpherical(Vector3 cart)
{
	Spherical sph;
	sph.rho = sqrtf(powf(cart.x, 2) + powf(cart.y, 2) + powf(cart.z, 2)); // RHO² = X² + Y² + Z²

	if (sph.rho < EPSILON) // Si la distance RHO est trop petite, les points se confondent peu importe la valeur des angles THETA ou PHI
	{
		sph.theta = 0.0f;
		sph.phi = 0.0f;
	}
	else 
	{
		sph.phi = acosf(cart.y / sph.rho);

		if ((sph.phi < EPSILON) || (sph.phi > PI - EPSILON)) sph.theta = 0.0f;
		else 
		{
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
