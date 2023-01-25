/*******************************************************************************************
* Coordonnées Polaires, Cylindriques et Sphériques
* ******************************************************************************************/
#include <raymath.h>
#include <math.h>
#include <float.h>
#include <vector>

#define EPSILON 1.e-6f

#ifndef COORDINATES_STRUCTS
#define COORDINATES_STRUCTS

// Stocke les informations relatives à un référentiel dans l'espace 3D
struct ReferenceFrame {
    Vector3 origin;
    Vector3 i, j, k;
    Quaternion q;

    ReferenceFrame()
    {
        origin = { 0,0,0 };
        i = { 1,0,0 };
        j = { 0,1,0 };
        k = { 0,0,1 };
        q = QuaternionIdentity();
    }

    ReferenceFrame(Vector3 origin, Quaternion q)
    {
        this->q = q;
        this->origin = origin;
        i = Vector3RotateByQuaternion({ 1,0,0 }, q);
        j = Vector3RotateByQuaternion({ 0,1,0 }, q);
        k = Vector3RotateByQuaternion({ 0,0,1 }, q);
    }

    void Translate(Vector3 vect)
    {
        this->origin = Vector3Add(this->origin, vect);
    }

    void RotateByQuaternion(Quaternion qRot)
    {
        q = QuaternionMultiply(qRot, q);
        i = Vector3RotateByQuaternion({ 1,0,0 }, q);
        j = Vector3RotateByQuaternion({ 0,1,0 }, q);
        k = Vector3RotateByQuaternion({ 0,0,1 }, q);
    }
};

// Représente un point dans un système de coordonnées polaires(2D)
struct Polar {
	float rho;
	float theta;
};

// Représente un point dans un système de coordonnées cylindriques(3D)
struct Cylindrical {
	float rho;
	float theta;
	float y;
};

// Représente un point dans un système de coordonnées sphériques(3D)
struct Spherical {
	float rho;
	float theta;
	float phi;
};

#endif

#ifndef COORDINATES_CONVERSIONS_METHODS
#define COORDINATES_CONVERSIONS_METHODS
// Fonctions de conversion de coordonnées cartésiennes en coordonnées polaires (et inversement)
Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive);
Vector2 PolarToCartesian(Polar polar);

// Fonctions de conversion de coordonnées cartésiennes en coordonnées cylindriques (et inversement)
Cylindrical CartesianToCylindrical(Vector3 cart);
Vector3 CylindricalToCartesien(Cylindrical cyl);

// Fonctions de conversion de coordonnées cartésiennes en coordonnées sphériques (et inversement)
Spherical CartesianToSpherical(Vector3 cart);
Vector3 SphericalToCartesian(Spherical sph);

#endif
