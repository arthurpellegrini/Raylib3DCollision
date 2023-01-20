#include <rlgl.h>
#include <iostream>
#include "My3DPrimitives.hpp"

/******************************************************************
*							LINE								  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un segment à partir des coordonnées définnisant son point de départ et sa direction
/// </summary>
/// <param name="line">Contient les deux vecteurs permettant de positionner le point de la ligne et son vecteur de direction</param>
/// <param name="color">Contient la couleur de la ligne</param>
void MyDrawLine(Line line, Color color)
{
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(line.pt.x, line.pt.y, line.pt.z);
	rlVertex3f(line.pt.x + line.dir.x, line.pt.y + line.dir.y, line.pt.z + line.dir.z);
	rlEnd();
}

/******************************************************************
*							SEGMENT								  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un segment à partir de deux points donnés en paramètre
/// </summary>
/// <param name="segment">Contient les deux vecteurs permettant de positionner les points du segment</param>
/// <param name="color">Contient la couleur du segment</param>
void MyDrawSegment(Segment segment, Color color)
{
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(segment.pt1.x, segment.pt1.y, segment.pt1.z);
	rlVertex3f(segment.pt2.x, segment.pt2.y, segment.pt2.z);
	rlEnd();
}

/******************************************************************
*							TRIANGLE							  *
*******************************************************************/

// <summary>
/// Méthode permettant de dessiner un triangle d'une certaine couleur à partir de trois points donnés en paramètre
/// </summary>
/// <param name="triangle">Contient les trois vecteurs permettant de positionner les points du triangle</param>
/// <param name="color">Contient la couleur du triangle</param>
void MyDrawPolygonTriangle(Triangle triangle, Color color)
{
	int numVertex = 3;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(triangle.pt1.x, triangle.pt1.y, triangle.pt1.z);
	rlVertex3f(triangle.pt2.x, triangle.pt2.y, triangle.pt2.z);
	rlVertex3f(triangle.pt3.x, triangle.pt3.y, triangle.pt3.z);
	rlEnd();
}

// <summary>
/// Méthode permettant de dessiner un triangle en mode fil de fer à partir de trois points donnés en paramètre
/// </summary>
/// <param name="triangle">Contient les trois vecteurs permettant de positionner les points du triangle</param>
/// <param name="color">Contient la couleur du maillage</param>
void MyDrawWireframeTriangle(Triangle triangle, Color color)
{
	int numVertex = 3;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(triangle.pt1.x, triangle.pt1.y, triangle.pt1.z);
	rlVertex3f(triangle.pt2.x, triangle.pt2.y, triangle.pt2.z);
	rlVertex3f(triangle.pt3.x, triangle.pt3.y, triangle.pt3.z);
	rlEnd();
}

/// <summary>
/// Méthode permettant de dessiner un triangle en mode polygone et/ou fil de fer
/// </summary>
/// <param name="triangle">Contient les trois vecteurs permettant de positionner les points du triangle</param>
/// <param name="drawPolygon">Indique si le triangle doit être dessiné en mode polygone</param>
/// <param name="drawWireframe">Indique si le triangle doit être dessiné en mode fil de fer</param>
/// <param name="polygonColor">Contient la couleur à utiliser pour dessiner le triangle en mode polygone</param>
/// <param name="wireframeColor">Contient la couleur à utiliser pour dessiner le triangle en mode fil de fer</param>
void MyDrawTriangle(Triangle triangle, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonTriangle(triangle, polygonColor);
	if (drawWireframe) MyDrawWireframeTriangle(triangle, wireframeColor);
}


/******************************************************************
*							QUAD								  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un quad en mode polygone à partir de quatre points donnés en paramètre
/// </summary>
/// <param name="quad">Contient les informations pour positionner et orienter le quad</param>
/// <param name="color">Contient la couleur de la surface</param>
void MyDrawPolygonQuad(Quad quad, Color color)
{
	int numVertex = 6;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(quad.ref.origin.x, quad.ref.origin.y, quad.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(quad.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(quad.extents.x, 1, quad.extents.z);
	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(1, 0, 1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(1, 0, 1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, 1);
	rlEnd();
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un quad en mode fil de fer à partir de quatre points donnés en paramètre
/// </summary>
/// <param name="quad">Contient les informations pour positionner et orienter le quad</param>
/// <param name="color">Contient la couleur du fil de fer</param>
void MyDrawWireframeQuad(Quad quad, Color color)
{
	int numVertex = 10;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(quad.ref.origin.x, quad.ref.origin.y, quad.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(quad.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(quad.extents.x, 1, quad.extents.z);
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(1, 0, 1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(1, 0, 1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, 1);
	rlVertex3f(-1, 0, 1);
	rlVertex3f(1, 0, 1);
	rlEnd();
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un quad en mode polygone et/ou fil de fer
/// </summary>
/// <param name="quad">Contient les informations pour positionner et orienter le quad</param>
/// <param name="drawPolygon">Indique si le quad doit être dessiné en mode polygone</param>
/// <param name="drawWireframe">Indique si le quad doit être dessiné en mode fil de fer</param>
/// <param name="polygonColor">Contient la couleur à utiliser pour dessiner le quad en mode polygone</param>
/// <param name="wireframeColor">Contient la couleur à utiliser pour dessiner le quad en mode fil de fer</param>
void MyDrawQuad(Quad quad, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
}


/******************************************************************
*							PLANE								  *
*******************************************************************/

// <summary>
/// Méthode permettant de dessiner un plane en mode polygone et/ou fil de fer (imagination requise pour étendre à l'infini)
/// </summary>
/// <param name="plane">Contient les informations pour positionner et orienter le plane</param>
/// <param name="drawPolygon">Indique si le plane doit être dessiné en mode polygone</param>
/// <param name="drawWireframe">Indique si le plane doit être dessiné en mode fil de fer</param>
/// <param name="polygonColor">Contient la couleur à utiliser pour dessiner le plane en mode polygone</param>
/// <param name="wireframeColor">Contient la couleur à utiliser pour dessiner le plane en mode fil de fer</param>
void MyDrawPlane(Plane plane, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	ReferenceFrame ref;
	ref.origin = Vector3Scale(plane.n, plane.d);
	ref.q = QuaternionFromVector3ToVector3({ 1,0,0 }, plane.n);

	rlPushMatrix();
	rlTranslatef(ref.origin.x, ref.origin.y, ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(1, 0, 1);

	Quad quad = { ref, { 40,0,40 } };

	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
	rlPopMatrix();
}


/******************************************************************
*							BOX 								  *
*******************************************************************/

// <summary>
/// Méthode permettant de dessiner une box en mode polygone
/// </summary>
/// <param name="box">Contient les informations pour positionner et orienter la box</param>
/// <param name="color">Contient la couleur à utiliser pour dessiner la box</param>
void MyDrawPolygonBox(Box box, Color color) 
{
	int numVertex = 36;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);

	Quaternion q = QuaternionIdentity();
	Quad top = { ReferenceFrame({ 0,1,0 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI);
	Quad bottom = { ReferenceFrame({ 0,-1,0 }, q), {1,0,1} };

	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), -PI / 2);
	Quad front = { ReferenceFrame({ 1,0,0 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2);
	Quad back = { ReferenceFrame({ -1,0,0 }, q), {1,0,1} };

	q = QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2);
	Quad left = { ReferenceFrame({ 0,0,1 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), -PI / 2);
	Quad right = { ReferenceFrame({ 0,0,-1 }, q), {1,0,1} };

	MyDrawPolygonQuad(top, color);
	MyDrawPolygonQuad(bottom, color);
	MyDrawPolygonQuad(front, color);
	MyDrawPolygonQuad(back, color);
	MyDrawPolygonQuad(left, color);
	MyDrawPolygonQuad(right, color);
	
	rlPopMatrix();
}

// <summary>
/// Méthode permettant de dessiner une box en mode fil de fer
/// </summary>
/// <param name="box">Contient les informations pour positionner et orienter la box</param>
/// <param name="color">Contient la couleur à utiliser pour dessiner le fil de fer</param>
void MyDrawWireframeBox(Box box, Color color) 
{
	int numVertex = 60;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);

	Quaternion q = QuaternionIdentity();
	Quad top = { ReferenceFrame({ 0,1,0 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI);
	Quad bottom = { ReferenceFrame({ 0,-1,0 }, q), {1,0,1} };

	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), -PI / 2);
	Quad front = { ReferenceFrame({ 1,0,0 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2);
	Quad back = { ReferenceFrame({ -1,0,0 }, q), {1,0,1} };

	q = QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2);
	Quad left = { ReferenceFrame({ 0,0,1 }, q), {1,0,1} };
	q = QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), -PI / 2);
	Quad right = { ReferenceFrame({ 0,0,-1 }, q), {1,0,1} };

	MyDrawWireframeQuad(top, color);
	MyDrawWireframeQuad(bottom, color);
	MyDrawWireframeQuad(front, color);
	MyDrawWireframeQuad(back, color);
	MyDrawWireframeQuad(left, color);
	MyDrawWireframeQuad(right, color);

	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une box en mode polygone et/ou fil de fer
/// </summary>
/// <param name="box">Contient les informations pour positionner et orienter la box</param>
/// <param name="drawPolygon">Indique si la box doit être dessinée en mode polygone</param>
/// <param name="drawWireframe">Indique si la box doit être dessinée en mode fil de fer</param>
/// <param name="polygonColor">Contient la couleur à utiliser pour dessiner la box en mode polygone</param>
/// <param name="wireframeColor">Contient la couleur à utiliser pour dessiner la box en mode fil de fer</param>
void MyDrawBox(Box box, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}


/******************************************************************
*							DISK								  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un disk d'une certaine couleur en utilisant des triangles
/// </summary>
/// <param name="disk">Contient les informations sur le disk (position, rayon, orientation)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le disk</param>
/// <param name="color">Contient la couleur du disk</param>
void MyDrawPolygonDisk(Disk disk, int nSectors, Color color)
{
	int numVertex = nSectors * 3;
	rlPushMatrix();
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 1, disk.radius);

	Cylindrical v1, v2;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des sommets du triangle
		v1 = { 1, 2 * PI / nSectors * i, 0 };
		v2 = { 1, 2 * PI / nSectors * (i + 1), 0 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawTriangle3D(CylindricalToCartesien(v2), { 0 }, CylindricalToCartesien(v1), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un disk en mode fil de fer
/// </summary>
/// <param name="disk">Contient les informations sur le disk (position, rayon, orientation)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le disk</param>
/// <param name="color">Contient la couleur du fil de fer</param>
void MyDrawWireframeDisk(Disk disk, int nSectors, Color color)
{
	int numVertex = nSectors * 3;
	rlPushMatrix();
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 1, disk.radius);

	Cylindrical v1, v2;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points des segments
		v1 = { 1, 2 * PI / nSectors * i, 0 };
		v2 = { 1, 2 * PI / nSectors * (i + 1), 0 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v2), color);
		DrawLine3D({ 0 }, CylindricalToCartesien(v2), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un disk en mode polygone et/ou fil de fer
/// </summary>
/// <param name="disk">Contient les informations relatives au disk (origine, rayon, orientation)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le disk</param>
/// <param name="drawPolygon">Définit si on dessine le disk en mode polygone</param>
/// <param name="drawWireframe">Définit si on dessine le disk en mode fil de fer</param>
/// <param name="polygonColor">Couleur utilisée pour dessiner le disk en mode polygone</param>
/// <param name="wireframeColor">Couleur utilisée pour dessiner le disk en mode fil de fer</param>
void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonDisk(disk, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeDisk(disk, nSectors, wireframeColor);
}


/******************************************************************
*							SPHERE 								  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner une sphère à partir des paramètres donnés
/// </summary>
/// <param name="sphere">Contient les informations sur la sphère (position, rotation, rayon)</param>
/// <param name="nMeridians">Nombre de méridiens utilisés pour dessiner la sphère</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner la sphère</param>
/// <param name="color">Couleur de la sphère</param>
void MyDrawPolygonSphere(Sphere sphere, int nMeridians, int nParallels, Color color) 
{
	int numVertex = nMeridians * nParallels * 4;
	rlPushMatrix();
	rlTranslatef(sphere.ref.origin.x, sphere.ref.origin.y, sphere.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(sphere.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(sphere.radius, sphere.radius, sphere.radius);

	// Points position for one quad in the sphere
	// 1-----2
	// | \   |
	// |  \  |
	// |   \ |
	// 3-----4
	Spherical pt1, pt2, pt3, pt4; // = {rho, theta, phi}

	//	MERIDIAN -> phi € [0°, 180°]
	//		|
	//		|
	// -----|----- PARALLEL -> theta € [0°, 360°]
	//		|
	//		|

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			pt1 = { 1, 2 * PI / nMeridians * m, PI / nParallels * p};
			pt2 = { 1, 2 * PI / nMeridians * (m+1), PI / nParallels * p };
			pt3 = { 1, 2 * PI / nMeridians * m, PI / nParallels * (p+1) };
			pt4 = { 1, 2 * PI / nMeridians * (m+1), PI / nParallels * (p+1) };

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt4), SphericalToCartesian(pt2), color);
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt3), SphericalToCartesian(pt4), color);
		}
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une sphère en fil de fer à partir des paramètres donnés
/// </summary>
/// <param name="sphere">Contient les informations sur la sphère (position, rotation, rayon)</param>
/// <param name="nMeridians">Nombre de méridiens utilisés pour dessiner la sphère</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner la sphère</param>
/// <param name="color">Couleur du fil de fer</param>
void MyDrawWireframeSphere(Sphere sphere, int nMeridians, int nParallels, Color color) 
{
	int numVertex = nMeridians * nParallels * 4;
	rlPushMatrix();
	rlTranslatef(sphere.ref.origin.x, sphere.ref.origin.y, sphere.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(sphere.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(sphere.radius, sphere.radius, sphere.radius);

	// Points position for one quad in the sphere
	// 1-----2
	// | \   |
	// |  \  |
	// |   \ |
	// 3-----4
	Spherical pt1, pt2, pt3, pt4; // = {rho, theta, phi}

	//	MERIDIAN -> phi € [0°, 180°]
	//		|
	//		|
	// -----|----- PARALLEL -> theta € [0°, 360°]
	//		|
	//		|

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			pt1 = { 1, 2 * PI / nMeridians * m, PI / nParallels * p };
			pt2 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * p };
			pt3 = { 1, 2 * PI / nMeridians * m, PI / nParallels * (p + 1) };
			pt4 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * (p + 1) };

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt4), color);
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt2), color);
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt3), color);
		}
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une sphère en mode polygone et/ou fil de fer
/// </summary>
/// <param name="sphere">Contient les informations sur la sphère tels que sa référence => orientation et position et son rayon</param>
/// <param name="nMeridians">Le nombre de méridiens de la sphère</param>
/// <param name="nParallels">Le nombre de parallèles de la sphère</param>
/// <param name="drawPolygon">Indique si on dessine la sphère en mode polygones</param>
/// <param name="drawWireframe">Indique si on dessine la sphère en mode fil de fer</param>
/// <param name="polygonColor">La couleur des polygones</param>
/// <param name="wireframeColor">La couleur du fil de fer</param>
void MyDrawSphere(Sphere sphere, int nMeridians, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonSphere(sphere, nMeridians, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeSphere(sphere, nMeridians, nParallels, wireframeColor);
}


/******************************************************************
*					Sphere Optimization Methods					  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner la portion de sphère à partir d'un certain nombre de méridiens et de parallèles,
///  ainsi que de la plage de début et de fin pour theta et phi
/// </summary>
/// <param name="sphere">Contient les informations de la sphère (centre, rayon, orientation)</param>
/// <param name="nMeridians">Nombre de meridiens utilisés pour dessiner la portion de sphère</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner la portion de sphère</param>
/// <param name="startTheta">Angle de départ (en radians) pour le dessin des parallèles</param>
/// <param name="endTheta">Angle final (en radians) pour le dessin des parallèles</param>
/// <param name="startPhi">Angle de départ (en radians) pour le dessin des meridiens</param>
/// <param name="endPhi">Angle final (en radians) pour le dessin des meridiens</param>
/// <param name="color">Couleur de la portion de sphère</param>
void MyDrawPolygonSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, Color color)
{
	int numVertex = nMeridians * nParallels * 4;
	rlPushMatrix();
	rlTranslatef(sphere.ref.origin.x, sphere.ref.origin.y, sphere.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(sphere.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(sphere.radius, sphere.radius, sphere.radius);

	// Points position for one quad in the sphere
	// 1-----2
	// | \   |
	// |  \  |
	// |   \ |
	// 3-----4
	Spherical pt1, pt2, pt3, pt4; // = {rho, theta, phi}

	//	MERIDIAN -> phi € [0°, 180°]
	//		|
	//		|
	// -----|----- PARALLEL -> theta € [0°, 360°]
	//		|
	//		|

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			pt1 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * m };
			pt2 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };
			pt3 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * m };
			pt4 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt2), SphericalToCartesian(pt4), color);
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt4), SphericalToCartesian(pt3), color);
		}
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner la portion de sphère en fil de fer à partir d'un certain nombre de méridiens et de parallèles,
///  ainsi que de la plage de début et de fin pour theta et phi
/// </summary>
/// <param name="sphere">Contient les informations de la sphère (centre, rayon, orientation)</param>
/// <param name="nMeridians">Nombre de meridiens utilisés pour dessiner la portion de sphère</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner la portion de sphère</param>
/// <param name="startTheta">Angle de départ (en radians) pour le dessin des parallèles</param>
/// <param name="endTheta">Angle final (en radians) pour le dessin des parallèles</param>
/// <param name="startPhi">Angle de départ (en radians) pour le dessin des meridiens</param>
/// <param name="endPhi">Angle final (en radians) pour le dessin des meridiens</param>
/// <param name="color">Couleur de la ligne</param>
void MyDrawWireframeSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, Color color)
{
	int numVertex = nMeridians * nParallels * 4;
	rlPushMatrix();
	rlTranslatef(sphere.ref.origin.x, sphere.ref.origin.y, sphere.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(sphere.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(sphere.radius, sphere.radius, sphere.radius);

	// Points position for one quad in the sphere
	// 1-----2
	// | \   |
	// |  \  |
	// |   \ |
	// 3-----4
	Spherical pt1, pt2, pt3, pt4; // = {rho, theta, phi}

	//	MERIDIAN -> phi € [0°, 180°]
	//		|
	//		|
	// -----|----- PARALLEL -> theta € [0°, 360°]
	//		|
	//		|

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			pt1 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * m };
			pt2 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };
			pt3 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * m };
			pt4 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt4), color);
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt2), color);
			DrawLine3D(SphericalToCartesian(pt1), SphericalToCartesian(pt3), color);
			DrawLine3D(SphericalToCartesian(pt2), SphericalToCartesian(pt4), color);
			DrawLine3D(SphericalToCartesian(pt3), SphericalToCartesian(pt4), color);
		}
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner la portion de sphère en mode polygone et/ou fil de fer
/// </summary>
/// <param name="sphere">Contient les informations de la sphère à dessiner, tels que sa position, sa rotation et son rayon</param>
/// <param name="nMeridians">Nombre de méridiens à utiliser pour dessiner la sphère</param>
/// <param name="nParallels">Nombre de parallèles à utiliser pour dessiner la sphère</param>
/// <param name="startTheta">Limite de début pour theta</param>
/// <param name="endTheta">Limite de fin pour theta</param>
/// <param name="startPhi">Limite de début pour phi</param>
/// <param name="endPhi">Limite de fin pour phi</param>
/// <param name="drawPolygon">Indicateur pour dessiner ou non la portion sphérique en mode polygone</param>
/// <param name="drawWireframe">Indicateur pour dessiner ou non la portion sphérique en mode fil de fer</param>
/// <param name="polygonColor">Couleur à utiliser pour dessiner la portion sphérique en mode polygone</param>
/// <param name="wireframeColor">Couleur à utiliser pour dessiner la portion sphérique en mode fil de fer</param>
void MyDrawSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, polygonColor);
	if (drawWireframe) MyDrawWireframeSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, wireframeColor);
}


/******************************************************************
*							CYLINDER							  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un cylindre en mode polygone d'une certaine couleur
///  et de spécifier s'il faut dessiner les disques supérieur et inférieur à partir de paramètres donnés
/// </summary>
/// <param name="cylinder">Contient les informations sur la position, la taille et l'orientation du cylindre</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le cylindre</param>
/// <param name="drawCaps">Indique s'il faut dessiner les disques supérieur et inférieur</param>
/// <param name="color">Contient la couleur du cylindre</param>
void MyDrawPolygonCylinder(Cylinder cylinder, int nSectors, bool drawCaps, Color color)
{
	int numVertex = nSectors * 3;
	if (drawCaps) numVertex *= 2;
	rlPushMatrix();
	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	Cylindrical v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		v1 = { 1, 2 * PI / nSectors * i, 1 };
		v2 = { 1, 2 * PI / nSectors * (i + 1), 1 };
		v3 = { 1, 2 * PI / nSectors * i, -1 };
		v4 = { 1, 2 * PI / nSectors * (i + 1), -1 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		if (drawCaps) { // alors dessin des disques supérieurs et inférieurs (formes discoïdales)
			DrawTriangle3D(CylindricalToCartesien(v2), { 0, 1, 0 }, CylindricalToCartesien(v1), color);
			DrawTriangle3D({ 0, -1, 0 }, CylindricalToCartesien(v4), CylindricalToCartesien(v3), color);
		}

		DrawTriangle3D(CylindricalToCartesien(v1), CylindricalToCartesien(v4), CylindricalToCartesien(v2), color);
		DrawTriangle3D(CylindricalToCartesien(v1), CylindricalToCartesien(v3), CylindricalToCartesien(v4), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un cylindre en mode fil de fer d'une certaine couleur
///  et de spécifier s'il faut dessiner les disques supérieur et inférieur à partir de paramètres donnés
/// </summary>
/// <param name="cylinder">Contient les informations sur la position, la taille et l'orientation du cylindre</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le cylindre</param>
/// <param name="drawCaps">Indique s'il faut dessiner les disques supérieur et inférieur</param>
/// <param name="color">Contient la couleur de la ligne</param>
void MyDrawWireframeCylinder(Cylinder cylinder, int nSectors, bool drawCaps, Color color)
{
	int numVertex = nSectors * 4;
	if (drawCaps) numVertex *= 2;
	rlPushMatrix();
	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	Cylindrical v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		v1 = { 1, 2 * PI / nSectors * i, 1 };
		v2 = { 1, 2 * PI / nSectors * (i + 1), 1 };
		v3 = { 1, 2 * PI / nSectors * i, -1 };
		v4 = { 1, 2 * PI / nSectors * (i + 1), -1 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		if (drawCaps) { // alors dessin des disques supérieurs et inférieurs (formes discoïdales)
			DrawLine3D(CylindricalToCartesien(v1), { 0, 1, 0 }, color);
			DrawLine3D(CylindricalToCartesien(v3), { 0, -1, 0 }, color);
		}

		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v2), color);
		DrawLine3D(CylindricalToCartesien(v3), CylindricalToCartesien(v4), color);
		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v3), color);
		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v4), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un cylindre en mode polygone et/ou fil de fer
/// </summary>
/// <param name="cylinder">Contient les informations relatives à la forme du cylindre (rayon, hauteur ...)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le cylindre</param>
/// <param name="drawCaps">Indique si les disques supérieur et inférieur du cylindre doivent être dessinés</param>
/// <param name="drawPolygon">Indique si le cylindre doit être dessiné en mode polygones</param>
/// <param name="drawWireframe">Indique si le cylindre doit être dessiné en mode fil de fer</param>
/// <param name="polygonColor">Couleur utilisée pour dessiner le cylindre en mode polygones</param>
/// <param name="wireframeColor">Couleur utilisée pour dessiner le cylindre en mode fil de fer</param>
void MyDrawCylinder(Cylinder cylinder, int nSectors, bool drawCaps, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCylinder(cylinder, nSectors, drawCaps, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(cylinder, nSectors, drawCaps, wireframeColor);
}

/// <summary>
/// Méthode permettant de dessiner un cylindre infini avec une certaine couleur
/// </summary>
/// <param name="infiniteCylinder">Contient les informations nécessaires pour positionner le cylindre</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner le cylindre</param>
/// <param name="drawPolygon">Indique si le cylindre doit être dessiné en mode polygones</param>
/// <param name="drawWireframe">Indique si le cylindre doit être dessiné en mode fil de fer</param>
/// <param name="polygonColor">Couleur utilisée pour dessiner le cylindre en mode polygones</param>
/// <param name="wireframeColor">Couleur utilisée pour dessiner le cylindre en mode fil de fer</param>
void MyDrawInfiniteCylinder(InfiniteCylinder infiniteCylinder, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	Cylinder inf_cylinder = Cylinder{ infiniteCylinder.ref,  40.0f, infiniteCylinder.radius };
	if (drawPolygon) MyDrawPolygonCylinder(inf_cylinder, nSectors, false, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(inf_cylinder, nSectors, false, wireframeColor);
}


/******************************************************************
*					Cylinder Optimization Methods				  *
*******************************************************************/

/// <summary>
/// Cette méthode permet de dessiner une portion de cylindre en utilisant un certain nombre de secteurs et une intervalle de début et de fin pour l'angle theta
/// </summary>
/// <param name="cylinder">Contient les informations nécessaires pour dessiner le cylindre, telles que la position, le rayon et la hauteur</param>
/// <param name="nSectors">Le nombre de secteurs à utiliser pour dessiner la portion de cylindre</param>
/// <param name="startTheta">L'angle de départ (en degrés) à utiliser pour dessiner la portion de cylindre</param>
/// <param name="endTheta">L'angle de fin (en degrés) à utiliser pour dessiner la portion de cylindre</param>
/// <param name="color">La couleur à utiliser pour dessiner le cylindre</param>
void MyDrawPolygonCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, Color color)
{
	int numVertex = nSectors * 4;
	rlPushMatrix();
	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	Cylindrical v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		v1 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, 1 };
		v2 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), 1 };
		v3 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, -1 };
		v4 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), -1 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawTriangle3D(CylindricalToCartesien(v1), CylindricalToCartesien(v4), CylindricalToCartesien(v2), color);
		DrawTriangle3D(CylindricalToCartesien(v1), CylindricalToCartesien(v3), CylindricalToCartesien(v4), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Cette méthode permet de dessiner une portion de cylindre en mode fil de fer en utilisant un certain nombre de secteurs et une intervalle de début et de fin pour l'angle theta
/// </summary>
/// <param name="cylinder">Contient les informations nécessaires pour dessiner le cylindre, telles que la position, le rayon et la hauteur</param>
/// <param name="nSectors">Le nombre de secteurs à utiliser pour dessiner la portion de cylindre</param>
/// <param name="startTheta">L'angle de départ (en degrés) à utiliser pour dessiner la portion de cylindre</param>
/// <param name="endTheta">L'angle de fin (en degrés) à utiliser pour dessiner la portion de cylindre</param>
/// <param name="color">La couleur à utiliser pour dessiner les lignes</param>
void MyDrawWireframeCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, Color color)
{
	int numVertex = nSectors * 4;
	rlPushMatrix();
	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	Cylindrical v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		v1 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, 1 };
		v2 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), 1 };
		v3 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, -1 };
		v4 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), -1 };

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v2), color);
		DrawLine3D(CylindricalToCartesien(v3), CylindricalToCartesien(v4), color);
		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v3), color);
		DrawLine3D(CylindricalToCartesien(v1), CylindricalToCartesien(v4), color);
		DrawLine3D(CylindricalToCartesien(v2), CylindricalToCartesien(v4), color);
	}
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une portion de cylindre en mode polygone et/ou de fil de fer
/// </summary>
/// <param name="cylinder">Contient les informations de base sur le cylindre (position, taille ...)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner la portion de cylindre</param>
/// <param name="startTheta">Angle de départ pour dessiner la portion de cylindre</param>
/// <param name="endTheta">Angle final pour dessiner la portion de cylindre</param>
/// <param name="drawPolygon">Détermine si la portion de cylindre doit être dessinée en utilisant des polygones</param>
/// <param name="drawWireframe">Détermine si la portion de cylindre doit être dessinée en utilisant un fil de fer</param>
/// <param name="polygonColor">Couleur à utiliser pour dessiner les polygones</param>
/// <param name="wireframeColor">Couleur à utiliser pour dessiner le fil de fer</param>
void MyDrawCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonCylinderPortion(cylinder, nSectors, startTheta, endTheta, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinderPortion(cylinder, nSectors, startTheta, endTheta, wireframeColor);
}


/******************************************************************
*							CAPSULE 							  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner une capsule en mode polygones
/// </summary>
/// <param name="capsule">Contient les informations de la capsule (référence => l'orientation et la position, hauteur, rayon)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner les sphères</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner les sphères et le cylindre</param>
/// <param name="color">Contient la couleur des polygones</param>
void MyDrawPolygonCapsule(Capsule capsule, int nSectors, int nParallels, Color color) 
{
	rlPushMatrix();
	rlTranslatef(capsule.ref.origin.x, capsule.ref.origin.y, capsule.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(capsule.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Cylinder capsule_cylinder = { ReferenceFrame({0, 0, 0}, QuaternionIdentity()), capsule.halfHeight, capsule.radius };
	Sphere capsule_sphere_top = { ReferenceFrame({0, capsule.halfHeight, 0}, QuaternionIdentity()), capsule.radius };
	Sphere capsule_sphere_bottom = { ReferenceFrame({0, -capsule.halfHeight, 0}, QuaternionIdentity()), capsule.radius };

	MyDrawPolygonSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonCylinder(capsule_cylinder, nParallels, false, color);
	MyDrawPolygonSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une capsule en mode fil de fer
/// </summary>
/// <param name="capsule">Contient les informations de la capsule (référence => l'orientation et la position, hauteur, rayon)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner les sphères</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner les sphères et le cylindre</param>
/// <param name="color">Contient la couleur des lignes</param>
void MyDrawWireframeCapsule(Capsule capsule, int nSectors, int nParallels, Color color) 
{
	rlPushMatrix();
	rlTranslatef(capsule.ref.origin.x, capsule.ref.origin.y, capsule.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(capsule.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Cylinder capsule_cylinder = { ReferenceFrame({0, 0, 0}, QuaternionIdentity()), capsule.halfHeight, capsule.radius };
	Sphere capsule_sphere_top = { ReferenceFrame({0, capsule.halfHeight, 0}, QuaternionIdentity()), capsule.radius};
	Sphere capsule_sphere_bottom = { ReferenceFrame({0, -capsule.halfHeight, 0}, QuaternionIdentity()), capsule.radius };

	MyDrawWireframeSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeCylinder(capsule_cylinder, nParallels, false, color);
	MyDrawWireframeSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	
	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une capsule en mode polygone et/ou fil de fer
/// </summary>
/// <param name="capsule">Contient les informations de la capsule (référence => l'orientation et la position, hauteur, rayon)</param>
/// <param name="nSectors">Nombre de secteurs utilisés pour dessiner les sphères</param>
/// <param name="nParallels">Nombre de parallèles utilisés pour dessiner les sphères et le cylindre</param>
/// <param name="drawPolygon">Détermine si la capsule doit être dessinée en mode polygone</param>
/// <param name="drawWireframe">Détermine si la capsule doit être dessinée en mode fil de fer</param>
/// <param name="polygonColor">Contient la couleur des polygones</param>
/// <param name="wireframeColor">Contient la couleur des lignes en mode fil de fer</param>
void MyDrawCapsule(Capsule capsule, int nSectors, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCapsule(capsule, nSectors, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeCapsule(capsule, nSectors, nParallels, wireframeColor);
}


/******************************************************************
*						ROUNDED BOX 							  *
*******************************************************************/

/// <summary>
/// Méthode permettant de dessiner un boîte arrondie en mode polygones
/// </summary>
/// <param name="roundedBox">Contient les informations de la boîte arrondie (référence => l'orientation et la position, dimensions, rayon des Sphères)</param>
/// <param name="nSectors">Nombre de secteurs utilisés</param>
/// <param name="color">Contient la couleur des polygones</param>
void MyDrawPolygonRoundedBox(RoundedBox roundedBox, int nSectors, Color color)
{
	rlPushMatrix();
	rlTranslatef(roundedBox.ref.origin.x, roundedBox.ref.origin.y, roundedBox.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(roundedBox.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Quaternion q = QuaternionIdentity();

	//SPHERES PART
	Sphere sph_top_front_left = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_front_right = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_right = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_left = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_left = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_right = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_right = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_left = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };

	MyDrawPolygonSpherePortion(sph_top_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);

	//QUADS PART
	Quad quad_top = { ReferenceFrame({ 0,roundedBox.extents.y + roundedBox.radius,0 }, q), { roundedBox.extents.x, 0, roundedBox.extents.z } };
	Quaternion quad_q = QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI));
	Quad quad_bottom = { ReferenceFrame({ 0,-(roundedBox.extents.y + roundedBox.radius),0 }, quad_q), { roundedBox.extents.x, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2));
	Quad quad_front = { ReferenceFrame({ roundedBox.extents.x + roundedBox.radius,0,0 }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI));
	Quad quad_back = { ReferenceFrame({ -(roundedBox.extents.x + roundedBox.radius),0,0 }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2));
	Quad quad_right = { ReferenceFrame({ 0,0,-(roundedBox.extents.z + roundedBox.radius) }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.x} };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI));
	Quad quad_left = { ReferenceFrame({ 0,0,roundedBox.extents.z + roundedBox.radius }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.x} };

	MyDrawPolygonQuad(quad_top, color);
	MyDrawPolygonQuad(quad_bottom, color);
	MyDrawPolygonQuad(quad_front, color);
	MyDrawPolygonQuad(quad_back, color);
	MyDrawPolygonQuad(quad_right, color);
	MyDrawPolygonQuad(quad_left, color);

	//CYLINDICALS PART
	Cylinder cyl_top_front = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_top_back = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_bottom_back = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_bottom_front = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	
	Cylinder cyl_front_right = { ReferenceFrame({ roundedBox.extents.x,0,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_back_right = { ReferenceFrame({ -roundedBox.extents.x,0,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_back_left = { ReferenceFrame({ -roundedBox.extents.x,0,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_front_left = { ReferenceFrame({ roundedBox.extents.x,0,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	
	Cylinder cyl_top_left = { ReferenceFrame({ 0,roundedBox.extents.y,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_top_right = { ReferenceFrame({ 0,roundedBox.extents.y,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_bottom_right = { ReferenceFrame({ 0,-roundedBox.extents.y,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_bottom_left = { ReferenceFrame({ 0,-roundedBox.extents.y,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };

	MyDrawPolygonCylinderPortion(cyl_bottom_front, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_top_front, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_top_back, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_bottom_back, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_front_right, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_back_right, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_back_left, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_front_left, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_top_left, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_top_right, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_bottom_right, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawPolygonCylinderPortion(cyl_bottom_left, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);

	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner un boîte arrondie en mode fil de fer
/// </summary>
/// <param name="roundedBox">Contient les informations de la boîte arrondie (référence => l'orientation et la position, dimensions, rayon des Sphères)</param>
/// <param name="nSectors">Nombre de secteurs utilisés</param>
/// <param name="color">Contient la couleur des lignes</param>
void MyDrawWireframeRoundedBox(RoundedBox roundedBox, int nSectors, Color color)
{
	rlPushMatrix();
	rlTranslatef(roundedBox.ref.origin.x, roundedBox.ref.origin.y, roundedBox.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(roundedBox.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Quaternion q = QuaternionIdentity();

	//SPHERES PART
	Sphere sph_top_front_left = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_front_right = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_right = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_left = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_left = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_right = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_right = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_left = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };

	MyDrawWireframeSpherePortion(sph_top_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);

	//QUADS PART
	Quad quad_top = { ReferenceFrame({ 0,roundedBox.extents.y + roundedBox.radius,0 }, q), { roundedBox.extents.x, 0, roundedBox.extents.z } };
	Quaternion quad_q = QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI));
	Quad quad_bottom = { ReferenceFrame({ 0,-(roundedBox.extents.y + roundedBox.radius),0 }, quad_q), { roundedBox.extents.x, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2));
	Quad quad_front = { ReferenceFrame({ roundedBox.extents.x + roundedBox.radius,0,0 }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI));
	Quad quad_back = { ReferenceFrame({ -(roundedBox.extents.x + roundedBox.radius),0,0 }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.z } };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2));
	Quad quad_right = { ReferenceFrame({ 0,0,-(roundedBox.extents.z + roundedBox.radius) }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.x} };
	quad_q = QuaternionMultiply(quad_q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI));
	Quad quad_left = { ReferenceFrame({ 0,0,roundedBox.extents.z + roundedBox.radius }, quad_q), { roundedBox.extents.y, 0, roundedBox.extents.x} };

	MyDrawWireframeQuad(quad_top, color);
	MyDrawWireframeQuad(quad_bottom, color);
	MyDrawWireframeQuad(quad_front, color);
	MyDrawWireframeQuad(quad_back, color);
	MyDrawWireframeQuad(quad_left, color);
	MyDrawWireframeQuad(quad_right, color);

	//CYLINDICALS PART
	Cylinder cyl_top_front = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_top_back = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_bottom_back = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_bottom_front = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,0 }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2))), roundedBox.extents.z, roundedBox.radius };
	Cylinder cyl_front_right = { ReferenceFrame({ roundedBox.extents.x,0,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_back_right = { ReferenceFrame({ -roundedBox.extents.x,0,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_back_left = { ReferenceFrame({ -roundedBox.extents.x,0,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_front_left = { ReferenceFrame({ roundedBox.extents.x,0,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), PI / 2))), roundedBox.extents.y, roundedBox.radius };
	Cylinder cyl_top_left = { ReferenceFrame({ 0,roundedBox.extents.y,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_top_right = { ReferenceFrame({ 0,roundedBox.extents.y,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_bottom_right = { ReferenceFrame({ 0,-roundedBox.extents.y,-roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };
	Cylinder cyl_bottom_left = { ReferenceFrame({ 0,-roundedBox.extents.y,roundedBox.extents.z }, QuaternionMultiply(q, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), PI / 2))), roundedBox.extents.x, roundedBox.radius };

	MyDrawWireframeCylinderPortion(cyl_bottom_front, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_top_front, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_top_back, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_bottom_back, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_front_right, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_back_right, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_back_left, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_front_left, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_top_left, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_top_right, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_bottom_right, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, color);
	MyDrawWireframeCylinderPortion(cyl_bottom_left, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, color);

	rlPopMatrix();
}

/// <summary>
/// Méthode permettant de dessiner une boîte arrondie en mode polygones et/ou fil de fer
/// </summary>
/// <param name="roundedBox">Contient les informations de la boîte arrondie (référence => l'orientation et la position, dimensions, rayon des Sphères)</param>
/// <param name="nSectors">Nombre de secteurs utilisés</param>
/// <param name="drawPolygon">Indique s'il faut dessiner les polygones</param>
/// <param name="drawWireframe">Indique s'il faut dessiner le fil de fer</param>
/// <param name="polygonColor">Contient la couleur des polygones</param>
/// <param name="wireframeColor">Contient la couleur du fil de fer</param>
void MyDrawRoundedBox(RoundedBox roundedBox, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonRoundedBox(roundedBox, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeRoundedBox(roundedBox, nSectors, wireframeColor);
}
