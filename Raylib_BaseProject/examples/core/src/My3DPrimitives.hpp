#include "Coordinates.hpp"

#ifndef _3D_PRIMITIVES 
#define _3D_PRIMITIVES

struct Line 
{
	Vector3 pt;
	Vector3 dir;			// dir != { 0, 0, 0 }

	Line(Vector3 pt, Vector3 dir)
	{
		this->pt = pt;		
		this->dir = Vector3Normalize(dir); // direction unitaire
	}
};

struct Segment 
{		
	// pt1 != pt2
	Vector3 pt1;
	Vector3 pt2; 
};

struct Triangle 
{	
	// pt1 != pt2 && pt1 != pt3 && pt2 != pt3
	Vector3 pt1;
	Vector3 pt2;
	Vector3 pt3; 

	Triangle(Vector3 pt1, Vector3 pt2, Vector3 pt3) 
	{
		this->pt1 = pt1;
		this->pt2 = pt2;
		this->pt3 = pt3;
	}

	Triangle(Vector3 triangle[3]) 
	{
		this->pt1 = triangle[0];
		this->pt2 = triangle[1];
		this->pt3 = triangle[2];
	}
};

struct Quad 
{
	ReferenceFrame ref;
	Vector3 extents;		// Demi-Longueurs depuis le centre du Quad (ici seuls X et Z sont utilisés)
};

struct Plane // "Infini" (effort d’imagination)
{				
	Vector3 n;				// Vecteur normal
	float d;				// Distance à l’origine signée

	Plane(Vector3 n, float d) 
	{
		this->n = Vector3Normalize(n);		// n unitaire
		this->d = d;
	}

	Plane(Vector3 n, Vector3 pt) 
	{
		this->n = Vector3Normalize(n);		// n unitaire
		this->d = Vector3DotProduct(n, pt);
	}

	Plane(Vector3 pt1, Vector3 pt2, Vector3 pt3) 
	{
		this->n = Vector3CrossProduct(Vector3Subtract(pt2, pt1), Vector3Subtract(pt3, pt2));

		// pt1, pt2, pt3 alignés ? 
		if (Vector3Length(n) < EPSILON)   // Pas de Plan 
		{
			this->n = { 0, 0, 0 };
			this->d = 0;
		}
		else 
		{
			this->n = Vector3Normalize(this->n);
			this->d = Vector3DotProduct(this->n, pt1);
		}
	}
};

struct Disk 
{
	ReferenceFrame ref;
	float radius;			// Rayon
};

struct Box 
{
	ReferenceFrame ref;
	Vector3 extents;		// Demi-Longueurs depuis le centre de la Box
};

struct Sphere 
{
	ReferenceFrame ref;
	float radius;			// Rayon
};

struct InfiniteCylinder // Cylinder avec Demi-Hauteur "Infinie" (effort d’imagination)
{	
	ReferenceFrame ref;
	float radius;			// Rayon
};

struct Cylinder 
{
	ReferenceFrame ref;
	float halfHeight;		// Demi-Hauteur depuis le centre
	float radius;			// Rayon
};

struct Capsule 
{
	ReferenceFrame ref;
	float halfHeight;		// Demi-Hauteur depuis le centre
	float radius;			// Rayon du Cylindre et des Demi-Sphères
};

struct RoundedBox 
{
	ReferenceFrame ref;
	Vector3 extents;		// Demi-Longueurs depuis le centre de la RoundedBox
	float radius;			// Rayon du Cylindre et des Demi-Sphères
};

#endif


#ifndef _3D_PRIMITIVES_DRAWING_METHODS
#define _3D_PRIMITIVES_DRAWING_METHODS
/************************************************
* Line											*
*************************************************/
void MyDrawLine(Line line, Color color = DARKGRAY);

/************************************************
* Segment										*
*************************************************/
void MyDrawSegment(Segment segment, Color color = DARKGRAY);

/************************************************
* Triangle										*
*************************************************/
void MyDrawPolygonTriangle(Triangle triangle, Color color = LIGHTGRAY);
void MyDrawWireframeTriangle(Triangle triangle, Color color = DARKGRAY);
void MyDrawTriangle(Triangle triangle, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Quad											*
*************************************************/
void MyDrawPolygonQuad(Quad quad, Color color = LIGHTGRAY);
void MyDrawWireframeQuad(Quad quad, Color color = DARKGRAY);
void MyDrawQuad(Quad quad, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Plane											*
*************************************************/
void MyDrawPlane(Plane plane, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Disk											*
*************************************************/
// nSectors -> Nombre de sections dessinées qui vont permettre de former la primitive Disk
// Plus on augmente ce paramètre et plus la primitive Disk sera ronde et précise 
void MyDrawPolygonDisk(Disk disk, int nSectors, Color color = LIGHTGRAY);
void MyDrawWireframeDisk(Disk disk, int nSectors, Color color = DARKGRAY);
void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor =	DARKGRAY);

/************************************************
* Box											*
*************************************************/
void MyDrawPolygonBox(Box box, Color color = LIGHTGRAY);
void MyDrawWireframeBox(Box box, Color color = DARKGRAY);
void MyDrawBox(Box box, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Sphere										*
*************************************************/
// nMeridians -> nombre de lignes verticales qui composent la primitive Sphere 
// nParralels -> nombre de lignes horizontales qui composent la primitive Sphere 
void MyDrawPolygonSphere(Sphere sphere, int nMeridians, int nParallels, Color color = LIGHTGRAY);
void MyDrawWireframeSphere(Sphere sphere, int nMeridians, int nParallels, Color color = DARKGRAY);
void MyDrawSphere(Sphere sphere, int nMeridians, int nParallels, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/********************* Fonctions Portion de primitive Sphère *********************/
// startTheta -> Angle de départ (en radians) pour le dessin des parallèles
// endTheta -> Angle final (en radians) pour le dessin des parallèles
// startPhi -> Angle de départ (en radians) pour le dessin des méridians
// endPhi -> Angle final (en radians) pour le dessin des méridians
void MyDrawPolygonSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, Color color = LIGHTGRAY);
void MyDrawWireframeSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, Color color = LIGHTGRAY);
void MyDrawSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Cylinder										*
*************************************************/
// nSectors -> Nombre de sections dessinées qui vont permettre de former la primitive Cylinder
void MyDrawInfiniteCylinder(InfiniteCylinder infiniteCylinder, int nSectors, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

// drawCaps -> Indique s'il faut dessiner les disques supérieur et inférieur de la primitive Cylinder
void MyDrawPolygonCylinder(Cylinder cylinder, int nSectors, bool drawCaps = false, Color color = LIGHTGRAY);
void MyDrawWireframeCylinder(Cylinder cylinder, int nSectors, bool drawCaps = false, Color color = LIGHTGRAY);
void MyDrawCylinder(Cylinder cylinder, int nSectors, bool drawCaps = false, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/******************** Fonctions Portion de primitive Cylinder *********************/
// startTheta -> Angle de départ (en radians) pour le dessin de la primitive Cylinder
// endTheta -> Angle final (en radians) pour le dessin de la primitive Cylinder
void MyDrawPolygonCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, Color color = LIGHTGRAY);
void MyDrawWireframeCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, Color color = LIGHTGRAY);
void MyDrawCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* Capsule										*
*************************************************/
// nSectors -> Nombre de sections dessinées qui vont permettre de former la primitive Cylinder qui compose la Capsule
// Permet aussi de déterminer le nombre de méridians qui vont former les portions de Sphere
// nParallels -> Nombre de parallèles qui vont former les portions de Sphere
void MyDrawPolygonCapsule(Capsule capsule, int nSectors, int nParallels, Color color = LIGHTGRAY);
void MyDrawWireframeCapsule(Capsule capsule, int nSectors, int nParallels, Color color = LIGHTGRAY);
void MyDrawCapsule(Capsule capsule, int nSectors, int nParallels, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

/************************************************
* RoundedBox									*
*************************************************/
// nSectors -> Nombre de sections dessinées qui vont permettre de former les portions de Cylinder qui composent la RoundedBox
// Pour les portions de Sphere : nSectors <=> nMeridians <=> nParrallels 
void MyDrawPolygonRoundedBox(RoundedBox roundedBox, int nSectors, Color color = LIGHTGRAY);
void MyDrawWireframeRoundedBox(RoundedBox roundedBox, int nSectors, Color color = LIGHTGRAY);
void MyDrawRoundedBox(RoundedBox roundedBox, int nSectors, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

#endif
