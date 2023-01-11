#include "my_3D_primitives.h"
#include <rlgl.h>
#include <iostream>


/******************************************************************
*							QUAD								  *
*******************************************************************/
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


void MyDrawQuad(Quad quad, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
}


/******************************************************************
*							PLANE								  *
*******************************************************************/
void MyDrawPlane(Plane plane, Color color)
{
	Vector3 origin = plane.n;
	Vector2 size = { plane.d, plane.d };
	DrawPlane(origin, size, color);
}

// TODO: PLANE à refaire avec 2 DrawTriangles3D()
// TODO: Ajouter méthode MyDrawInfinitePlane()


/******************************************************************
*							BOX 								  *
*******************************************************************/
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

void MyDrawBox(Box box, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}


/******************************************************************
*							DISK								  *
*******************************************************************/
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

		DrawTriangle3D(CylindricToCartesien(v2), { 0 }, CylindricToCartesien(v1), color);
	}
	rlPopMatrix();
}

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

		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v2), color);
		DrawLine3D({ 0 }, CylindricToCartesien(v2), color);
	}
	rlPopMatrix();
}

void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonDisk(disk, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeDisk(disk, nSectors, wireframeColor);
}


/******************************************************************
*							SPHERE 								  *
*******************************************************************/
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

void MyDrawSphere(Sphere sphere, int nMeridians, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonSphere(sphere, nMeridians, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeSphere(sphere, nMeridians, nParallels, wireframeColor);
}



/******************************************************************
*							CYLINDER							  *
*******************************************************************/
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
			DrawTriangle3D(CylindricToCartesien(v2), { 0, 1, 0 }, CylindricToCartesien(v1), color);
			DrawTriangle3D({ 0, -1, 0 }, CylindricToCartesien(v4), CylindricToCartesien(v3), color);
		}

		DrawTriangle3D(CylindricToCartesien(v1), CylindricToCartesien(v4), CylindricToCartesien(v2), color);
		DrawTriangle3D(CylindricToCartesien(v1), CylindricToCartesien(v3), CylindricToCartesien(v4), color);
	}
	rlPopMatrix();
}

void MyDrawWireframeCylinder(Cylinder cylinder, int nSectors, bool drawCaps, Color color)
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
			DrawLine3D(CylindricToCartesien(v1), { 0, 1, 0 }, color);
			DrawLine3D(CylindricToCartesien(v3), { 0, -1, 0 }, color);
		}

		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v2), color);
		DrawLine3D(CylindricToCartesien(v3), CylindricToCartesien(v4), color);
		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v3), color);
		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v4), color);
	}
	rlPopMatrix();
}

void MyDrawCylinder(Cylinder cylinder, int nSectors, bool drawCaps, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCylinder(cylinder, nSectors, drawCaps, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(cylinder, nSectors, drawCaps, wireframeColor);
}

// TODO: Ajouter méthode MyDrawInfiniteCylinder()


/******************************************************************
*							CAPSULE 							  *
*******************************************************************/
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

	MyDrawPolygonSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawPolygonCylinder(capsule_cylinder, nSectors, false, color);
	MyDrawPolygonSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	rlPopMatrix();
}

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

	MyDrawWireframeSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	MyDrawWireframeCylinder(capsule_cylinder, nSectors, false, color);
	MyDrawWireframeSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 360.0f * DEG2RAD, color);
	
	rlPopMatrix();
}

void MyDrawCapsule(Capsule capsule, int nSectors, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCapsule(capsule, nSectors, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeCapsule(capsule, nSectors, nParallels, wireframeColor);
}


/******************************************************************
*					Sphere Optimization Methods					  *
*******************************************************************/
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
			pt1 = { 1, startPhi + (endPhi - startPhi) / nMeridians * m, startTheta + (endTheta - startTheta) / nParallels * p };
			pt2 = { 1, startPhi + (endPhi - startPhi) / nMeridians * (m + 1), startTheta + (endTheta - startTheta) / nParallels * p };
			pt3 = { 1, startPhi + (endPhi - startPhi) / nMeridians * m, startTheta + (endTheta - startTheta) / nParallels * (p + 1) };
			pt4 = { 1, startPhi + (endPhi - startPhi) / nMeridians * (m + 1), startTheta + (endTheta - startTheta) / nParallels * (p + 1) };

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt4), SphericalToCartesian(pt2), color);
			DrawTriangle3D(SphericalToCartesian(pt1), SphericalToCartesian(pt3), SphericalToCartesian(pt4), color);
		}
	}
	rlPopMatrix();
}

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
			pt1 = { 1, startPhi + (endPhi - startPhi) / nMeridians * m, startTheta + (endTheta - startTheta) / nParallels * p };
			pt2 = { 1, startPhi + (endPhi - startPhi) / nMeridians * (m + 1), startTheta + (endTheta - startTheta) / nParallels * p };
			pt3 = { 1, startPhi + (endPhi - startPhi) / nMeridians * m, startTheta + (endTheta - startTheta) / nParallels * (p + 1) };
			pt4 = { 1, startPhi + (endPhi - startPhi) / nMeridians * (m + 1), startTheta + (endTheta - startTheta) / nParallels * (p + 1) };

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

void MyDrawSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, polygonColor);
	if (drawWireframe) MyDrawWireframeSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, wireframeColor);
}

/******************************************************************
*					Cylinder Optimization Methods				  *
*******************************************************************/
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

		DrawTriangle3D(CylindricToCartesien(v1), CylindricToCartesien(v4), CylindricToCartesien(v2), color);
		DrawTriangle3D(CylindricToCartesien(v1), CylindricToCartesien(v3), CylindricToCartesien(v4), color);
	}
	rlPopMatrix();
}

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

		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v2), color);
		DrawLine3D(CylindricToCartesien(v3), CylindricToCartesien(v4), color);
		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v3), color);
		DrawLine3D(CylindricToCartesien(v1), CylindricToCartesien(v4), color);
		DrawLine3D(CylindricToCartesien(v2), CylindricToCartesien(v4), color);
	}
	rlPopMatrix();
}

void MyDrawCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonCylinderPortion(cylinder, nSectors, startTheta, endTheta, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinderPortion(cylinder, nSectors, startTheta, endTheta, wireframeColor);
}

/******************************************************************
*						ROUNDED BOX 							  *
*******************************************************************/
void MyDrawPolygonRoundedBox(RoundedBox roundedBox, int nSectors, Color color)
{
	rlPushMatrix();
	rlTranslatef(roundedBox.ref.origin.x, roundedBox.ref.origin.y, roundedBox.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(roundedBox.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);



	rlPopMatrix();

}

void MyDrawWireframeRoundedBox(RoundedBox roundedBox, int nSectors, Color color)
{
	rlPushMatrix();
	rlTranslatef(roundedBox.ref.origin.x, roundedBox.ref.origin.y, roundedBox.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(roundedBox.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	

	rlPopMatrix();
}

void MyDrawRoundedBox(RoundedBox roundedBox, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonRoundedBox(roundedBox, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeRoundedBox(roundedBox, nSectors, wireframeColor);
}
