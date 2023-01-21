#include <rlgl.h>
#include <iostream>
#include "My3DPrimitives.hpp"

/************************************************
* Line											*
*************************************************/
void MyDrawLine(Line line, Color color)
{
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(line.pt.x, line.pt.y, line.pt.z); // Position du premier point de la ligne
	rlVertex3f(line.pt.x + line.dir.x, line.pt.y + line.dir.y, line.pt.z + line.dir.z); // Position du second point de la ligne (calculé en partant du premier point et en ajoutant la direction de la ligne)
	rlEnd();
}

/************************************************
* Segment										*
*************************************************/
void MyDrawSegment(Segment segment, Color color)
{
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(segment.pt1.x, segment.pt1.y, segment.pt1.z); // Position du premier point du segment
	rlVertex3f(segment.pt2.x, segment.pt2.y, segment.pt2.z); // Position du second point du segment
	rlEnd();
}

/************************************************
* Triangle										*
*************************************************/
void MyDrawPolygonTriangle(Triangle triangle, Color color)
{
	int numVertex = 3;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(triangle.pt1.x, triangle.pt1.y, triangle.pt1.z); // Position du premier point du triangle
	rlVertex3f(triangle.pt2.x, triangle.pt2.y, triangle.pt2.z); // Position du deuxième point du triangle
	rlVertex3f(triangle.pt3.x, triangle.pt3.y, triangle.pt3.z); // Position du troisième point du triangle
	rlEnd();
}

void MyDrawWireframeTriangle(Triangle triangle, Color color)
{
	int numVertex = 3;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(triangle.pt1.x, triangle.pt1.y, triangle.pt1.z); // Position du premier point du triangle
	rlVertex3f(triangle.pt2.x, triangle.pt2.y, triangle.pt2.z); // Position du deuxième point du triangle
	rlVertex3f(triangle.pt3.x, triangle.pt3.y, triangle.pt3.z); // Position du troisième point du triangle
	rlEnd();
}

void MyDrawTriangle(Triangle triangle, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonTriangle(triangle, polygonColor);
	if (drawWireframe) MyDrawWireframeTriangle(triangle, wireframeColor);
}

/************************************************
* Quad											*
*************************************************/
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

/************************************************
* Plane											*
*************************************************/
//void MyDrawPlane(Plane plane, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
//{
//	static int time = 0;
//	ReferenceFrame ref = ReferenceFrame();
//	ref.Translate(Vector3Scale(plane.n, plane.d));
//	ref.RotateByQuaternion(QuaternionFromVector3ToVector3({ 1,0,0 }, plane.n));
//	ref.RotateByQuaternion(QuaternionFromVector3ToVector3({ 0,1,0 }, plane.n));
//	ref.RotateByQuaternion(QuaternionFromAxisAngle({ 0,0,1 }, -PI/2));
//	//ref.RotateByQuaternion(QuaternionFromAxisAngle({ 0,0,1 }, time++/60));
//
//	DrawSphere(ref.origin, 0.1f, RED);
//	MyDrawLine({ ref.origin, plane.n }, GREEN);
//	MyDrawLine({ ref.origin, { 0, 0, 1 } }, BLUE);
//
//	rlPushMatrix();
//	rlTranslatef(ref.origin.x, ref.origin.y, ref.origin.z);
//	Vector3 vect;
//	float angle;
//	QuaternionToAxisAngle(ref.q, &vect, &angle);
//	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
//
//	Quad quad = { ReferenceFrame(), {40,0,40}}; //Impression d'infini avec des extensions de 40m
//
//	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
//	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
//
//	rlPopMatrix();
//}

void MyDrawPlane(Plane plane, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	ReferenceFrame ref;
	ref.origin = Vector3Scale(plane.n, plane.d);
	Vector3 right = Vector3CrossProduct({ 0, 0, 1 }, plane.n);
	ref.q = QuaternionFromAxisAngle(right, 90 * DEG2RAD);

	DrawSphere(ref.origin, 0.1f, RED);
	MyDrawLine({ ref.origin, plane.n }, GREEN);
	MyDrawLine({ ref.origin, { 0, 0, 1 } }, BLUE);

	rlPushMatrix();
	rlTranslatef(ref.origin.x, ref.origin.y, ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Quad quad = { ReferenceFrame(), {40, 0, 40} }; //Impression d'infini avec des extensions de 40m

	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
	rlPopMatrix();
}

/************************************************
* Disk											*
*************************************************/
void MyDrawPolygonDisk(Disk disk, int nSectors, Color color)
{
	int numVertex = nSectors * 3;
	rlPushMatrix();
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 0, disk.radius);

	Cylindrical c1, c2;
	Vector3 v1, v2;

	// On parcourt le nombre de secteurs du disque
	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points des segments
		c1 = { 1, 2 * PI / nSectors * i, 0 };
		c2 = { 1, 2 * PI / nSectors * (i + 1), 0 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawTriangle3D(v2, { 0 }, v1, color);
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
	rlScalef(disk.radius, 0, disk.radius);

	Cylindrical c1, c2;
	Vector3 v1, v2;

	// On parcourt le nombre de secteurs du disque
	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points des segments
		c1 = { 1, 2 * PI / nSectors * i, 0 };
		c2 = { 1, 2 * PI / nSectors * (i + 1), 0 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawLine3D(v1, v2, color);
		DrawLine3D({ 0 }, v2, color);
	}
	rlPopMatrix();
}

void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonDisk(disk, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeDisk(disk, nSectors, wireframeColor);
}

/************************************************
* Box											*
*************************************************/
//{ X -> far; Y -> vertical; Z -> horizontal }
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

	// On détermine l'emplacement des points de la Box
	Vector3 front_top_left = { -1,  1,  1 };
	Vector3 front_top_right = { 1,  1,  1 };
	Vector3 front_bottom_left = { -1, -1,  1 };
	Vector3 front_bottom_right = { 1, -1,  1 };
	Vector3 back_top_left = { -1,  1, -1 };
	Vector3 back_top_right = { 1,  1, -1 };
	Vector3 back_bottom_left = { -1, -1, -1 };
	Vector3 back_bottom_right = { 1, -1, -1 };

	// On dessine la face de devant
	DrawTriangle3D(front_bottom_right, front_top_left, front_bottom_left, color);
	DrawTriangle3D(front_bottom_right, front_top_right, front_top_left, color);
	// On dessine la face de derrière
	DrawTriangle3D(back_bottom_left, back_top_left, back_top_right, color);
	DrawTriangle3D(back_top_right, back_bottom_right, back_bottom_left, color);
	// On dessine la face supérieure
	DrawTriangle3D(back_top_left, front_top_left, front_top_right, color);
	DrawTriangle3D(front_top_right, back_top_right, back_top_left, color);
	// On dessine la face inférieure
	DrawTriangle3D(back_bottom_right, front_bottom_left, back_bottom_left, color);
	DrawTriangle3D(back_bottom_right, front_bottom_right, front_bottom_left, color);
	// On dessine la face gauche
	DrawTriangle3D(front_bottom_left, back_top_left, back_bottom_left, color);
	DrawTriangle3D(front_bottom_left, front_top_left, back_top_left, color);
	// On dessine la face droite
	DrawTriangle3D(front_top_right, front_bottom_right, back_bottom_right, color);
	DrawTriangle3D(front_top_right, back_bottom_right, back_top_right, color);
	rlPopMatrix();
}

void MyDrawWireframeBox(Box box, Color color)
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

	// On détermine l'emplacement des points de la Box
	Vector3 front_top_left = { -1,  1,  1 };
	Vector3 front_top_right = { 1,  1,  1 };
	Vector3 front_bottom_left = { -1, -1,  1 };
	Vector3 front_bottom_right = { 1, -1,  1 };
	Vector3 back_top_left = { -1,  1, -1 };
	Vector3 back_top_right = { 1,  1, -1 };
	Vector3 back_bottom_left = { -1, -1, -1 };
	Vector3 back_bottom_right = { 1, -1, -1 };

	// On dessine la face de devant
	DrawLine3D(front_top_left, front_top_right, color);
	DrawLine3D(front_bottom_left, front_bottom_right, color);
	DrawLine3D(front_top_left, front_bottom_left, color);
	DrawLine3D(front_top_right, front_bottom_right, color);
	DrawLine3D(front_bottom_right, front_top_left, color);
	// On dessine la face de derrière
	DrawLine3D(back_top_left, back_top_right, color);
	DrawLine3D(back_bottom_left, back_bottom_right, color);
	DrawLine3D(back_top_left, back_bottom_left, color);
	DrawLine3D(back_top_right, back_bottom_right, color);
	DrawLine3D(back_bottom_left, back_top_right, color);
	// On dessine la face supérieure
	DrawLine3D(front_top_left, back_top_left, color);
	DrawLine3D(front_top_right, back_top_right, color);
	DrawLine3D(front_top_right, back_top_left, color);
	// On dessine la face inférieure
	DrawLine3D(front_bottom_left, back_bottom_left, color);
	DrawLine3D(front_bottom_right, back_bottom_right, color);
	DrawLine3D(front_bottom_left, back_bottom_right, color);
	// On dessine la face gauche
	DrawLine3D(front_bottom_left, back_top_left, color);
	// On dessine la face droite
	DrawLine3D(back_bottom_right, front_top_right, color);
	rlPopMatrix();
}

void MyDrawBox(Box box, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}

/************************************************
* Sphere										*
*************************************************/
// Positions des points à chaque itération
// 1-----2
// | \   |
// |  \  |
// |   \ |
// 3-----4
//
// MERIDIAN -> phi € [0°, 180°]
//		|
//		|
// -----|----- PARALLEL -> theta € [0°, 360°]
//		|
//		|
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

	Spherical s1, s2, s3, s4;
	Vector3 v1, v2, v3, v4;

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			// On calcule les coordonnées sphériques des points de la face
			s1 = { 1, 2 * PI / nMeridians * m, PI / nParallels * p };
			s2 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * p };
			s3 = { 1, 2 * PI / nMeridians * m, PI / nParallels * (p + 1) };
			s4 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * (p + 1) };

			v1 = SphericalToCartesian(s1);
			v2 = SphericalToCartesian(s2);
			v3 = SphericalToCartesian(s3);
			v4 = SphericalToCartesian(s4);

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawTriangle3D(v1, v4, v2, color);
			DrawTriangle3D(v1, v3, v4, color);
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

	Spherical s1, s2, s3, s4;
	Vector3 v1, v2, v3, v4;

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			// On calcule les coordonnées sphériques des points de la face
			s1 = { 1, 2 * PI / nMeridians * m, PI / nParallels * p };
			s2 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * p };
			s3 = { 1, 2 * PI / nMeridians * m, PI / nParallels * (p + 1) };
			s4 = { 1, 2 * PI / nMeridians * (m + 1), PI / nParallels * (p + 1) };

			v1 = SphericalToCartesian(s1);
			v2 = SphericalToCartesian(s2);
			v3 = SphericalToCartesian(s3);
			v4 = SphericalToCartesian(s4);

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawLine3D(v1, v4, color);
			DrawLine3D(v1, v2, color);
			DrawLine3D(v1, v3, color);
			// Inutile de dessiner [V3,V4] et [V2,V4] car ils correspondront respectivement lors du prochain tour de boucle à [V1,V2] et [V1,V3]
		}
	}
	rlPopMatrix();
}

void MyDrawSphere(Sphere sphere, int nMeridians, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonSphere(sphere, nMeridians, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeSphere(sphere, nMeridians, nParallels, wireframeColor);
}

/********************* Fonctions Portion de primitive Sphère *********************/
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

	Spherical s1, s2, s3, s4;
	Vector3 v1, v2, v3, v4;

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			// On calcule les coordonnées sphériques des points de la face
			s1 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * m };
			s2 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };
			s3 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * m };
			s4 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };

			v1 = SphericalToCartesian(s1);
			v2 = SphericalToCartesian(s2);
			v3 = SphericalToCartesian(s3);
			v4 = SphericalToCartesian(s4);

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawTriangle3D(v1, v2, v4, color);
			DrawTriangle3D(v1, v4, v3, color);
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

	Spherical s1, s2, s3, s4;
	Vector3 v1, v2, v3, v4;

	for (int m = 0; m < nMeridians; m++)
	{
		for (int p = 0; p < nParallels; p++)
		{
			// On calcule les coordonnées sphériques des points de la face
			s1 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * m };
			s2 = { 1, startTheta + (endTheta - startTheta) / nParallels * p, startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };
			s3 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * m };
			s4 = { 1, startTheta + (endTheta - startTheta) / nParallels * (p + 1), startPhi + (endPhi - startPhi) / nMeridians * (m + 1) };

			v1 = SphericalToCartesian(s1);
			v2 = SphericalToCartesian(s2);
			v3 = SphericalToCartesian(s3);
			v4 = SphericalToCartesian(s4);

			if (rlCheckBufferLimit(numVertex)) rlglDraw();
			DrawLine3D(v1, v4, color);
			DrawLine3D(v1, v2, color);
			DrawLine3D(v1, v3, color);
			DrawLine3D(v2, v4, color);
			DrawLine3D(v3, v4, color);
		}
	}
	rlPopMatrix();
}

void MyDrawSpherePortion(Sphere sphere, int nMeridians, int nParallels, float startTheta, float endTheta, float startPhi, float endPhi, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, polygonColor);
	if (drawWireframe) MyDrawWireframeSpherePortion(sphere, nMeridians, nParallels, startTheta, endTheta, startPhi, endPhi, wireframeColor);
}

/************************************************
* Cylinder										*
*************************************************/
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

	Cylindrical c1, c2, c3, c4;
	Vector3 v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points de la face
		c1 = { 1, 2 * PI / nSectors * i, 1 };
		c2 = { 1, 2 * PI / nSectors * (i + 1), 1 };
		c3 = { 1, 2 * PI / nSectors * i, -1 };
		c4 = { 1, 2 * PI / nSectors * (i + 1), -1 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);
		v3 = CylindricalToCartesien(c3);
		v4 = CylindricalToCartesien(c4);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		if (drawCaps) { // alors dessin des disques supérieurs et inférieurs
			DrawTriangle3D(v2, { 0, 1, 0 }, v1, color);
			DrawTriangle3D({ 0, -1, 0 }, v4, v3, color);
		}

		DrawTriangle3D(v1, v4, v2, color);
		DrawTriangle3D(v1, v3, v4, color);
	}
	rlPopMatrix();
}


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

	Cylindrical c1, c2, c3, c4;
	Vector3 v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points de la face
		c1 = { 1, 2 * PI / nSectors * i, 1 };
		c2 = { 1, 2 * PI / nSectors * (i + 1), 1 };
		c3 = { 1, 2 * PI / nSectors * i, -1 };
		c4 = { 1, 2 * PI / nSectors * (i + 1), -1 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);
		v3 = CylindricalToCartesien(c3);
		v4 = CylindricalToCartesien(c4);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		if (drawCaps) { // alors dessin des disques supérieurs et inférieurs
			DrawLine3D(v1, { 0, 1, 0 }, color);
			DrawLine3D(v3, { 0, -1, 0 }, color);
		}

		DrawLine3D(v1, v2, color);
		DrawLine3D(v3, v4, color);
		DrawLine3D(v1, v3, color);
		DrawLine3D(v1, v4, color);
		// Inutile de dessiner [V2,V4] car il correspondra lors du prochain tour de boucle à [V1,V3]
	}
	rlPopMatrix();
}

void MyDrawCylinder(Cylinder cylinder, int nSectors, bool drawCaps, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCylinder(cylinder, nSectors, drawCaps, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(cylinder, nSectors, drawCaps, wireframeColor);
}

void MyDrawInfiniteCylinder(InfiniteCylinder infiniteCylinder, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	Cylinder inf_cylinder = Cylinder{ infiniteCylinder.ref,  40.0f, infiniteCylinder.radius };
	if (drawPolygon) MyDrawPolygonCylinder(inf_cylinder, nSectors, false, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(inf_cylinder, nSectors, false, wireframeColor);
}

/******************** Fonctions Portion de primitive Cylinder *********************/
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

	Cylindrical c1, c2, c3, c4;
	Vector3 v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points de la face
		c1 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, 1 };
		c2 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), 1 };
		c3 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, -1 };
		c4 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), -1 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);
		v3 = CylindricalToCartesien(c3);
		v4 = CylindricalToCartesien(c4);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawTriangle3D(v1, v4, v2, color);
		DrawTriangle3D(v1, v3, v4, color);
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

	Cylindrical c1, c2, c3, c4;
	Vector3 v1, v2, v3, v4;

	for (int i = 0; i < nSectors; i++) {
		// On calcule les coordonnées cylindriques des points de la face
		c1 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, 1 };
		c2 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), 1 };
		c3 = { 1, startTheta + (endTheta - startTheta) / nSectors * i, -1 };
		c4 = { 1, startTheta + (endTheta - startTheta) / nSectors * (i + 1), -1 };

		v1 = CylindricalToCartesien(c1);
		v2 = CylindricalToCartesien(c2);
		v3 = CylindricalToCartesien(c3);
		v4 = CylindricalToCartesien(c4);

		if (rlCheckBufferLimit(numVertex)) rlglDraw();

		DrawLine3D(v1, v2, color);
		DrawLine3D(v3, v4, color);
		DrawLine3D(v1, v3, color);
		DrawLine3D(v1, v4, color);
		DrawLine3D(v2, v4, color);
	}
	rlPopMatrix();
}

void MyDrawCylinderPortion(Cylinder cylinder, int nSectors, float startTheta, float endTheta, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonCylinderPortion(cylinder, nSectors, startTheta, endTheta, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinderPortion(cylinder, nSectors, startTheta, endTheta, wireframeColor);
}

/************************************************
* Capsule										*
*************************************************/
// Pour dessiner la Capsule nous utilisons les Primitives 3D "PortionSphere" et "PortionCylinder".
void MyDrawPolygonCapsule(Capsule capsule, int nSectors, int nParallels, Color color) 
{
	rlPushMatrix();
	rlTranslatef(capsule.ref.origin.x, capsule.ref.origin.y, capsule.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(capsule.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Quaternion q = QuaternionIdentity();
	Sphere capsule_sphere_top = { ReferenceFrame({0, capsule.halfHeight, 0}, q), capsule.radius };
	Cylinder capsule_cylinder = { ReferenceFrame({0, 0, 0}, q), capsule.halfHeight, capsule.radius };
	Sphere capsule_sphere_bottom = { ReferenceFrame({0, -capsule.halfHeight, 0}, q), capsule.radius };

	MyDrawPolygonSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonCylinder(capsule_cylinder, nParallels, false, color);
	MyDrawPolygonSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
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

	Quaternion q = QuaternionIdentity();
	Sphere capsule_sphere_top = { ReferenceFrame({0, capsule.halfHeight, 0}, q), capsule.radius };
	Cylinder capsule_cylinder = { ReferenceFrame({0, 0, 0}, q), capsule.halfHeight, capsule.radius };
	Sphere capsule_sphere_bottom = { ReferenceFrame({0, -capsule.halfHeight, 0}, q), capsule.radius };

	MyDrawWireframeSpherePortion(capsule_sphere_top, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeCylinder(capsule_cylinder, nParallels, false, color);
	MyDrawWireframeSpherePortion(capsule_sphere_bottom, nSectors, nParallels, 0.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	
	rlPopMatrix();
}

void MyDrawCapsule(Capsule capsule, int nSectors, int nParallels, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) 
{
	if (drawPolygon) MyDrawPolygonCapsule(capsule, nSectors, nParallels, polygonColor);
	if (drawWireframe) MyDrawWireframeCapsule(capsule, nSectors, nParallels, wireframeColor);
}

/************************************************
* RoundedBox									*
*************************************************/
// Pour dessiner la RoundedBox nous utilisons les Primitives 3D "PortionSphere", "PortionCylinder" et "Quad".
void MyDrawPolygonRoundedBox(RoundedBox roundedBox, int nSectors, Color color)
{
	rlPushMatrix();
	rlTranslatef(roundedBox.ref.origin.x, roundedBox.ref.origin.y, roundedBox.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(roundedBox.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Quaternion q = QuaternionIdentity();

	// Initialisation des parties sphériques de la RoundedBox (8)
	Sphere sph_top_front_left = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_front_right = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_right = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_left = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_left = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_right = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_right = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_left = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };

	// Initialisation des parties cylindriques de la RoundedBox (12)
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

	// Initialisation des Quads de la RoundedBox (6)
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

	// Affichage des différentes parties de la RoundedBox en Polygon
	MyDrawPolygonSpherePortion(sph_top_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_top_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawPolygonSpherePortion(sph_bottom_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);

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

	MyDrawPolygonQuad(quad_top, color);
	MyDrawPolygonQuad(quad_bottom, color);
	MyDrawPolygonQuad(quad_front, color);
	MyDrawPolygonQuad(quad_back, color);
	MyDrawPolygonQuad(quad_right, color);
	MyDrawPolygonQuad(quad_left, color);

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

	Quaternion q = QuaternionIdentity();

	// Initialisation des parties sphériques de la RoundedBox (8)
	Sphere sph_top_front_left = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_front_right = { ReferenceFrame({ roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_right = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_top_back_left = { ReferenceFrame({ -roundedBox.extents.x,roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_left = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_front_right = { ReferenceFrame({ roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_right = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,-roundedBox.extents.z }, q), roundedBox.radius };
	Sphere sph_bottom_back_left = { ReferenceFrame({ -roundedBox.extents.x,-roundedBox.extents.y,roundedBox.extents.z }, q), roundedBox.radius };

	// Initialisation des parties cylindriques de la RoundedBox (12)
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

	// Initialisation des Quads de la RoundedBox (6)
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
	
	// Affichage des différentes parties de la RoundedBox en Wireframe
	MyDrawWireframeSpherePortion(sph_top_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_top_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 0.0f * DEG2RAD, 90.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_front_left, nSectors, nSectors, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_front_right, nSectors, nSectors, 90.0f * DEG2RAD, 180.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_back_right, nSectors, nSectors, 180.0f * DEG2RAD, 270.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);
	MyDrawWireframeSpherePortion(sph_bottom_back_left, nSectors, nSectors, 270.0f * DEG2RAD, 360.0f * DEG2RAD, 90.0f * DEG2RAD, 180.0f * DEG2RAD, color);

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

	MyDrawWireframeQuad(quad_top, color);
	MyDrawWireframeQuad(quad_bottom, color);
	MyDrawWireframeQuad(quad_front, color);
	MyDrawWireframeQuad(quad_back, color);
	MyDrawWireframeQuad(quad_left, color);
	MyDrawWireframeQuad(quad_right, color);

	rlPopMatrix();
}

void MyDrawRoundedBox(RoundedBox roundedBox, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor)
{
	if (drawPolygon) MyDrawPolygonRoundedBox(roundedBox, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeRoundedBox(roundedBox, nSectors, wireframeColor);
}
