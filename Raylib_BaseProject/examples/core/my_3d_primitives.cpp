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


/******************************************************************
*							DISK								  *
*******************************************************************/
void MyDrawPolygonDisk(Disk disk, int nSectors, Color color) {
	int numVertex = nSectors;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 1, disk.radius);
	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);

	float theta = 0;
	while (theta < 2 * PI){
		rlVertex3f(cos(theta), 0, sin(theta));
		rlVertex3f(0, 0, 0);
		theta += (2 * PI / nSectors);
		rlVertex3f(cos(theta), 0, sin(theta));
	}
	rlEnd();
	rlPopMatrix();
}

void MyDrawWireframeDisk(Disk disk, int nSectors, Color color) {
	int numVertex = nSectors;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 1, disk.radius);
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);

	float theta = 0;
	while (theta < 2 * PI) {
		rlVertex3f(cos(theta), 0, sin(theta));
		theta += (2 * PI / nSectors);
		rlVertex3f(cos(theta), 0, sin(theta));
		
		// Facultatif (Permet d'afficher les traits qui correspondent aux rayons du disque)
		rlVertex3f(0, 0, 0);
		rlVertex3f(cos(theta), 0, sin(theta));
	}
	rlEnd();
	rlPopMatrix();
}

void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) {
	if (drawPolygon) MyDrawPolygonDisk(disk, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeDisk(disk, nSectors, wireframeColor);
}


/******************************************************************
*							BOX 								  *
*******************************************************************/
void MyDrawPolygonBox(Box box, Color color) {
	int numVertex = 36;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);
	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);

	//FRONT  
	rlVertex3f(1, 1, -1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(1, -1, -1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(1, -1, -1);

	//BACK
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, -1, 1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(-1, 1, -1);
	
	//TOP 
	rlVertex3f(1, 1, 1);
	rlVertex3f(1, 1, -1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(-1, 1, 1);

	//BOTTOM
	rlVertex3f(-1, -1, 1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(1, -1, -1);
	rlVertex3f(1, -1, 1);
	
	//LEFT
	rlVertex3f(1, 1, -1);
	rlVertex3f(1, -1, -1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(1, -1, -1);

	//RIGHT
	rlVertex3f(1, -1, 1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(-1, -1, 1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(1, 1, 1);
	
	rlEnd();
	rlPopMatrix();
}

void MyDrawWireframeBox(Box box, Color color) {
	int numVertex = 32;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);
	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	
	//FRONT  
	rlVertex3f(1, 1, -1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(1, -1, -1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(1, 1, -1);
	rlVertex3f(1, -1, -1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(1, 1, -1);

	//BACK
	rlVertex3f(-1, 1, -1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, -1, 1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(-1, -1, 1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(-1, 1, 1);

	//TOP 
	rlVertex3f(1, 1, -1);
	rlVertex3f(-1, 1, -1);
	rlVertex3f(1, 1, 1);
	rlVertex3f(-1, 1, 1);
	rlVertex3f(1, 1, -1);
	rlVertex3f(-1, 1, 1);

	//BOTTOM
	rlVertex3f(1, -1, -1);
	rlVertex3f(-1, -1, -1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(-1, -1, 1);
	rlVertex3f(1, -1, 1);
	rlVertex3f(-1, -1, -1);

	//LEFT
	rlVertex3f(-1, -1, -1);
	rlVertex3f(1, 1, -1);

	//RIGHT
	rlVertex3f(-1, 1, 1);
	rlVertex3f(1, -1, 1);

	rlEnd();
	rlPopMatrix();
}

void MyDrawBox(Box box, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) {
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}
