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

	Vector3 pt1, pt2;
	float theta = 0;

	while (theta < 2 * PI){
		pt1 = { cos(theta), 0, sin(theta) };
		theta += (2 * PI / nSectors);
		pt2 = { cos(theta), 0, sin(theta) };
		DrawTriangle3D(pt1,  { 0 }, pt2, color);
	}
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

	Vector3 pt1, pt2;
	float theta = 0;

	while (theta < 2 * PI) {
		pt1 = { cos(theta), 0, sin(theta) };
		theta += (2 * PI / nSectors);
		pt2 = { cos(theta), 0, sin(theta) };
		DrawLine3D(pt1, pt2, color);
		DrawLine3D({ 0 }, pt2, color);
	}
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

	//{ X -> horizontal; Y -> vertical; Z -> far }
	Vector3 front_top_left		= { -1,  1,  1 };
	Vector3 front_top_right		= {  1,  1,  1 };
	Vector3 front_bottom_left	= { -1, -1,  1 };
	Vector3 front_bottom_right	= {  1, -1,  1 };
	Vector3 back_top_left		= { -1,  1, -1 };
	Vector3 back_top_right		= {  1,  1, -1 };
	Vector3 back_bottom_left	= { -1, -1, -1 };
	Vector3 back_bottom_right	= {  1, -1, -1 };


	//It is necessary to follow the trigonometric direction to have a good orientation of the triangle.
	// 
	//FRONT  
	DrawTriangle3D(front_bottom_right, front_top_left, front_bottom_left, color);
	DrawTriangle3D(front_bottom_right, front_top_right, front_top_left, color);
	//BACK
	DrawTriangle3D(back_bottom_left, back_top_left, back_top_right, color);
	DrawTriangle3D(back_top_right, back_bottom_right, back_bottom_left, color);
	//TOP 
	DrawTriangle3D(back_top_left, front_top_left, front_top_right, color);
	DrawTriangle3D(front_top_right, back_top_right, back_top_left, color);
	//BOTTOM
	DrawTriangle3D(back_bottom_right, front_bottom_left, back_bottom_left, color);
	DrawTriangle3D(back_bottom_right, front_bottom_right, front_bottom_left, color);
	//LEFT
	DrawTriangle3D(front_bottom_left, back_top_left, back_bottom_left, color);
	DrawTriangle3D(front_bottom_left, front_top_left, back_top_left, color);
	//RIGHT
	DrawTriangle3D(front_top_right, front_bottom_right, back_bottom_right, color);
	DrawTriangle3D(front_top_right, back_bottom_right, back_top_right, color);
	
	rlPopMatrix();
}

void MyDrawWireframeBox(Box box, Color color) {
	int numVertex = 36;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();
	rlPushMatrix();
	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);
	
	//{ X -> horizontal; Y -> vertical; Z -> far }
	Vector3 front_top_left		= { -1,  1,  1 };
	Vector3 front_top_right		= {  1,  1,  1 };
	Vector3 front_bottom_left	= { -1, -1,  1 };
	Vector3 front_bottom_right	= {  1, -1,  1 };
	Vector3 back_top_left		= { -1,  1, -1 };
	Vector3 back_top_right		= {  1,  1, -1 };
	Vector3 back_bottom_left	= { -1, -1, -1 };
	Vector3 back_bottom_right	= {  1, -1, -1 };

	//FRONT
	DrawLine3D(front_top_left, front_top_right, color);
	DrawLine3D(front_bottom_left, front_bottom_right, color);
	DrawLine3D(front_top_left, front_bottom_left, color);
	DrawLine3D(front_top_right, front_bottom_right, color);
	DrawLine3D(front_bottom_right, front_top_left, color);
	//BACK
	DrawLine3D(back_top_left, back_top_right, color);
	DrawLine3D(back_bottom_left, back_bottom_right, color);
	DrawLine3D(back_top_left, back_bottom_left, color);
	DrawLine3D(back_top_right, back_bottom_right, color);
	DrawLine3D(back_bottom_left, back_top_right, color);
	//TOP 
	DrawLine3D(front_top_left, back_top_left, color);
	DrawLine3D(front_top_right, back_top_right, color);
	DrawLine3D(front_top_right, back_top_left, color);
	//BOTTOM
	DrawLine3D(front_bottom_left, back_bottom_left, color);
	DrawLine3D(front_bottom_right, back_bottom_right, color);
	DrawLine3D(front_bottom_left, back_bottom_right, color);
	//LEFT
	DrawLine3D(front_bottom_left, back_top_left, color);
	//RIGHT
	DrawLine3D(back_bottom_right, front_top_right, color);

	rlPopMatrix();
}

void MyDrawBox(Box box, bool drawPolygon, bool drawWireframe, Color polygonColor, Color wireframeColor) {
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}
