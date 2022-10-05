#include "reference_frame.h"
#include "coordonnees.h"
#include <raymath.h>
#include <math.h>
#include <float.h>
#include <vector>

#ifndef OBJETS_PRIMITIFS 
#define OBJETS_PRIMITIFS

struct Quad {
	ReferenceFrame ref;
	Vector3 extents;
};

struct Plane {
	Vector3 n;
	float d;
	Plane(Vector3 n, float d) {
		this->n = n;
		this->d = d;
	}

	Plane(Vector3 n, Vector3 pt) {
		this->n = n;
		this->d = Vector3DotProduct(n, pt);

	}
	Plane(Vector3 pt1, Vector3 pt2, Vector3 pt3) {
		this->n = Vector3CrossProduct(Vector3Subtract(pt2, pt1), Vector3Subtract(pt3, pt2));
		if (Vector3Length(n) < EPSILON) {
			this->n = { 0, 0, 0 };
			this->d = 0;
		}
		else {
			this->n = Vector3Normalize(this->n);
			this->d = Vector3DotProduct(this->n, pt1);
		}
	}
};

#endif

void MyDrawPolygonQuad(Quad quad, Color color = LIGHTGRAY);
void MyDrawWireframeQuad(Quad quad, Color color = DARKGRAY);
void MyDrawQuad(Quad quad, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY);

//void MyDrawPlan();
