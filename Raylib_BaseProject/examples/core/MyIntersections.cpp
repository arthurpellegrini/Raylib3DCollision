#include "MyIntersections.hpp"

/************************************************
* Méthodes Changement Référentiel				*
*************************************************/
Vector3 LocalToGlobalVect(Vector3 localVect, ReferenceFrame localRef)
{
	// Formule : Vw = Vlx*Iw + Vly*Jw + Vlz*Kw
	return Vector3Add(Vector3Scale(localRef.i, localVect.x), Vector3Add(Vector3Scale(localRef.j, localVect.y), Vector3Scale(localRef.k, localVect.z)));
}

Vector3 GlobalToLocalVect(Vector3 globalVect, ReferenceFrame localRef)
{
	// Formule : Vl = Vw.Iw + Vy.Jw + Vz.Kw
	return { Vector3DotProduct(globalVect, localRef.i), Vector3DotProduct(globalVect, localRef.j), Vector3DotProduct(globalVect, localRef.k) };
}

Vector3 LocalToGlobalPos(Vector3 localPos, ReferenceFrame localRef)
{
	// Formule : OPw = OO'w + Plx*Iw + Ply*Jw + Plz*Kw
	return Vector3Add(localRef.origin, Vector3Add(Vector3Scale(localRef.i, localPos.x), Vector3Add(Vector3Scale(localRef.j, localPos.y), Vector3Scale(localRef.k, localPos.z))));
}

Vector3 GlobalToLocalPos(Vector3 globalPos, ReferenceFrame localRef)
{
	// Formule : O'Pl = O'Pw.Iw + O'Pw.Jw + O'Pw.Kw
	Vector3 local_origin_to_global_pt = Vector3Subtract(globalPos, localRef.origin);
	return { Vector3DotProduct(local_origin_to_global_pt, localRef.i), Vector3DotProduct(local_origin_to_global_pt, localRef.j), Vector3DotProduct(local_origin_to_global_pt, localRef.k) };
}

/************************************************
* Méthodes Géométriques Diverses				*
*************************************************/
Vector3 ProjectedPointOnLine(Vector3 linePt, Vector3 lineUnitDir, Vector3 pt)
{
	Vector3 line_to_point = Vector3Subtract(pt, linePt);
	float dot_product = Vector3DotProduct(line_to_point, lineUnitDir);
	return Vector3Add(Vector3Scale(lineUnitDir, dot_product), linePt);
}

float SqDistPointSegment(Segment seg, Vector3 pt)
{
	Vector3 ab = Vector3Subtract(seg.pt2, seg.pt1);
	Vector3 a_to_point = Vector3Subtract(pt, seg.pt1);
	float dot_product = Vector3DotProduct(ab, a_to_point);

	if (dot_product <= 0.0f) {
		return Vector3LengthSqr(a_to_point);
	}

	Vector3 b_to_point = Vector3Subtract(pt, seg.pt2);
	dot_product = Vector3DotProduct(ab, b_to_point);

	if (dot_product >= 0.0f) {
		return Vector3LengthSqr(b_to_point);
	}
	return Vector3LengthSqr(Vector3Subtract(ProjectedPointOnLine(seg.pt1, Vector3Normalize(ab), pt), pt));
}

bool IsPointInsideBox(Box box, Vector3 globalPt)
{
	Vector3 local_pt_pos = GlobalToLocalPos(globalPt, box.ref);
	return fabsf(local_pt_pos.x) <= box.extents.x && fabsf(local_pt_pos.y) <= box.extents.y && fabsf(local_pt_pos.z) <= box.extents.z;
}

/************************************************
* Méthodes d’intersection Segment/Primitives3D	*
*************************************************/
bool IntersectLinePlane(Line line, Plane plane, float& t, Vector3& interPt, Vector3& interNormal)
{
	// no intersection if line is parallel to the plane
	float dotProd = Vector3DotProduct(plane.n, line.dir);
	if (fabsf(dotProd) < EPSILON) return false;
	t = (plane.d - Vector3DotProduct(plane.n, line.pt)) / dotProd; // intersection: t, interPt & interNormal
	interPt = Vector3Add(line.pt, Vector3Scale(line.dir, t)); // OM = OA+tAB
	interNormal = Vector3Scale(plane.n, Vector3DotProduct(Vector3Subtract(line.pt, interPt), plane.n) < 0 ? -1.f : 1.f);
	return true; 
}

bool IntersectSegmentPlane(Segment seg, Plane plane, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentQuad(Segment seg, Quad quad, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentDisk(Segment segment, Disk disk, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentSphere(Segment seg, Sphere s, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentInfiniteCylinder(Segment segment, InfiniteCylinder cyl, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentCylinder(Segment segment, Cylinder cyl, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentCapsule(Segment seg, Capsule capsule, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentBox(Segment seg, Box box, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}

bool IntersectSegmentRoundedBox(Segment seg, RoundedBox rndBox, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}
