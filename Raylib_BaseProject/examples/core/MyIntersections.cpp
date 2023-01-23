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
	// Formule : OH = OA + (AP.u)u
	return Vector3Add(linePt, Vector3Scale(lineUnitDir, Vector3DotProduct( Vector3Subtract(pt, linePt), lineUnitDir ) ) );
}

float SqDistPointSegment(Segment seg, Vector3 pt)
{
	// Formule : R² = [PM - (PM.PQ)PQ/PQ.PQ].[PM - (PM.PQ)PQ/PQ.PQ]
	Vector3 pm = Vector3Subtract(pt, seg.pt1); 
	Vector3 pq = Vector3Subtract(seg.pt2, seg.pt1);
	Vector3 res = Vector3Subtract(pm, Vector3Scale(Vector3Scale(pq, Vector3DotProduct(pm, pq)), 1.0f / (Vector3DotProduct(pq, pq))));
	return Vector3DotProduct(res, res);
}

bool IsPointInsideBox(Box box, Vector3 globalPt)
{
	// Formule :	|pt.x| <= exts.x 
	//				|pt.y| <= exts.y 
	//				|pt.z| <= exts.z
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

	// intersection: t, interPt & interNormal
	t = (plane.d - Vector3DotProduct(plane.n, line.pt)) / dotProd; 
	interPt = Vector3Add(line.pt, Vector3Scale(line.dir, t)); // OM = OA+tAB
	interNormal = Vector3Scale(plane.n, Vector3DotProduct(Vector3Subtract(line.pt, interPt), plane.n) < 0 ? -1.f : 1.f);
	return true; 
}

bool IntersectSegmentPlane(Segment seg, Plane plane, float& t, Vector3& interPt, Vector3& interNormal)
{
	Vector3 ab = Vector3Subtract(seg.pt2, seg.pt1);

	// no intersection if line is parallel to the plane
	float dotProd = Vector3DotProduct(plane.n, ab);
	if (fabsf(dotProd) < EPSILON) return false;

	// Formule :   t = d - (OA.n) / AB.n
	t = (plane.d - Vector3DotProduct(seg.pt1, plane.n)) / dotProd;

	// On vérifie si l'intersection se trouve bien sur le segment en utilisant la variable t
	if (t < 0.f || t > 1.f) return false;

	interPt = Vector3Add(seg.pt1, Vector3Scale(ab, t));
	interNormal = dotProd < 0.f ? plane.n : Vector3Negate(plane.n);
	return true;
}

bool IntersectSegmentQuad(Segment seg, Quad quad, float& t, Vector3& interPt, Vector3& interNormal)
{
	// On convertit les points du segment et le segment en coordonnées locales par rapport au Quad
	Vector3 pt1 = GlobalToLocalPos(seg.pt1, quad.ref);
	Vector3 pt2 = GlobalToLocalPos(seg.pt2, quad.ref);
	Vector3 segment = Vector3Subtract(pt2, pt1);

	// Si le segment est parallèle au plan du Quad ou ne le croise pas, il n'y a pas d'intersection
	if ((segment.y <= 0.0f && pt1.y < 0.0f) || (segment.y >= 0.0f && pt1.y > 0.0f)) return false;

	if (segment.y != 0.0f) t = fabsf(pt1.y / segment.y);
	else t = 0.0f;

	// Vérifier si l'intersection se trouve bien sur le segment en utilisant la variable t
	if (t < 0.0f || t > 1.0f) return false;

	Vector3 interPt_local = Vector3Add(pt1, Vector3Scale(segment, t));

	// On vérifie si le point d'intersection est dans les limites du Quad
	if (fabsf(interPt_local.x) > quad.extents.x || fabsf(interPt_local.z) > quad.extents.z) return false;

	// Conversion des coordonées locales vers globales
	interPt = LocalToGlobalPos(interPt_local, quad.ref);
	interNormal = (pt1.y > 0.0f) ? quad.ref.j : Vector3Negate(quad.ref.j);
	return true;
}

bool IntersectSegmentDisk(Segment seg, Disk disk, float& t, Vector3& interPt, Vector3& interNormal)
{
	// On convertit les points du segment et le segment en coordonnées locales par rapport au Disk
	Vector3 pt1 = GlobalToLocalPos(seg.pt1, disk.ref);
	Vector3 pt2 = GlobalToLocalPos(seg.pt2, disk.ref);
	Vector3 segment = Vector3Subtract(pt2, pt1);

	// Si le segment est parallèle au plan du Disk ou ne le croise pas, il n'y a pas d'intersection
	if ((segment.y <= 0.0f && pt1.y < 0.0f) || (segment.y >= 0.0f && pt1.y > 0.0f)) return false;

	if (segment.y != 0.0f) t = fabsf(pt1.y / segment.y);
	else t = 0.0f;

	// Vérifier si l'intersection se trouve bien sur le segment en utilisant la variable t
	if (t < 0.0f || t > 1.0f) return false;

	Vector3 interPt_local = Vector3Add(pt1, Vector3Scale(segment, t));

	// On vérifie si le point d'intersection est dans les limites du Disk
	if (Vector3Length(interPt_local) > disk.radius) return false;

	// Conversion des coordonées locales vers globales
	interPt = LocalToGlobalPos(interPt_local, disk.ref);
	interNormal = (pt1.y > 0.0f) ? disk.ref.j : Vector3Negate(disk.ref.j);
	return true;
}

bool IntersectSegmentSphere(Segment seg, Sphere s, float& t, Vector3& interPt, Vector3& interNormal)
{
	// On converti les points du segment et le centre de la sphere dans le référentiel local de la sphere
	Vector3 localSegment = Vector3Subtract(GlobalToLocalPos(seg.pt2, s.ref), GlobalToLocalPos(seg.pt1, s.ref));
	Vector3 originPt1 = GlobalToLocalPos(seg.pt1, s.ref);

	// On calcule les coefficients de la forme quadratique pour résoudre l'équation
	float a = Vector3DotProduct(localSegment, localSegment);
	float b = 2 * Vector3DotProduct(localSegment, originPt1);
	float c = Vector3DotProduct(originPt1, originPt1) - (s.radius * s.radius);

	// On calcule le delta de la forme quadratique
	float delta = b * b - 4 * a * c;
	if (delta < 0.0f) return false; // Si delta est négatif, il n'y a pas d'intersection

	// On calcule les racines de l'équation
	float sqrtDelta = sqrt(delta);
	t = (-b - sqrtDelta) / (2 * a); // Il est inutile de calculer la 2e racine de cette forme quadratique car elle correspondrait au 2e point d'intersection qui ne nous intéresse pas dans notre cas

	// On vérifie si les racines sont dans l'intervalle [0,1] pour valider l'intersection
	if (t < 0.0f || t > 1.0f) return false;

	// On calcule les informations de sortie de la fonction (point d'intersection et normale)
	interPt = Vector3Add(seg.pt1, Vector3Scale(Vector3Subtract(seg.pt2, seg.pt1), t));
	interNormal = Vector3Normalize(Vector3Subtract(interPt, s.ref.origin));
	return true;
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
