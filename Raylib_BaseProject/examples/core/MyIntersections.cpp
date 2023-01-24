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
Vector3 ProjectedPointOnLine(Line line, Vector3 pt)
{
	// Formule : OH = OA + (AP.u)u
	return Vector3Add(line.pt, Vector3Scale(line.dir, Vector3DotProduct( Vector3Subtract(pt, line.pt), line.dir ) ) );
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

bool IntersectSegmentInfiniteCylinder(Segment seg, InfiniteCylinder cyl, float& t, Vector3& interPt, Vector3& interNormal)
{
	// Convertir les positions du segment en coordonnées locales par rapport au référentiel du cylindre
	Vector3 localSegment = Vector3Subtract(GlobalToLocalPos(seg.pt2, cyl.ref), GlobalToLocalPos(seg.pt1, cyl.ref));
	Vector3 localOriginPt1 = Vector3Subtract(GlobalToLocalPos(seg.pt1, cyl.ref), {0});

	// On ignore la composante Y de la segment et de la position de départ pour ne travailler qu'avec les positions sur le plan xz
	localSegment.y = 0.0f;
	localOriginPt1.y = 0.0f;

	// On calcule les coefficients de la forme quadratique pour résoudre l'équation pour trouver le point d'intersection
	float a = Vector3DotProduct(localSegment, localSegment);
	float b = 2 * Vector3DotProduct(localSegment, localOriginPt1);
	float c = Vector3DotProduct(localOriginPt1, localOriginPt1) - (cyl.radius * cyl.radius);

	float delta = b * b - 4 * a * c;
	if (delta < 0.0f) return false; // Pas de solution, pas d'intersection

	float sqrtDelta = sqrt(delta);

	t = (-b - sqrtDelta) / (2 * a);

	if (t < 0.0f || t > 1.0f) return false; // Intersection en dehors du segment (t !€ [0,1])

	// On calcule les informations de sortie de la fonction(point d'intersection et normale)
	interPt = Vector3Add(seg.pt1, Vector3Scale(Vector3Subtract(seg.pt2, seg.pt1), t));
	// On calcule le vecteur normal à partir du projecté du point d'intersection sur l'axe central du cylindre
	Vector3 P = Vector3Subtract(cyl.ref.origin, cyl.ref.j);
	Vector3 Q = Vector3Add(cyl.ref.origin, cyl.ref.j);
	interNormal = Vector3Normalize(Vector3Subtract(interPt, ProjectedPointOnLine({ P, Vector3Subtract(Q, P) }, interPt)));
	return true;
}

bool IntersectSegmentCylinder(Segment seg, Cylinder cyl, float& t, Vector3& interPt, Vector3& interNormal) {
	// On vérifie si le segment intersecte un cylindre infini contenant le cylindre donné
	if (!IntersectSegmentInfiniteCylinder(seg, { cyl.ref, cyl.radius }, t, interPt, interNormal))
		return false;

	// On calcule les vecteurs PQ, P utilisés pour vérifier si l'intersection se trouve entre les deux disques fermant le cylindre
	Vector3 PQ = Vector3Scale(cyl.ref.j, 2 * cyl.halfHeight);
	Vector3 P = Vector3Subtract(cyl.ref.origin, Vector3Scale(cyl.ref.j, cyl.halfHeight));
	// On calcule le produit scalaire entre (interPt - P) et PQ pour vérifier si l'intersection se trouve entre les deux disques fermant le cylindre
	float InterPtDotPQ = Vector3DotProduct(Vector3Subtract(interPt, P), PQ);

	// Si l'intersection se trouve entre les deux disques fermant le cylindre, on la retourne
	if (!(InterPtDotPQ < 0 || InterPtDotPQ > Vector3DotProduct(PQ, PQ))) return true;

	// Sinon, on crée un disque en utilisant les informations du cylindre (rayon et référentiel)
	// On déplace le référentiel du disque pour qu'il soit au bon endroit par rapport au cylindre (soit en haut, soit en bas)
	Disk disk = { cyl.ref, cyl.radius };
	if (InterPtDotPQ < 0) disk.ref.Translate(Vector3Scale(cyl.ref.j, -cyl.halfHeight));
	else disk.ref.Translate(Vector3Scale(cyl.ref.j, cyl.halfHeight));

	// On vérifie si le segment intersecte ce disque, et on retourne le résultat
	return IntersectSegmentDisk(seg, disk, t, interPt, interNormal);
}

bool IntersectSegmentCapsule(Segment seg, Capsule capsule, float& t, Vector3& interPt, Vector3& interNormal)
{
	// On vérifie si le segment intersecte un cylindre infini contenant le cylindre donné
	if (!IntersectSegmentInfiniteCylinder(seg, { capsule.ref, capsule.radius }, t, interPt, interNormal))
		return false;

	// On calcule les vecteurs PQ, P utilisés pour vérifier si l'intersection se trouve entre les deux disques fermant le cylindre
	Vector3 PQ = Vector3Scale(capsule.ref.j, 2 * capsule.halfHeight);
	Vector3 P = Vector3Subtract(capsule.ref.origin, Vector3Scale(capsule.ref.j, capsule.halfHeight));
	// On calcule le produit scalaire entre (interPt - P) et PQ pour vérifier si l'intersection se trouve entre les deux disques fermant le cylindre
	float InterPtDotPQ = Vector3DotProduct(Vector3Subtract(interPt, P), PQ);

	// Si l'intersection se trouve entre les deux disques fermant le cylindre, on la retourne
	if (!(InterPtDotPQ < 0 || InterPtDotPQ > Vector3DotProduct(PQ, PQ))) return true;

	// Sinon, on crée un disque en utilisant les informations du cylindre (rayon et référentiel)
	// On déplace le référentiel du disque pour qu'il soit au bon endroit par rapport au cylindre (soit en haut, soit en bas)
	Sphere sph = { capsule.ref, capsule.radius };
	if (InterPtDotPQ < 0) sph.ref.Translate(Vector3Scale(capsule.ref.j, -capsule.halfHeight));
	else sph.ref.Translate(Vector3Scale(capsule.ref.j, capsule.halfHeight));

	// On vérifie si le segment intersecte ce disque, et on retourne le résultat
	return IntersectSegmentSphere(seg, sph, t, interPt, interNormal);
}

bool IntersectSegmentBox(Segment seg, Box box, float& t, Vector3& interPt, Vector3& interNormal)
{
	if (IsPointInsideBox(box, seg.pt1))
		return false;

	Quad box_faces[6];
	Quaternion q = box.ref.q;

	// TOP & BOTTOM FACES
	box_faces[0] = { ReferenceFrame(LocalToGlobalPos({0, box.extents.y, 0}, box.ref), q), {box.extents.x, 0, box.extents.z} };
	q = QuaternionMultiply(q, QuaternionFromAxisAngle({ 0,0,1 }, PI));
	box_faces[1] = { ReferenceFrame(LocalToGlobalPos({ 0, -box.extents.y, 0 }, box.ref), q), { box.extents.x, 0, box.extents.z } };

	// FRONT & BACK FACES
	q = QuaternionMultiply(q, QuaternionFromAxisAngle({ 0,0,1 }, PI / 2));
	box_faces[2] = { ReferenceFrame(LocalToGlobalPos({ box.extents.x, 0, 0 }, box.ref), q), { box.extents.y, 0, box.extents.z } };
	q = QuaternionMultiply(q, QuaternionFromAxisAngle({ 0,0,1 }, PI));
	box_faces[3] = { ReferenceFrame(LocalToGlobalPos({ -box.extents.x, 0, 0 }, box.ref), q), { box.extents.y, 0, box.extents.z } };

	// LEFT & RIGHT FACES
	q = QuaternionMultiply(QuaternionMultiply(q, QuaternionFromAxisAngle({ 0,0,1 }, -PI / 2)), QuaternionFromAxisAngle({ 1,0,0 }, PI / 2));
	box_faces[4] = { ReferenceFrame(LocalToGlobalPos({ 0, 0, box.extents.z}, box.ref), q), { box.extents.x, 0, box.extents.y } };
	q = QuaternionMultiply(q, QuaternionFromAxisAngle({ 1,0,0 }, PI));
	box_faces[5] = { ReferenceFrame(LocalToGlobalPos({0, 0, -box.extents.z}, box.ref), q), { box.extents.x, 0, box.extents.y } };

	// Initialisation des variables pour stocker les valeurs correspondantes au point le plus proche de l'origine du segment
	float closest_t = FLT_MAX;
	Vector3 closest_interPt;
	Vector3 closest_interNormal;
	bool hasIntersect = false;

	// Check pour chaque face de la Box
	for (int i = 0; i < 6; i++) {
		MyDrawPolygonQuad(box_faces[i]);
		// Calcul t, interPt et interNormal pour la face en cours
		if (IntersectSegmentQuad(seg, box_faces[i], t, interPt, interNormal)) {
			// Si l'intersection est la plus proche de seg.pt1, on met à jour les variables
			if (t >= 0 && t < closest_t) {
				closest_t = t;
				closest_interPt = interPt;
				closest_interNormal = interNormal;
				hasIntersect = true;
			}
		}
	}

	// Si il y a une intersection, on met à jour les variables de sortie t, interPt et interNormal
	if (hasIntersect) {
		t = closest_t;
		interPt = closest_interPt;
		interNormal = closest_interNormal;
	}

	return hasIntersect;
}

bool IntersectSegmentRoundedBox(Segment seg, RoundedBox rndBox, float& t, Vector3& interPt, Vector3& interNormal)
{
	return false;
}
