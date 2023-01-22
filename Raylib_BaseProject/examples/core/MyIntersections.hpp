#include "My3DPrimitives.hpp"


/************************************************
* Méthodes Changement Référentiel				*
*************************************************/
Vector3 LocalToGlobalVect(Vector3 localVect, ReferenceFrame localRef);
Vector3 GlobalToLocalVect(Vector3 globalVect, ReferenceFrame localRef);

Vector3 LocalToGlobalPos(Vector3 localPos, ReferenceFrame localRef);
Vector3 GlobalToLocalPos(Vector3 globalPos, ReferenceFrame localRef);

/************************************************
* Méthodes Géométriques Diverses				*
*************************************************/

//Méthode retournant le projeté orthogonal d’un point sur une droite
Vector3 ProjectedPointOnLine(Vector3 linePt, Vector3 lineUnitDir, Vector3 pt);
//Méthode retournant la distance au carré d’un point à un segment
float SqDistPointSegment(Segment seg, Vector3 pt);
//Méthode qui permet de déterminer si un point est situé à l’intérieur d’une Box
bool IsPointInsideBox(Box box, Vector3 globalPt);

/************************************************
* Méthodes d’intersection Segment/Primitives3D	*
*************************************************/
bool IntersectLinePlane(Line line, Plane plane, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentPlane(Segment seg, Plane plane, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentQuad(Segment seg, Quad quad, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentDisk(Segment segment, Disk disk, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentSphere(Segment seg, Sphere s, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentInfiniteCylinder(Segment segment, InfiniteCylinder cyl, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentCylinder(Segment segment, Cylinder cyl, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentCapsule(Segment seg, Capsule capsule, float& t, Vector3& interPt, Vector3& interNormal);
bool IntersectSegmentBox(Segment seg, Box box, float& t, Vector3 & interPt, Vector3 & interNormal);
bool IntersectSegmentRoundedBox(Segment seg, RoundedBox rndBox, float& t, Vector3& interPt, Vector3& interNormal);
