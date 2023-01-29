#include "MyCollisions.hpp"

bool GetSphereNewPositionAndVelocityIfCollidingWithRoundedBox(Sphere sphere, RoundedBox rndBox, Vector3 velocity, float deltaTime, float& colT, Vector3& colSpherePos, Vector3& colNormal, Vector3& newPosition, Vector3& newVelocity)
{
	// SI segment sphere entre en collision avec la Somme de Minkowski de la roundedBox ou Box (une roundedBox) alors calculer newPos + newVelocity 
	Segment seg_vel_sph = { sphere.ref.origin, Vector3Add(sphere.ref.origin,Vector3Scale(Vector3Normalize(velocity), 0.5f)) };
	RoundedBox minkowski = { rndBox.ref, rndBox.extents, rndBox.radius + sphere.radius };

	if (IntersectSegmentRoundedBox(seg_vel_sph, minkowski, colT, colSpherePos, colNormal))
	{
		// Nouvelle position du centre de la sphère en prennant en compte le T correspondant à la collision sur le segment
		newPosition = Vector3Add(colSpherePos, Vector3Scale(velocity, 1-colT));
		// Vn+1 = Vn - 2*n(n.Vn) avec Vn+1 le nouveau vecteur vitesse, Vn l'ancien vecteur vitesse et n le vecteur normal orthogonal à la figure dans laquelle la sphère entre en collision 
		newVelocity = Vector3Normalize(Vector3Subtract(velocity, Vector3Scale(Vector3Scale(colNormal, Vector3DotProduct(colNormal, velocity)), 2)));
		// On projecte le point de collision de la box de Minkwski sur la RoundedBox Initiale
		colSpherePos = Vector3Subtract(colSpherePos, Vector3Scale(Vector3Normalize(colNormal), sphere.radius));
		return true;
	}
	return false;
}


bool GetSphereNewPositionAndVelocityIfCollidingWithRoundedBoxes(Sphere sphere, const std::vector<RoundedBox>& rndBoxes, Vector3 velocity, float deltaTime, float& colT, Vector3& colSpherePos, Vector3& colNormal, Vector3& newPosition, Vector3& newVelocity) 
{
	return false;
}

bool GetSphereNewPositionAndVelocityIfMultiCollidingWithRoundedBoxes(Sphere sphere, const std::vector<RoundedBox>& rndBoxes, Vector3 velocity, float rotInertia, Vector3 angularMomentum, float deltaTime, int nMaxSuccessiveCollisions,
	Vector3& newPosition, Vector3& newVelocity, Vector3 newAngularMomentum)
{
	return false;
}
