#include "MyIntersections.hpp"

#define PESANTEUR 9.80665f // g = 9,80665 m/s² ~~ 9,81 m/s² ou 9,81 N/kg (sur Terre)
#define VPESANTEUR { 0.0f, -PESANTEUR, 0.0f } // Vecteur Pesanteur
#define ENERGIE 1.0f // Energie du système 


#ifndef COLLISIONS_METHODS
#define COLLISIONS_METHODS

void MoveSphere(Sphere sphere, Vector3 velocity, float deltaTime);

bool GetSphereNewPositionAndVelocityIfCollidingWithRoundedBox(
	Sphere sphere,
	RoundedBox rndBox,
	Vector3 velocity,
	float deltaTime,
	float& colT,
	Vector3& colSpherePos,
	Vector3& colNormal,
	Vector3& newPosition,
	Vector3& newVelocity);


bool GetSphereNewPositionAndVelocityIfCollidingWithRoundedBoxes(
	Sphere sphere,
	const std::vector<RoundedBox>& rndBoxes,
	Vector3 velocity,
	float deltaTime,
	float& colT,
	Vector3& colSpherePos,
	Vector3& colNormal,
	Vector3& newPosition,
	Vector3& newVelocity);

bool GetSphereNewPositionAndVelocityIfMultiCollidingWithRoundedBoxes(
	Sphere sphere,
	const std::vector<RoundedBox>& rndBoxes,
	Vector3 velocity,
	float rotInertia,
	Vector3 angularMomentum,
	float deltaTime,
	int nMaxSuccessiveCollisions,
	Vector3& newPosition,
	Vector3& newVelocity,
	Vector3 newAngularMomentum);

#endif
