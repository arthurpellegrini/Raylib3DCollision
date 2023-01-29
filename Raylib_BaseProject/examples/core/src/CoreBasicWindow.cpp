/*******************************************************************************************
*
*   raylib [core] example - Basic window
*
*   Welcome to raylib!
*
*   To test examples, just press F6 and execute raylib_compile_execute script
*   Note that compiled executable is placed in the same folder as .c file
*
*   You can find all basic examples on C:\raylib\raylib\examples folder or
*   raylib official webpage: www.raylib.com
*
*   Enjoy using raylib. :)
*
*   This example has been created using raylib 1.0 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   Copyright (c) 2014 Ramon Santamaria (@raysan5)
*
********************************************************************************************/
#include "raylib.h"
#include "rlgl.h"
#include <raymath.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <float.h>
#include <vector>

#include "MyCollisions.hpp"

using namespace std;

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

/********************************************************************************************
* Gestion Caméra																			*
* *******************************************************************************************/
void MyUpdateOrbitalCamera(Camera* camera, float deltaTime)
{
	static Spherical sphPos = { 10.0f, PI / 4.0f, PI / 4.0f }; // la position de départ de la caméra est rho=10m, theta=45° et phi=45°
	Spherical sphSpeed = { 2.0f, 0.04f, 0.04f }; // 2m/incrément de molette et 0.04 radians/pixel

	float rhoMin = 4.0f; // 4m
	float rhoMax = 40.0f; // 40m

	float phiMin = 1.0f * DEG2RAD; 
	float phiMax = 179.0f * DEG2RAD; 

	Vector2 mousePos;
	static Vector2 prevMousePos = { 0.0f, 0.0f };
	Vector2 mouseVect;
	Spherical sphDelta;

	mousePos = GetMousePosition(); // on récupère la position de la souris
	mouseVect = Vector2Subtract(mousePos, prevMousePos); // on récupère le vecteur de déplacement de la souris
	prevMousePos = mousePos; // mise à jour de la position précédente de la souris
	
	float mouseWheelRotation = -GetMouseWheelMove(); // le mouvement de la molette de la souris

	sphDelta.rho = mouseWheelRotation * sphSpeed.rho;
	sphDelta.theta = mouseVect.x * DEG2RAD * sphSpeed.theta;
	sphDelta.phi = Clamp(mouseVect.y, -180.0f, 180.0f) * DEG2RAD * sphSpeed.phi;
	
	if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
	{
		sphPos.theta += sphDelta.theta;
		sphPos.phi += sphDelta.phi;
		if (sphPos.phi < phiMin) sphPos.phi = phiMin;
		if (sphPos.phi > phiMax) sphPos.phi = phiMax;
	}
	
	sphPos.rho = Clamp(sphPos.rho + sphDelta.rho, rhoMin, rhoMax);
	
	// Mise à jour de la caméra
	camera->position = SphericalToCartesian(sphPos);
}

/********************************************************************************************
* Boucle Principale (Main)																	*
* *******************************************************************************************/
int main(int argc, char* argv[])
{
	float screenSizeCoef = 0.8f;
	const int screenWidth = 1920 * screenSizeCoef;
	const int screenHeight = 1080 * screenSizeCoef;

	InitWindow(screenWidth, screenHeight, "ESIEE - E3FI - 2022/2023 - Maths3D - Arthur PELLEGRINI, Clement BRISSARD, Tristan MARTIN");
	SetTargetFPS(60);

	// Set Camera
	Vector3 cameraPos = { 8.0f, 15.0f, 14.0f };
	Camera camera = { 0 };
	camera.position = cameraPos;
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_CUSTOM);  // Set an orbital camera mode

	//----------------------------------------------------------------------------------
	// Init Time Variables
	float deltaTime;
	float time;
	
	// Providing a seed value because random function is used
	srand((unsigned)std::time(NULL));
	
	// GENERATION ALEATOIRE DE LA POSITION X ET Z (Y FIXE) 
	// RAND USAGE => min + rand() % range
	Vector3 newPosition, initPosition = { -15 + rand() % 31, 10, -15 + rand() % 31 };
	Vector3 position = initPosition;
	Vector3 newVelocity, init_velocity = { 10.0f, -2.0f, 6.0f };
	Vector3 velocity = init_velocity;

	// Init 3DPrimitives
	Sphere sphere = { ReferenceFrame(position, QuaternionIdentity()), 1.5f };

	// ROUNDEDBOXES VECTOR
	vector<RoundedBox> rndBoxes;
	// Zone de Test
	float taille_zone = 20.0f;
	rndBoxes.push_back( { { { 0.0f, taille_zone, 0.0f }, QuaternionFromAxisAngle({1,0,0}, 0)}, {taille_zone, 0.5f, taille_zone}, 0.0f } );
	rndBoxes.push_back( { { { 0.0f, -taille_zone, 0.0f }, QuaternionFromAxisAngle({1,0,0}, 0)}, {taille_zone, 0.5f, taille_zone}, 0.0f } );
	rndBoxes.push_back( { { { taille_zone, 0.0f, 0.0f }, QuaternionFromAxisAngle({1,0,0}, 0)}, {0.5f, taille_zone, taille_zone}, 0.0f } ); // FRONT
	rndBoxes.push_back( { { { -taille_zone, 0.0f, 0.0f }, QuaternionFromAxisAngle({1,0,0}, 0)}, {0.5f, taille_zone, taille_zone}, 0.0f } ); // BACK
	rndBoxes.push_back( { { { 0.0f, 0.0f, taille_zone }, QuaternionFromAxisAngle({1,0,0}, 0)}, {taille_zone, taille_zone, 0.5f}, 0.0f } );	 // LEFT
	rndBoxes.push_back( { { { 0.0f, 0.0f, -taille_zone }, QuaternionFromAxisAngle({1,0,0}, 0)}, {taille_zone, taille_zone, 0.5f}, 0.0f } );	// RIGHT
	// Obstacles
	rndBoxes.push_back( { { { -6.0f, -5.0f, 6.0f }, QuaternionFromAxisAngle({1,1,0}, PI/6)}, {2.0f, 1.0f, 2.0f}, 1.0f } );
	rndBoxes.push_back( { { { 3.0f, -10.0f, -5.0f }, QuaternionFromAxisAngle({0,1,1}, PI/5)}, {3.0f, 1.0f, 3.0f}, 1.0f } );

	// Init Sphere Variables
	float masse = 10.0f; // Masse de la Sphère
	float energie = 100.00f; // Energie du système 
	// I = 2/5 * mR² -> Moment d'inertie d'une Sphere Homogène
	float I = 2 * masse * sphere.radius * sphere.radius / 5;
	// Vitesse Angulaire, en rad/s
	float angular_velocity = PI/12;
	// Norme Moment Cinétique 
	float LG_Lenght = I * angular_velocity;
	Vector3 LG = { 0 }, newLG = { 0 };
	float kf = 2.12f;
	Vector3 colOmega = { 0 }, rot = { 0 };

	// Init Colliding Variables
	bool has_collide = false;
	float colT;
	Vector3 colSpherePos, colNormal;

	//----------------------------------------------------------------------------------

	while (!WindowShouldClose())    // Detect window close button or ESC key
	{
		// Update variables
		//----------------------------------------------------------------------------------

		deltaTime = GetFrameTime(); //tn+1 - tn
		time = (float)GetTime();

		MyUpdateOrbitalCamera(&camera, deltaTime);

		if (!has_collide && GetSphereNewPositionAndVelocityIfCollidingWithRoundedBoxes(sphere, rndBoxes, velocity, deltaTime, colT, colSpherePos, colNormal, newPosition, newVelocity))
		{
			// Position & Vitesse;
			position = newPosition;
			init_velocity = newVelocity;
			velocity = newVelocity;

			// Rotation
			colOmega = Vector3Subtract(colSpherePos, sphere.ref.origin);
			rot = Vector3Normalize(Vector3CrossProduct(colOmega, Vector3Scale(velocity, masse)));

			newLG = Vector3Subtract(Vector3Normalize(LG),
				Vector3Scale(Vector3CrossProduct(colOmega,
					Vector3Subtract(velocity,
						Vector3Add(Vector3Scale(colNormal, Vector3DotProduct(velocity, colNormal)), Vector3CrossProduct(rot, colOmega)))),
					kf * deltaTime));

			has_collide = true;
		}
		else
		{
			// v = sqrt( 2/m * (E - mgh) )
			velocity = Vector3Scale(Vector3Normalize(init_velocity), sqrtf((2 * (energie - (masse * PESANTEUR * (position.y - initPosition.y)))) / masse));
			// Vn+1 = Vn + (Tn+1 - Tn) * G
			newVelocity = Vector3Add(velocity, Vector3Scale(VPESANTEUR, deltaTime));
			velocity = newVelocity;

			// OΩn+1 = OΩn + (Tn+1 - Tn) * Vn+1
			sphere.ref.Translate(Vector3Scale(newVelocity, deltaTime));
			position = newPosition;

			// Update Sphere Rotation by Quaternion
			sphere.ref.RotateByQuaternion(QuaternionFromAxisAngle(newLG, LG_Lenght * deltaTime));

			has_collide = false;
		}


		//----------------------------------------------------------------------------------
		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(camera);
		{
		/************************************************
		* TD1											*
		*************************************************/
			//// LINE
			//Line line = { { 8,4,8 }, { 1,9,0 } };
			//MyDrawLine(line, DARKGRAY);
			//// FIN LINE			

			//// SEGMENT
			//Segment segment = { { 8,4,8 }, { 1,9,0 } };
			//MyDrawSegment(segment, DARKGRAY);
			//// FIN SEGMENT

			//// TRIANGLE
			//Triangle triangle = { { 8,4,8 }, {8,9,1}, { 7,3,2 } };
			//MyDrawTriangle(triangle, true, true, DARKBLUE);
			//// FIN TRIANGLE

			//// PLANE
			//Plane plane1 = Plane({ 1, 1,0 }, 5.0f);
			//Plane plane2 = Plane({ 7, 2, 6 }, { 5, 1, 3 });
			//Plane plane3 = Plane({ 5, 1, 3 }, { 3, 2, 4 }, { 9, 3, 6 });
			//MyDrawPlane(plane1, true, true, DARKGRAY);
			//MyDrawPlane(plane2, true, true, RED);
			//MyDrawPlane(plane3, true, true, ORANGE);
			//// FIN PLANE

			//// QUAD
			//ReferenceFrame ref_quad = ReferenceFrame({ -7,-5,-8 },QuaternionFromAxisAngle({ 0,1,0 }, time));
			//Quad quad = { ref_quad, {3,1,5} };
			//MyDrawQuad(quad, true, true, YELLOW);
			//// FIN QUAD

			//// DISK
			//ReferenceFrame ref_disk = ReferenceFrame({ -7,-5,8 }, QuaternionFromAxisAngle({ 0,1,0 }, time));
			//Disk disk = { ref_disk, 5.0f };
			//MyDrawDisk(disk, 20, true, true, BROWN);
			//// FIN DISK

			//// BOX
			//ReferenceFrame ref_box = ReferenceFrame({ -7,5,-8 }, QuaternionFromAxisAngle({ 1,1,1 }, time));
			//Box box = { ref_box, { 1,2,3 } };
			//MyDrawBox(box, true, true, DARKGREEN);
			//// FIN BOX

			//// SPHERE
			//ReferenceFrame ref_sphere = ReferenceFrame({ -7,5,0 }, QuaternionFromAxisAngle({ 0,0,1 }, -time));
			//Sphere sphere = { ref_sphere, 3.0f };
			//MyDrawSphere(sphere, 90, 90, true, true, RED);
			//// FIN SPHERE

			//// CYLINDER
			//ReferenceFrame ref_cylinder = ReferenceFrame({ -7,5,8 }, QuaternionFromAxisAngle({ 1,0,0 }, time));
			//Cylinder cylinder = { ref_cylinder, 3.0f, 2.0f };
			//MyDrawCylinder(cylinder, 20, true, true, true, BLUE);
			//// FIN CYLINDER			

			//// INFINITE CYLINDER
			//ReferenceFrame ref_infinite_cylinder = ReferenceFrame({ -7,5,8 }, QuaternionFromAxisAngle({0}, 0));
			//InfiniteCylinder infinite_cylinder = { ref_infinite_cylinder, 5.0f };
			//MyDrawInfiniteCylinder(infinite_cylinder, 10, true, true, BLUE);
			//// FIN INFINITE CYLINDER

			//// CAPSULE
			//ReferenceFrame ref_capsule = ReferenceFrame({ 7,5,8 }, QuaternionFromAxisAngle({ 0,1,1 }, -time));
			//Capsule capsule = { ref_capsule, 3.0f, 2.0f };
			//MyDrawCapsule(capsule, 10, 10, true, true, SKYBLUE);
			//// FIN CAPSULE		

			//// ROUNDED BOX
			//ReferenceFrame ref_rounded_box = { { 7,5,-8 }, QuaternionFromAxisAngle({ 5,1,0 }, 0) };
			//RoundedBox rounded_box = { ref_rounded_box, { 1,1,1 }, 1.0f };
			//MyDrawRoundedBox(rounded_box, 8, true, true, GREEN);
			//// FIN ROUNDED BOX


		/************************************************
		* TD2											*
		*************************************************/
			//Vector3 interPt;
			//Vector3 interNormal;
			//float t;
			//
			////Draw Line & Segment
			//Vector3 pt1 = { -5,8,0 };
			//Vector3 pt2 = { 5,-8,3 };
			//Segment segment = { pt1, pt2 };
			//Line line = { pt1,Vector3Subtract(pt2,pt1) };
			//MyDrawLine(line, BLACK);
			//MyDrawPolygonSphere({ {pt1,QuaternionIdentity()},.4f }, 8, 8, RED);
			//MyDrawPolygonSphere({ {pt2,QuaternionIdentity()},.4f }, 8, 8, GREEN);

			////Calcul des coordonnées de l'objet (déplacement pour tester les intersections)
			//static Spherical sph = { 5, 0, 90 * DEG2RAD };
			//sph.theta += 1 * DEG2RAD;
			//sph.phi += 1 * DEG2RAD;

			//// TEST PLANE INTERSECTION
			//Plane plane = { Vector3RotateByQuaternion({0,1,0}, QuaternionFromAxisAngle({1,0,0},time * .5f)), 2 };
			//MyDrawPlane(plane);
			//// WITH LINE
			//if (IntersectLinePlane(line, plane, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}
			//// WITH SEGMENT
			//if (IntersectSegmentPlane(segment, plane, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST QUAD INTERSECTION
			//Quad quad = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,0,0},time * .5f) }, {2, 0, 2}};
			//MyDrawQuad(quad);
			//if (IntersectSegmentQuad(segment, quad, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST DISK INTERSECTION
			//Disk disk = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,0,0},time * .5f) }, 5.0f};
			//MyDrawDisk(disk, 20);
			//if (IntersectSegmentDisk(segment, disk, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST SPHERE INTERSECTION
			//Sphere sphere = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({0,2,1},time * .2f)}, 2.0f};
			//MyDrawSphere(sphere, 20, 20, true, true);
			//if (IntersectSegmentSphere(segment, sphere, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.05f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST INFINITE_CYLINDER INTERSECTION
			//InfiniteCylinder inf_cyl = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,2,4},time * .2f)}, 2.0f};
			//MyDrawInfiniteCylinder(inf_cyl, 12, true, true);
			//if (IntersectSegmentInfiniteCylinder(segment, inf_cyl, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.05f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST CYLINDER INTERSECTION
			//Cylinder cyl = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,2,4},time * .2f)}, 3.0f, 2.0f};
			//MyDrawCylinder(cyl, 20, true, true, true);
			//if (IntersectSegmentCylinder(segment, cyl, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.05f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST CAPSULE INTERSECTION
			//Capsule capsule = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,2,4},time * .2f)}, 3.0f, 2.0f};
			//MyDrawCapsule(capsule, 10, 10, true, true);
			//if (IntersectSegmentCapsule(segment, capsule, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.05f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			//// TEST BOX INTERSECTION
			//Box box = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,1,1},time)}, { 3, 4, 3.5f } };
			//MyDrawBox(box, false, true);
			//if (IntersectSegmentBox(segment, box, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.05f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			// TEST ROUNDEDBOX INTERSECTION
			//RoundedBox rndBox = { { SphericalToCartesian(sph), QuaternionFromAxisAngle({1,1,1},time)}, {1.0f, 2.0f, 1.5f}, 1.0f };
			//MyDrawRoundedBox(rndBox, 10, true, true);
			//if (IntersectSegmentRoundedBox(segment, rndBox, t, interPt, interNormal))
			//{
			//	printf("\tt : { %f }\n", t);
			//	printf("\tinterPt : { %f, %f, %f }\n", interPt.x, interPt.y, interPt.z);
			//	printf("\tinterNormal : { %f, %f, %f }\n", interNormal.x, interNormal.y, interNormal.z);
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.4f }, 8, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}


		/************************************************
		* TD3											*
		*************************************************/
			MyDrawSphere(sphere, 20, 20, true, true, RED);

			//Scène de test
			for (int j = 0; j < rndBoxes.size(); j++) {
				if(j < 6) MyDrawWireframeRoundedBox(rndBoxes[j], 10);
				else if(j % 2 == 0) MyDrawRoundedBox(rndBoxes[j], 10, true, true, GREEN);
				else MyDrawRoundedBox(rndBoxes[j], 10, true, true, BLUE);
			}

			//3D REFERENTIAL
			DrawGrid(30, 1.0f);
			DrawLine3D({ 0 }, { 0,15,0 }, DARKGRAY);
			DrawSphere({ 15,0,0 }, .2f, RED);
			DrawSphere({ 0,15,0 }, .2f, GREEN);
			DrawSphere({ 0,0,15 }, .2f, BLUE);
		}
		EndMode3D();
		DrawFPS(10, 10);
		EndDrawing();
	}
	CloseWindow();        // Close window and OpenGL context
	return 0;
}