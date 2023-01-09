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
#include <raymath.h>
#include "rlgl.h"
#include <math.h>
#include <float.h>
#include <vector>

#include "coordinates.h"
#include "reference_frame.h"
#include "my_3d_primitives.h"

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
	static Spherical sphPos = { 10, PI / 4.f, PI / 4.f }; // la position de départ de la caméra est rho=10m, theta=45° et phi=45°
	Spherical sphSpeed = { 2.0f, 0.04f, 0.04f }; // 2m/incrément de molette et 0.04 radians/pixel

	float rhoMin = 4; // 4m
	float rhoMax = 40; // 40m

	float phiMin = 1.0f * DEG2RAD; 
	float phiMax = 179.0f * DEG2RAD; 

	Vector2 mousePos;
	static Vector2 prevMousePos = { 0, 0 };
	Vector2 mouseVect;
	Spherical sphDelta;

	mousePos = GetMousePosition(); // on récupère la position de la souris
	mouseVect = Vector2Subtract(mousePos, prevMousePos); // on récupère le vecteur de déplacement de la souris
	prevMousePos = mousePos; // mise à jour de la position précédente de la souris
	
	float mouseWheelRotation = -GetMouseWheelMove(); // le mouvement de la molette de la souris

	sphPos.rho += mouseWheelRotation * sphSpeed.rho;
	if (sphPos.rho < rhoMin) sphPos.rho = rhoMin;
	if (sphPos.rho > rhoMax) sphPos.rho = rhoMax;

	if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
	{
		sphPos.theta += mouseVect.x  * DEG2RAD * sphSpeed.theta;
		sphPos.phi += Clamp(mouseVect.y, -179.0f, 179.0f) * DEG2RAD * sphSpeed.phi;
		if (sphPos.phi < phiMin) sphPos.phi = phiMin;
		if (sphPos.phi > phiMax) sphPos.phi = phiMax;
	}
	
	// Mise à jour de la caméra
	camera->position = SphericalToCartesian(sphPos);

	// Monitoring
	//printf("Position de la souris -> x:%f & y:%f \n", mousePos.x, mousePos.y);
	//printf("Delta déplacement souris -> x:%f & y:%f \n", mouseVect.x, mouseVect.y);
	//printf("rho -> %f;theta -> %f; phi -> %f \n", sphPos.rho, sphPos.theta, sphPos.phi);
}


int main(int argc, char* argv[])
{
	// Initialization
	//--------------------------------------------------------------------------------------
	float screenSizeCoef = .9f;
	const int screenWidth = 1920 * screenSizeCoef;
	const int screenHeight = 1080 * screenSizeCoef;

	InitWindow(screenWidth, screenHeight, "ESIEE - E3FI - 2022/2023 - Maths 3D - Arthur PELLEGRINI, Clement BRISSARD, Tristan MARTIN");

	SetTargetFPS(60);

	//CAMERA
	Vector3 cameraPos = { 8.0f, 15.0f, 14.0f };
	Camera camera = { 0 };
	camera.position = cameraPos;
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_CUSTOM);  // Set an orbital camera mode


	//--------------------------------------------------------------------------------------

	// Main game loop
	while (!WindowShouldClose())    // Detect window close button or ESC key
	{
		// Update
		//----------------------------------------------------------------------------------
		// TODO: Update your variables here
		//----------------------------------------------------------------------------------

		float deltaTime = GetFrameTime();
		float time = (float)GetTime();

		MyUpdateOrbitalCamera(&camera, deltaTime);

		// Draw
		//----------------------------------------------------------------------------------
		BeginDrawing();

		ClearBackground(RAYWHITE);

		BeginMode3D(camera);
		{
			//Quaternion qRot1 = QuaternionFromAxisAngle({ 1,0,0 }, PI / 4);
			//Quaternion qRot2 = QuaternionFromAxisAngle({ 0,1,0 }, PI / 2);

			//Quaternion qOrient = QuaternionFromAxisAngle({ 1,0,0 }, PI / 4); // permet de changer l'orientation de l'objet
			//Quaternion qOrient = QuaternionMultiply(qRot1, qRot2); // permet définir la rotation de l'objet en fontionde la multiplication
			//
			//Quaternion qRot = QuaternionFromAxisAngle(Vector3Normalize({ 1, 6, -3 }), time); // Obligé de normaliser le Vecteur car il n'est pas unitaire

			//Quaternion qInitOrient = QuaternionMultiply(qRot1, qRot2); // permet définir la rotation de l'objet en fontionde la multiplication
		
			//Quaternion qOrient = QuaternionMultiply(qRot, qInitOrient); 		

			//ReferenceFrame refRndBox = { { 0,0,0}, QuaternionIdentity() };
			//RoundedBox rndBox = { refRndBox, {2,4, 6}, 1};
			//MyDrawRoundedBox(rndBox, 8);

			// LINE (SEGMENT)
			//Vector3 pt1 = {8,4,8};
			//Vector3 pt2 = {1,9,0};
			//DrawSphere(pt1, .1f, PURPLE); // pour afficher un point (optionnel)
			//DrawSphere(pt2, .1f, DARKBLUE);
			// 
			//DrawLine3D(pt1, pt2, DARKGRAY);
			// FIN LINE


			// TRIANGLE
			//Vector3 pts[3] = { {8, 4, 8}, { 1,9,0 }, { 4,6,8 } }; // utlisation d'un tableau ou de pts  
			//Vector3 pt1 = {8,4,8};
			//Vector3 pt2 = {1,9,0};
			//Vector3 pt3 = {4,6,8};
			//DrawTriangle3D(pt1, pt2, pt3, DARKBLUE);
			// FIN TRIANGLE

			// PLANE
			// MyDrawPlane(Plane({ 0, 2, 7 }, { 2, 7 }));
			// FIN PLANE
			
			// QUAD DISPLAY TEST
			//ReferenceFrame ref = ReferenceFrame({ 0,2,0 },QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), PI / 2));
			//Quad quad = { ref,{3,1,5} };
			//MyDrawQuad(quad);
			// FIN QUAD

			// DISK
			ReferenceFrame ref_disk = ReferenceFrame({ -7, 2, 7 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Disk disk = { ref_disk, 5.0f };
			MyDrawDisk(disk, 50, true, true, DARKBLUE);
			//FIN DISK

			//BOX
			ReferenceFrame ref_box = ReferenceFrame({ -7, 4, -5 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Box box = { ref_box, {4, 2, 3} };
			MyDrawBox(box, true, true, RED);
			//FIN BOX

			// SPHERE
			ReferenceFrame ref_sphere = ReferenceFrame({ 7, 6, -7 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Sphere sphere = { ref_sphere, 5.0f };
			MyDrawSphere(sphere, 50, 50, true, true, SKYBLUE);
			//FIN SPHERE

			//SPHERE PORTION
			ReferenceFrame ref_sphere_portion = ReferenceFrame({ 5, 7, 10 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Sphere sphere_portion = { ref_sphere_portion, 1.0f };
			MyDrawSpherePortion(sphere_portion, 10, 10, 0.0f * DEG2RAD, 90.0f * DEG2RAD, 0.0f * DEG2RAD, 180.0f * DEG2RAD, true, true, DARKGREEN);
			// FIN SPHERE PORTION

			//CYLINER
			ReferenceFrame ref_cylinder = ReferenceFrame({ -7, 7, 7 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Cylinder cylinder = { ref_cylinder, 5.0f, 2.0f };
			MyDrawCylinder(cylinder, 50, true, true, true, BLUE);
			// FIN CYLINDER

			//CYLINER PORTION
			ReferenceFrame ref_cylinder_portion = ReferenceFrame({ 5, 5, 10 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Cylinder cylinder_portion = { ref_cylinder_portion, 2.0f, 1.0f };
			MyDrawCylinderPortion(cylinder_portion, 10, 0.0f * DEG2RAD, 180.0f * DEG2RAD, true, true, GREEN);
			// FIN CYLINDER PORTION
			
			//CAPSULE
			ReferenceFrame ref_capsule = ReferenceFrame({ 10, 7, 7 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), 0));
			Capsule capsule = { ref_capsule, 3.0f, 2.0f };
			MyDrawCapsule(capsule, 10, 10, true, true, ORANGE);
			// FIN CAPSULE

			//3D REFERENTIAL
			DrawGrid(30, 1.0f);        // Draw a grid
			DrawLine3D({ 0 }, { 0,15,0 }, DARKGRAY);
			DrawSphere({ 15,0,0 }, .2f, RED);
			DrawSphere({ 0,15,0 }, .2f, GREEN);
			DrawSphere({ 0,0,15 }, .2f, BLUE);
		}
		EndMode3D();

		EndDrawing();
		//----------------------------------------------------------------------------------
	}

	// De-Initialization
	//--------------------------------------------------------------------------------------   
	CloseWindow();        // Close window and OpenGL context
	//--------------------------------------------------------------------------------------

	return 0;
}