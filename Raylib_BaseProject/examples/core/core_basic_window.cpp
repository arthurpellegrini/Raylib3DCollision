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
	//Spherical sphDelta;

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
	float screenSizeCoef = 0.9f;
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
			// LINE
			//Line line = { { 8,4,8 }, { 1,9,0 } };
			//MyDrawLine(line, DARKGRAY);
			// FIN LINE			
			
			// SEGMENT
			//Segment segment = { { 8,4,8 }, { 1,9,0 } };
			//MyDrawSegment(segment, DARKGRAY);
			// FIN SEGMENT

			// TRIANGLE
			//Triangle triangle = { { 8,4,8 }, {8,9,1}, { 7,3,2 } };
			//MyDrawTriangle(triangle, true, true, DARKBLUE);
			// FIN TRIANGLE

			// PLANE
			//Plane plane1 = Plane(Vector3Normalize( { 9, 1, 1 } ), 5.0f);
			//Plane plane2 = Plane(Vector3Normalize( { 7, 2, 6 } ), { 5, 1, 3 });
			//Plane plane3 = Plane(Vector3Normalize( { 5, 1, 3 } ), { 3, 2, 4 }, { 9, 3, 6 });
			//MyDrawPlane(plane1, true, true, DARKGRAY);
			//MyDrawPlane(plane2, true, true, RED);
			//MyDrawPlane(plane3, true, true, ORANGE);
			// FIN PLANE

			// QUAD
			ReferenceFrame ref_quad = ReferenceFrame({ -7,-5,-8 },QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), time));
			Quad quad = { ref_quad, {3,1,5} };
			MyDrawQuad(quad, true, true, YELLOW);
			// FIN QUAD

			// DISK
			ReferenceFrame ref_disk = ReferenceFrame({ -7,-5,8 }, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,0 }), time));
			Disk disk = { ref_disk, 5.0f };
			MyDrawDisk(disk, 20, true, true, BROWN);
			// FIN DISK

			// BOX
			ReferenceFrame ref_box = ReferenceFrame({ -7,5,-8 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), time));
			Box box = { ref_box, { 1,2,3 } };
			MyDrawBox(box, true, true, DARKGREEN);
			// FIN BOX

			// SPHERE
			ReferenceFrame ref_sphere = ReferenceFrame({ -7,5,0 }, QuaternionFromAxisAngle(Vector3Normalize({ 0,0,1 }), -time));
			Sphere sphere = { ref_sphere, 3.0f };
			MyDrawSphere(sphere, 20, 20, true, true, RED);
			// FIN SPHERE

			// CYLINDER
			ReferenceFrame ref_cylinder = ReferenceFrame({ -7,5,8 }, QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), time));
			Cylinder cylinder = { ref_cylinder, 3.0f, 2.0f };
			MyDrawCylinder(cylinder, 20, true, true, true, BLUE);
			// FIN CYLINDER			
			 
			// INFINITE CYLINDER
			//ReferenceFrame ref_infinite_cylinder = ReferenceFrame({ -7,5,8 }, QuaternionFromAxisAngle({0,0,0}, 0));
			//InfiniteCylinder infinite_cylinder = { ref_infinite_cylinder, 5.0f };
			//MyDrawInfiniteCylinder(infinite_cylinder, 10, true, true, BLUE);
			// FIN INFINITE CYLINDER
			
			// CAPSULE
			ReferenceFrame ref_capsule = ReferenceFrame({ 7,5,8 }, QuaternionFromAxisAngle(Vector3Normalize({ 0,1,1 }), -time));
			Capsule capsule = { ref_capsule, 3.0f, 2.0f };
			MyDrawCapsule(capsule, 10, 10, true, true, SKYBLUE);
			// FIN CAPSULE		

			// ROUNDED BOX
			ReferenceFrame ref_rounded_box = { { 7,5,-8 }, QuaternionFromAxisAngle(Vector3Normalize({ 5,1,0 }), time) };
			RoundedBox rounded_box = { ref_rounded_box, { 2,1,3 }, 1.0f };
			MyDrawRoundedBox(rounded_box, 8, true, true, GREEN);
			// FIN ROUNDED BOX

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