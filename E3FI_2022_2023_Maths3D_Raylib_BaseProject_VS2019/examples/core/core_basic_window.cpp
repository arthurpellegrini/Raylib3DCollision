﻿/*******************************************************************************************
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

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

#define EPSILON 1.e-6f

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

/*******************************************************************************************
* Conversion coordonnées
* *******************************************************************************************/
struct Polar {
	float rho;
	float theta;
};

Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive = true)
{
	Polar polar = { Vector2Length(cart),atan2f(cart.y,cart.x) };
	if (keepThetaPositive && polar.theta < 0)polar.theta += 2 * PI;
	return polar;
}

Vector2 PolarToCartesian(Polar polar)
{
	return Vector2Scale({ cosf(polar.theta),sinf(polar.theta) }, polar.rho);
}

struct Cylindrical {
	float rho;
	float theta;
	float y;
};

Cylindrical CartesianToCylindric(Vector3 cart)
{
	Cylindrical cyl;
	cyl.y = cart.y;
	cyl.rho = sqrtf(powf(cart.x, 2) + powf(cart.z, 2));

	if (cyl.rho < EPSILON) cyl.theta = 0;
	else {
		cyl.theta = asinf(cart.x / cyl.rho);
		if (cart.z < 0) cyl.theta = PI - cyl.theta;
	}
}

Vector3 CylindricToCartesien(Cylindrical cyl)
{
	return { cyl.rho * sinf(cyl.theta), cyl.y, cyl.rho * cosf(cyl.theta) };
}

struct Spherical {
	float rho;
	float theta; 
	//float y;
	float phi;
};

Spherical CartesianToSpherical(Vector3 cart)
{
	Spherical sph; 
	sph.rho = sqrtf(pow(cart.x, 2) + pow(cart.y, 2) + pow(cart.z, 2));

	if (sph.rho < EPSILON) {
		sph.theta = 0;
		sph.phi = 0;
	}
	else {
		sph.phi = acosf(cart.y / sph.rho);
		if ((sph.phi < EPSILON) || (sph.phi > PI - EPSILON)) sph.theta = 0;
		else {
			sph.theta = asinf(cart.x / (sph.rho * sinf(sph.phi)));
			
			if (cart.z < 0) sph.theta = PI - sph.theta;
		}
	} 
}

Vector3 SphericalToCartesian(Spherical sph)
{
	return { sph.rho * sinf(sph.phi) * sinf(sph.theta), sph.rho * cosf(sph.phi), sph.rho * sinf(sph.phi) * cosf(sph.theta)};
}

//tester double passage dans les fonctions et vérifier l'égalité

/*******************************************************************************************
* Gestion Caméra
* *******************************************************************************************/
void MyUpdateOrbitalCamera(Camera* camera, float deltaTime)
{
	static Spherical sphPos = { 10, PI / 4.f, PI / 4.f }; // la position de départ de la caméra est rho=10m, theta=45° et phi=45°
	Spherical sphSpeed = { 2.0f, 0.04f, 0.04f }; // 2m/incrément de molette et 0.04 radians/pixel

	float rhoMin = 4; // 4m
	float rhoMax = 40; // 40m

	Vector2 mousePos;
	static Vector2 prevMousePos = { 0, 0 };
	Vector2 mouseVect;
	Spherical sphDelta;

	mousePos = GetMousePosition(); // on récupère la position de la souris
	//printf("Position de la souris -> x:%f & y:%f \n", mousePos.x, mousePos.y);

	mouseVect = Vector2Subtract(mousePos, prevMousePos); // on récupère le vecteur de déplacement de la souris
	//printf("Delta déplacement souris -> x:%f & y:%f \n", mouseVect.x, mouseVect.y);

	prevMousePos = mousePos; // maj de la position précédente de la souris
	

	float mouseWheelRotation = GetMouseWheelMove(); // le mouvement de la molette de la souris

	if (mouseWheelRotation != 0.0f)
	{
		sphPos.rho += mouseWheelRotation * sphSpeed.rho;
		if (sphPos.rho < rhoMin) sphPos.rho = rhoMin;
		if (sphPos.rho > rhoMax) sphPos.rho = rhoMax;
	}
		
	if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON))
	{
		if (mouseVect.x != 0.0f) sphPos.theta += mouseVect.x  * DEG2RAD * sphSpeed.theta;
		if (mouseVect.y != 0.0f)
		{

			sphPos.phi += Clamp(mouseVect.y * sphSpeed.phi, -179.0f, 179.0f) * DEG2RAD;
		
			// TENTATIVE LIMITATION EN 0° ET 180° pour éviter l'effet GIMBAL LOCK (je crois)
			//float degre1 = 1.0f * DEG2RAD;
			//if (sphPos.phi < 0)
			//{
			//	if (sphPos.phi < -PI) sphPos.phi = degre1 - PI;
			//	if (sphPos.phi > degre1) sphPos.phi = degre1;
			//}
			//else if (sphPos.phi > 0)
			//{
			//	if (sphPos.phi > PI) sphPos.phi = PI - degre1;
			//	if (sphPos.phi < degre1) sphPos.phi = degre1;
			//}
		}
		//printf("Clic Droit Souris avec Vx -> %f & Vy -> %f\n", mouseVect.x, mouseVect.y);
	}
	
	// MAJ CAMERA
	Vector3 cameraPos = SphericalToCartesian(sphPos);
	camera->position = cameraPos; 

	//printf("rho -> %f;theta -> %f; phi -> %f \n", sphPos.rho, sphPos.theta, sphPos.phi);
}

int main(int argc, char* argv[])
{
	// Initialization
	//--------------------------------------------------------------------------------------
	float screenSizeCoef = .9f;
	const int screenWidth = 1920 * screenSizeCoef;
	const int screenHeight = 1080 * screenSizeCoef;

	InitWindow(screenWidth, screenHeight, "ESIEE - E3FI - 2022 - 2023 -Maths 3D");

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
			//
				
			//3D REFERENTIAL
			DrawGrid(20, 1.0f);        // Draw a grid
			DrawLine3D({ 0 }, { 0,10,0 }, DARKGRAY);
			DrawSphere({ 10,0,0 }, .2f, RED);
			DrawSphere({ 0,10,0 }, .2f, GREEN);
			DrawSphere({ 0,0,10 }, .2f, BLUE);
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