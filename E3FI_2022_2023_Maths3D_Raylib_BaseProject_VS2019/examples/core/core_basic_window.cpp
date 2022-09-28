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

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

#define EPSILON 1.e-6f

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

/********************************************************************************************
* Conversion coordonnées																	*
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
	float phi;
};

Spherical CartesianToSpherical(Vector3 cart)
{
	Spherical sph; 
	sph.rho = sqrtf(powf(cart.x, 2) + powf(cart.y, 2) + powf(cart.z, 2));

	if (sph.rho < EPSILON) {
		sph.theta = 0.0f;
		sph.phi = 0.0f;
	}
	else {
		sph.phi = acosf(cart.y / sph.rho);
		if ((sph.phi < EPSILON) || (sph.phi > PI - EPSILON)) sph.theta = 0.0f;
		else {
			sph.theta = asinf(cart.x / (sph.rho * sinf(sph.phi)));
			
			if (cart.z < 0.0f) sph.theta = PI - sph.theta;
		}
	} 
}

Vector3 SphericalToCartesian(Spherical sph)
{
	return { sph.rho * sinf(sph.phi) * sinf(sph.theta), sph.rho * cosf(sph.phi), sph.rho * sinf(sph.phi) * cosf(sph.theta)};
}

/********************************************************************************************
* ReferenceFrame																			*
* *******************************************************************************************/

struct ReferenceFrame {
	Vector3 origin;
	Vector3 i, j, k;
	Quaternion q;
	ReferenceFrame()
	{
		origin = { 0,0,0 };
		i = { 1,0,0 };
		j = { 0,1,0 };
		k = { 0,0,1 };
		q = QuaternionIdentity();
	}
	ReferenceFrame(Vector3 origin, Quaternion q)
	{
		this->q = q;
		this->origin = origin;
		i = Vector3RotateByQuaternion({ 1,0,0 }, q);
		j = Vector3RotateByQuaternion({ 0,1,0 }, q);
		k = Vector3RotateByQuaternion({ 0,0,1 }, q);
	}
	void Translate(Vector3 vect)
	{
		this->origin = Vector3Add(this->origin, vect);
	}
	void RotateByQuaternion(Quaternion qRot)
	{
		q = QuaternionMultiply(qRot, q);
		i = Vector3RotateByQuaternion({ 1,0,0 }, q);
		j = Vector3RotateByQuaternion({ 0,1,0 }, q);
		k = Vector3RotateByQuaternion({ 0,0,1 }, q);
	}
};

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
	prevMousePos = mousePos; // maj de la position précédente de la souris

	//printf("Position de la souris -> x:%f & y:%f \n", mousePos.x, mousePos.y);
	//printf("Delta déplacement souris -> x:%f & y:%f \n", mouseVect.x, mouseVect.y);
	
	float mouseWheelRotation = GetMouseWheelMove(); // le mouvement de la molette de la souris

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
	
	// MAJ CAMERAS
	camera->position = SphericalToCartesian(sphPos);

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
			/*Vector3 pts[3] = { {8, 4, 8}, { 1,9,0 }, { 4,6,8 } }; // utlisation d'un tableau ou de pts  
			Vector3 pt1 = {8,4,8};
			Vector3 pt2 = {1,9,0};
			Vector3 pt3 = {4,6,8};
			DrawTriangle3D(pt1, pt2, pt3, DARKBLUE);*/
			// FIN TRIANGLE

			// PLANE
			//DrawPlane({0, 0, 0}, {40,40}, PINK);
			// FIN PLANE
			
			// QUAD 
			
			// Faire la méthode pour le Quad

			// FIN QUAD

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