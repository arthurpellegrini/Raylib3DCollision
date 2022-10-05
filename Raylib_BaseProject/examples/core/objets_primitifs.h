#include "reference_frame.h"
#include <raymath.h>
#include <math.h>
#include <float.h>
#include <vector>

struct Quad {
	ReferenceFrame ref;
	Vector3 extends;
};

void DrawQuad(Quad quad, Color color);
