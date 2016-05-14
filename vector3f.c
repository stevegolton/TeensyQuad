#include "vector3f.h"

vector3f_t VECTOR3F_Add( vector3f_t a, vector3f_t b )
{
	vector3f_t stNew;

	stNew.x = a.x + b.x;
	stNew.y = a.y + b.y;
	stNew.z = a.z + b.z;

	return stNew;
}

vector3f_t VECTOR3F_Subtract( vector3f_t a, vector3f_t b )
{
	vector3f_t stNew;

	stNew.x = a.x - b.x;
	stNew.y = a.y - b.y;
	stNew.z = a.z - b.z;

	return stNew;
}

vector3f_t VECTOR3F_Scale( vector3f_t a, float fScale )
{
	a.x *= fScale;
	a.y *= fScale;
	a.z *= fScale;

	return a;
}
