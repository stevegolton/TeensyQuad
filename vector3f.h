#ifndef VECTOR3F_H
#define VECTOR3F_H

typedef struct
{
	float x;
	float y;
	float z;

} vector3f_t;

vector3f_t VECTOR3F_Add( vector3f_t a, vector3f_t b );
vector3f_t VECTOR3F_Subtract( vector3f_t a, vector3f_t b );
vector3f_t VECTOR3F_Scale( vector3f_t a, float fScale );

#endif /* SOURCES_VECTOR3_H_ */
