
#ifndef __MATRIX_H_
#define __MATRIX_H_


typedef struct vector3def {
	float e1;
	float e2;
	float e3;
} vector3;

typedef struct matrix3x3def{
	float e11;
	float e12;
	float e13;
	float e21;
	float e22;
	float e23;
	float e31;
	float e32;
	float e33;
} matrix3x3;

extern void      vector_reset    (vector3   *a);
extern vector3   vector_add      (vector3    a, vector3  b);
extern vector3   vector_sub      (vector3    a, vector3  b);
extern vector3   vector_multiply (matrix3x3 *a, vector3 *b);
extern vector3   vector_scale    (vector3   *a, float b);
extern void      matrix_multiply (matrix3x3       *r, matrix3x3 *a, matrix3x3 *b);
extern void      matrix_scale    (matrix3x3       *r, matrix3x3 *a, float b);
extern matrix3x3 matrix_add      (matrix3x3 *a, matrix3x3 *b);
extern void      matrix_transpose(matrix3x3       *t, matrix3x3 *a);

#endif
