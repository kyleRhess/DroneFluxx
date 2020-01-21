#include "matrix.h"

void vector_reset(vector3 *a)
{
   a->e1 = 0.0f;
   a->e2 = 0.0f;
   a->e3 = 0.0f;
}

vector3 vector_add(vector3 a, vector3 b)
{
    vector3 c;

    c.e1 = a.e1 + b.e1;
    c.e2 = a.e2 + b.e2;
    c.e3 = a.e3 + b.e3;

    return c;
}

vector3 vector_sub(vector3 a, vector3 b)
{
    vector3 c;

    c.e1 = a.e1 - b.e1;
    c.e2 = a.e2 - b.e2;
    c.e3 = a.e3 - b.e3;

    return c;
}

vector3 vector_multiply(matrix3x3 *a, vector3 *b)
{
    vector3 c;

    c.e1 = (a->e11 * b->e1) + (a->e12 * b->e2) + (a->e13 * b->e3);
    c.e2 = (a->e21 * b->e1) + (a->e22 * b->e2) + (a->e23 * b->e3);
    c.e3 = (a->e31 * b->e1) + (a->e32 * b->e2) + (a->e33 * b->e3);

    return c;
}

vector3 vector_scale(vector3 *a, float b)
{
    vector3 c;
    
    c.e1 = a->e1 * b;
    c.e2 = a->e2 * b;
    c.e3 = a->e3 * b;

    return c;
}

void matrix_multiply (matrix3x3 *r, matrix3x3 *a, matrix3x3 *b)
{
    r->e11 = (a->e11 * b->e11) + (a->e12 * b->e21) + (a->e13 * b->e31);
    r->e12 = (a->e11 * b->e12) + (a->e12 * b->e22) + (a->e13 * b->e32);
    r->e13 = (a->e11 * b->e13) + (a->e12 * b->e23) + (a->e13 * b->e33);

    r->e21 = (a->e21 * b->e11) + (a->e22 * b->e21) + (a->e23 * b->e31);
    r->e22 = (a->e21 * b->e12) + (a->e22 * b->e22) + (a->e23 * b->e32);
    r->e23 = (a->e21 * b->e13) + (a->e22 * b->e23) + (a->e23 * b->e33);

    r->e31 = (a->e31 * b->e11) + (a->e32 * b->e21) + (a->e33 * b->e31);
    r->e32 = (a->e31 * b->e12) + (a->e32 * b->e22) + (a->e33 * b->e32);
    r->e33 = (a->e31 * b->e13) + (a->e32 * b->e23) + (a->e33 * b->e33);
}

void matrix_scale(matrix3x3 *r, matrix3x3 *a, float b)
{
    r->e11 = a->e11 * b;
    r->e12 = a->e12 * b;
    r->e13 = a->e13 * b;

    r->e21 = a->e21 * b;
    r->e22 = a->e22 * b;
    r->e23 = a->e23 * b;

    r->e31 = a->e31 * b;
    r->e32 = a->e32 * b;
    r->e33 = a->e33 * b;
}

matrix3x3 matrix_add(matrix3x3 *a, matrix3x3 *b)
{
    matrix3x3 c;

    c.e11 = a->e11 + b->e11;
    c.e12 = a->e12 + b->e12;
    c.e13 = a->e13 + b->e13;

    c.e21 = a->e21 + b->e21;
    c.e22 = a->e22 + b->e22;
    c.e23 = a->e23 + b->e23;

    c.e31 = a->e31 + b->e31;
    c.e32 = a->e32 + b->e32;
    c.e33 = a->e33 + b->e33;

    return c;
}

void matrix_transpose(matrix3x3 *t, matrix3x3 *a)
{
    t->e11 = a->e11;
    t->e12 = a->e21;
    t->e13 = a->e31;

    t->e21 = a->e12;
    t->e22 = a->e22;
    t->e23 = a->e32;

    t->e31 = a->e13;
    t->e32 = a->e23;
    t->e33 = a->e33;
}

