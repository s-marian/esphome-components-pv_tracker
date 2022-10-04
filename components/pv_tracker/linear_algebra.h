#pragma once

#include <math.h>
#include <string.h>

class LinearAlgebra {
    
  public:
    static void sphericToCart(double r, double theta, double phi, double *res);
    static double *matmult(double *mat1, double *mat2, int m, int n, int p, double *res);
    static double *Rx3D(double theta, double *m);
    static double *Ry3D(double theta, double *m);
    static double *Rz3D(double theta, double *m);
    static double *crossProduct3D(double *v_A, double *v_B, double *c_P);
    static double dotProduct ( double *a, double *b, int dim);

};

