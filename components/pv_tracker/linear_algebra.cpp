#include "linear_algebra.h"


#define MAT_ITEM( mat, w, i, j) *((mat) + (i)*(w) + (j) )


double *LinearAlgebra::matmult(double *mat1, double *mat2, int m, int n, int p, double *res) {
    // (m, n) x (n, p) -> (m, p)

    for ( int i = 0; i < m; i++) {
        for ( int j = 0; j < p; j++) {
            MAT_ITEM(res, p, i, j) = 0;
            for ( int k = 0; k < n; k++ ) {
                MAT_ITEM(res, p, i, j) += MAT_ITEM(mat1, n, i, k ) * MAT_ITEM(mat2, p, k, j);
            }
        }
    }

    return res;
}


void LinearAlgebra::sphericToCart(double r, double theta, double phi, double *res) {

    res[0] = r * sin(theta) * cos(phi);
    res[1] = r * sin(theta) * sin(phi);
    res[2] = r * cos(theta);

}


double *LinearAlgebra::Rx3D(double theta, double *m) {
    const double rm[3*3] = {
        1 , 0, 0,
        0, cos(theta),-sin(theta),
        0, sin(theta), cos(theta)
    };

    memcpy(m, rm, sizeof(rm));
    return m;
}

double *LinearAlgebra::Ry3D(double theta, double *m) {
    const double rm[3*3] = {
        cos(theta), 0, sin(theta),
        0         , 1, 0         ,
        -sin(theta), 0, cos(theta)
    };

    memcpy(m, rm, sizeof(rm));
    return m;
}


double *LinearAlgebra::Rz3D(double theta, double *m) {
    const double rm[3*3] = {
		cos(theta), -sin(theta), 0,
		sin(theta), cos(theta) , 0,
		0         , 0          , 1
    };

    memcpy(m, rm, sizeof(rm));
    return m;
}

double *LinearAlgebra::crossProduct3D(double v_A[], double v_B[], double c_P[]) {
    c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
    c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
    c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];

    return c_P;
}

double LinearAlgebra::dotProduct ( double *a, double *b, int dim) {
    double p = 0;
    for ( int i = 0 ; i < dim ; i++) {
        p += a[i]*b[i];
    }
    return p;
}

