#include "pv_backtracking.h"
#include <math.h>
#include "linear_algebra.h"
#include <stdio.h>

double PVBacktracking::get_critical_angle(double rot) {

    // double panel_edge[3];
    // panel_edge[0] = 0;
    // panel_edge[1] = panel_width;
    // panel_edge[2] = 0;

    double v[3] = { 0, panel_width, 0};
    Point3D panel_edge(v);

    Point3D panel_edge_rot(panel_edge.Rx(rot));

    
    double dist = systems_spacing/2 - panel_edge_rot.get(1);

    return M_PI/2 - atan2(panel_edge_rot.get(2), dist);


}

#define SIGN(x) ( (x) < 0 ? -1 : 1 )

double PVBacktracking::get_backtracking_angle(double ideal_rot) {

    double rot = fabs(ideal_rot);


    if ( rot > max_angle) {
        rot = max_angle;
    }

    double ca = get_critical_angle(rot);

    if ( ca > fabs(ideal_rot) )  {
        return ideal_rot;
    }

    //printf("Searching\n");

    double step = D2R(1.0);
    for ( double new_rot = rot - step; new_rot >= 0; new_rot -= step ) {

        double new_ca = get_critical_angle(new_rot);
        //printf (" bt new_rot = %f ca = %f\n", new_rot, new_ca);
        if ( new_ca > fabs(ideal_rot) ) {
            return new_rot * SIGN(ideal_rot);
        }

    }

    return 0;


    


}



Point3D Point3D::Rx(double rad) {
    double rot_mat[3*3];

    LinearAlgebra::Rx3D(rad, rot_mat);
    double res[3];

    return Point3D(LinearAlgebra::matmult(rot_mat, this->v, 3, 3, 1, res));
}


