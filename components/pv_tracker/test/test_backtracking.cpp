#include "pv_backtracking.h"
#include <math.h>
#include <stdio.h>

#define PI M_PI
#define D2R(x) ( (x) / 180.0 * PI)
#define R2D(x) ( (x) * 180.0 / PI )

int main() {

    PVBacktracking bt;

    bt.set_panel_width(1.85);
    bt.set_systems_spacing(10.0);
    bt.set_max_angle(D2R(90));
    

//    for ( double r = 0; r < 50; r+=2 ) {
//        printf("crit_angle for rotation %5.1f = %5.1f\n", r, R2D(bt.get_critical_angle( D2R(r) )) );
//    }

    for ( double r = 0; r < 90; r+=1 ) {
        printf("backtrack angle for rotation %5.1f = %5.1f\n", r, R2D(bt.get_backtracking_angle( D2R(r) )) );
    }

    return 0;

}


