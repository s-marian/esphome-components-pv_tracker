#pragma once

#include <string.h>
#include <math.h>

#define PI M_PI
#define D2R(x) ( (x) / 180.0 * PI)
#define R2D(x) ( (x) * 180.0 / PI )

class PVBacktracking {

    public:
        PVBacktracking() {}
        void set_panel_width ( double w ) { panel_width = w; }
        void set_systems_spacing ( double s ) { systems_spacing = s; }
        void set_max_angle (double s) { max_angle = s; }

        double get_critical_angle(double rot);
        double get_backtracking_angle(double ideal_rot);


    private:

        double panel_width;
        double systems_spacing;
        double max_angle { M_PI/2 } ;




};


class Point3D {

    public:
        Point3D(const double *v) {
            memcpy(this->v, v, sizeof(this->v));
        }

        Point3D(const Point3D &p) {
            memcpy(this->v, p.v, sizeof(this->v));
        }

        Point3D Rx(double rad);

        double get(int i) { return v[i]; }



    private:
        double v[3];

};

