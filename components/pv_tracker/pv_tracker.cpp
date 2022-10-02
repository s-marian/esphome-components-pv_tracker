#include "pv_tracker.h"
#include "esphome/core/log.h"
#include <math.h>


#define PI M_PI
#define D2R(x) ( (x) / 180.0 * PI)
#define R2D(x) ( (x) * 180.0 / PI )

namespace esphome {
namespace pv_tracker {

static const char *const TAG = "pv_tracker";

void PVTrackerSensor::update() {

    float azimuth, elevation;

    if ( ! (azimuth_sensor_->has_state() && elevation_sensor_->has_state()) ) {
        return;
    }
    azimuth_ = azimuth_sensor_->state;
    elevation_ = elevation_sensor_->state;
    getPanelAngleEnergy(panel_angle_, energy_ideal_, energy_real_, energy_norot_);

    psi_sensor_->publish_state(panel_angle_);

    if ( energy_ideal_sensor_ != nullptr ) {
        energy_ideal_sensor_->publish_state(energy_ideal_);
    }

    if ( energy_actual_sensor_ != nullptr ) {
        energy_actual_sensor_->publish_state(energy_real_);
    }

    if ( energy_norot_sensor_ != nullptr ) {
        energy_norot_sensor_->publish_state(energy_norot_);
    }

    ESP_LOGD(TAG, "azimuth=%f elevation=%f panel_angle=%f", azimuth_, elevation_, panel_angle_ );

    

}

void PVTrackerSensor::set_south_tilt_angle(float angle) {
    south_tilt_angle_ = angle; 
    LinearAlgebra::Ry3D(D2R(south_tilt_angle_), rx_mat_ );
}


double PVTrackerSensor::getPanelAngleEnergy(double &realPsi, double &energyIdeal, double &energyReal, double &energyNoRot) {
    double sun_vect_prerot[3];
    double sun_vect[3];
    LinearAlgebra::sphericToCart(1, PI/2 - D2R(elevation_), D2R(azimuth_), sun_vect_prerot);

    // east: y, west: -y
    // north: x, south: -x
    // Plane yOz points towards north



    LinearAlgebra::matmult(rx_mat_, sun_vect_prerot, 3, 3, 1, sun_vect);


    double psi_prerot = atan2( sun_vect_prerot[1], sun_vect_prerot[2] );
    double psi = atan2( sun_vect[1], sun_vect[2] );

    double iD_airmass;
    double airmass = getAirMass(90 - elevation_);
    iD_airmass = computeSolarIntensity(90 - elevation_, airmass);
    static const double am1p5_factor = (1.0 - 0.15); // calculated at zenith=48.2, h=0m

    double capacity_1am = installed_capacity_ / am1p5_factor;


    energyIdeal = iD_airmass * computeEnergy(psi, sun_vect_prerot) * capacity_1am;
    realPsi     = getRealAngle(psi);
    energyReal  = iD_airmass * computeEnergy(D2R(realPsi), sun_vect_prerot) * capacity_1am;
    energyNoRot = iD_airmass * computeEnergy(0, sun_vect_prerot) * capacity_1am;

    ESP_LOGD(TAG, "sun prerot  XYZ          = %f %f %f", sun_vect_prerot[0], sun_vect_prerot[1], sun_vect_prerot[2]);
    ESP_LOGD(TAG, "sun postrot XYZ          = %f %f %f", sun_vect[0], sun_vect[1], sun_vect[2]);
    ESP_LOGD(TAG, "pv  tilt (pre/post/real) = %.1f %.1f %.1f", R2D(psi_prerot), R2D(psi), realPsi );
    ESP_LOGD(TAG, "energy ideal             = %.2f"     , energyIdeal );
    ESP_LOGD(TAG, "energy real              = %.2f"     , energyReal );
    ESP_LOGD(TAG, "energy assuming no rot   = %.2f"     , energyNoRot );
    ESP_LOGD(TAG, "iD                       = %.2f"     , iD_airmass );
    ESP_LOGD(TAG, "air mass AM              = %.2f"     , airmass );

    return realPsi;
}

double PVTrackerSensor::getRealAngle(double psi) {
    // during evening after sunset, align the panels to maximum rotation
    if ( azimuth_ > 180 ) {
        if ( elevation_ < -2 && elevation_ > -4 ) {
            return -90;
        } else if ( elevation_ <= -4 ) {
            return 0;
        }
    } else {
        // this is early morning
        if ( elevation_ < -2 ) {
            return 0;
        }
    }

    double angle_deg = R2D(psi);

    if ( angle_deg > tilt_angle_max_ ) {
        return tilt_angle_max_;
    }


    if ( angle_deg < tilt_angle_min_ ) {
        return tilt_angle_min_;
    }


    return angle_deg;
}


double PVTrackerSensor::computeEnergy( double psi, double *sun_vect ) {

    if ( elevation_ < 2 ) {
        return 0;
    }

    double unity_up[3] = { 0, 0, 1};
    double tmp1[3*3], panel_normal[3];
    double rot_psi[3*3];



    double structure_tilt[3*3];
    LinearAlgebra::Ry3D(-D2R(south_tilt_angle_), structure_tilt );

    LinearAlgebra::matmult(structure_tilt, LinearAlgebra::Rx3D(-psi, rot_psi) , 3, 3, 3, tmp1);
    LinearAlgebra::matmult(tmp1, unity_up, 3, 3, 1, panel_normal);

    ESP_LOGD(TAG, "sun vect                 = %f %f %f", sun_vect[0], sun_vect[1], sun_vect[2]);
    ESP_LOGD(TAG, "panel normal             = %f %f %f", panel_normal[0], panel_normal[1], panel_normal[2]);

    return LinearAlgebra::dotProduct(panel_normal, sun_vect, 3);

}



double PVTrackerSensor::computeSolarIntensity(double zenith_deg, double airmass) {
    double AM = airmass;

    double iD;

    double h = altitude_; // kilometers
    const double a = 0.14;
    iD = 1.353*( (1-a*h) * pow(0.7, pow(AM, 0.678)) + a*h);
    return iD;
}

double PVTrackerSensor::getAirMass(double zenith_deg) {
    if ( zenith_deg <= 90) {
        return (1.0 / ( cos(D2R(zenith_deg)) + 0.50572*pow( (96.07995 - (zenith_deg)), -1.6364)));
    }
    return -1;
}






#define MAT_ITEM( mat, w, i, j) *((mat) + (i)*(w) + (j) )


void LinearAlgebra::matmult(double *mat1, double *mat2, int m, int n, int p, double *res) {
    // (m, n) x (n, p) -> (m, p)

    for ( int i = 0; i < m; i++) {
        for ( int j = 0; j < p; j++) {
            MAT_ITEM(res, p, i, j) = 0;
            for ( int k = 0; k < n; k++ ) {
                MAT_ITEM(res, p, i, j) += MAT_ITEM(mat1, n, i, k ) * MAT_ITEM(mat2, p, k, j);
            }
        }
    }
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

}
}
