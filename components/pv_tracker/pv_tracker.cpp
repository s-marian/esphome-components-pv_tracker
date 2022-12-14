#include "pv_tracker.h"
#include "esphome/core/log.h"
#include <math.h>


#undef D2R
#undef R2D
#define D2R(x) ( (x) / 180.0 * M_PI)
#define R2D(x) ( (x) * 180.0 / M_PI )

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
    LinearAlgebra::sphericToCart(1, M_PI/2 - D2R(elevation_), D2R(azimuth_), sun_vect_prerot);

    // east: y, west: -y
    // north: x, south: -x
    // Plane yOz points towards north



    LinearAlgebra::matmult(rx_mat_, sun_vect_prerot, 3, 3, 1, sun_vect);


    double psi_prerot = atan2( sun_vect_prerot[1], sun_vect_prerot[2] );
    double psi_no_bt = atan2( sun_vect[1], sun_vect[2] );
    double psi = bt.get_backtracking_angle(psi_no_bt);

    double iD_airmass;
    double airmass = getAirMass(90 - elevation_);
    iD_airmass = computeSolarIntensity(90 - elevation_, airmass);
    double capacity_1am = installed_capacity_;


    energyIdeal = iD_airmass * computeEnergy(psi_no_bt, sun_vect_prerot) * capacity_1am;
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
    // this is early morning
    if ( elevation_ < -2 ) {
        return 0;
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


    if ( AM < 0 ) {
        return 0;
    }

    double h = altitude_; // kilometers
    const double a = 0.14;
    iD = 1.353*( (1-a*h) * pow(0.7, pow(AM, 0.678)) + a*h) * 1.1;
    return iD;
}

double PVTrackerSensor::getAirMass(double zenith_deg) {
    if ( zenith_deg <= 90) {
        return (1.0 / ( cos(D2R(zenith_deg)) + 0.50572*pow( (96.07995 - (zenith_deg)), -1.6364)));
    }
    return -1;
}






}
}
