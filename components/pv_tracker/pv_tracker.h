#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace pv_tracker {

class PVTrackerSensor : public PollingComponent {
 public:
  PVTrackerSensor() : PollingComponent(5000) { set_south_tilt_angle(0); }
  void set_sensor_azimuth(sensor::Sensor *sensor) { azimuth_sensor_ = sensor; }
  void set_sensor_elevation(sensor::Sensor *sensor) { elevation_sensor_ = sensor; }
  void set_psi_sensor(sensor::Sensor *sensor) { psi_sensor_ = sensor; }
  void set_energy_actual_sensor(sensor::Sensor *sensor) { energy_actual_sensor_ = sensor; }
  void set_energy_ideal_sensor(sensor::Sensor *sensor) { energy_ideal_sensor_ = sensor; }
  void set_south_tilt_angle(float angle);
  void set_tilt_angle_bounds(float min, float max) { tilt_angle_min_ = min; tilt_angle_max_ = max; }
  void set_installed_capacity(double val) { installed_capacity_ = val; }


  void setup() override {
    panel_angle_ = 0;
    //set_south_tilt_angle(0);
/*
    this->sensor_->add_on_state_callback([this](float value) { this->process_(value); });
    if (this->sensor_->has_state())
      this->process_(this->sensor_->state);
*/
  }

  void update() override ;

  void dump_config() override {};
  float get_setup_priority() const override { return setup_priority::DATA; }


 private:
  double getPanelAngleEnergy(double &realPsi, double &energyIdeal, double &energyReal);
  double getRealAngle(double psi);
  double computeEnergy( double psi, double *sun_vect );

 protected:
  void process_(float value);
  sensor::Sensor *azimuth_sensor_;
  sensor::Sensor *elevation_sensor_;
  sensor::Sensor *psi_sensor_;
  sensor::Sensor *energy_ideal_sensor_{nullptr};
  sensor::Sensor *energy_actual_sensor_{nullptr};
  float south_tilt_angle_;
  float tilt_angle_max_, tilt_angle_min_;
  float installed_capacity_ {1000};

  double azimuth_, elevation_;
  double panel_angle_;
  double energy_real_, energy_ideal_;
  double rx_mat_[3*3];


};


class LinearAlgebra {
    
  public:
    static void sphericToCart(double r, double theta, double phi, double *res);
    static void matmult(double *mat1, double *mat2, int m, int n, int p, double *res);
    static double *Rx3D(double theta, double *m);
    static double *Ry3D(double theta, double *m);
    static double *Rz3D(double theta, double *m);
    static double *crossProduct3D(double *v_A, double *v_B, double *c_P);
    static double dotProduct ( double *a, double *b, int dim);

};





}  // namespace pv_tracker
}  // namespace esphome
