#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "linear_algebra.h"

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
  void set_energy_norot_sensor(sensor::Sensor *sensor) { energy_norot_sensor_ = sensor; }
  void set_south_tilt_angle(float angle);
  void set_tilt_angle_bounds(float min, float max) { tilt_angle_min_ = min; tilt_angle_max_ = max; }
  void set_installed_capacity(double val) { installed_capacity_ = val; }
  void set_altitude(double altitude) { altitude_ = altitude; }


  void setup() override {
    panel_angle_ = 0;
    //set_south_tilt_angle(0);
  }

  void update() override ;

  void dump_config() override {};
  float get_setup_priority() const override { return setup_priority::DATA; }


 private:
  double getPanelAngleEnergy(double &realPsi, double &energyIdeal, double &energyReal, double &energyNoRot);
  double getRealAngle(double psi);
  double computeEnergy( double psi, double *sun_vect );
  double computeSolarIntensity(double zenith, double airmass);
  double getAirMass(double zenith_deg);

 protected:
  void process_(float value);
  sensor::Sensor *azimuth_sensor_;
  sensor::Sensor *elevation_sensor_;
  sensor::Sensor *psi_sensor_;
  sensor::Sensor *energy_ideal_sensor_{nullptr};
  sensor::Sensor *energy_actual_sensor_{nullptr};
  sensor::Sensor *energy_norot_sensor_{nullptr};
  double south_tilt_angle_;
  double tilt_angle_max_, tilt_angle_min_;
  double installed_capacity_ {1000};
  double altitude_;

  double azimuth_, elevation_;
  double panel_angle_;
  double energy_real_, energy_ideal_, energy_norot_;
  double rx_mat_[3*3];


};



}  // namespace pv_tracker
}  // namespace esphome
