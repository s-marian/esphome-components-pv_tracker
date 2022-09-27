import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_SENSOR,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_ENERGY,
    UNIT_DEGREES,
    UNIT_WATT_HOURS,
    ICON_FLASH,
)

pv_tracker_ns = cg.esphome_ns.namespace("pv_tracker")
PVTrackerSensor = pv_tracker_ns.class_("PVTrackerSensor", cg.PollingComponent)

CONF_AZIMUTH   = "azimuth_sensor"
CONF_ELEVATION = "elevation_sensor"
CONF_SOUTH_TILT_ANGLE = "south_tilt_angle"
CONF_TILT_ANGLE_MAX = "tilt_angle_max"
CONF_TILT_ANGLE_MIN = "tilt_angle_min"
CONF_PSI_ANGLE = "psi_angle"
CONF_ENERGY_IDEAL = "energy_ideal"
CONF_ENERGY_ACTUAL = "energy_actual"
CONF_ENERGY_NOROT = "energy_norot"
CONF_INSTALLED_CAPACITY = "installed_panels_capacity"

# CONFIG_SCHEMA = (
#     sensor.sensor_schema(
#         PVTrackerSensor,
#         unit_of_measurement=UNIT_DEGREES,
#         icon=ICON_FLASH,
#         accuracy_decimals=1,
#         state_class=STATE_CLASS_MEASUREMENT,
#     )
#     .extend(
#         {
#             cv.Required(CONF_AZIMUTH): cv.use_id(sensor.Sensor)
#            ,cv.Required(CONF_ELEVATION): cv.use_id(sensor.Sensor)
#            ,cv.Required(CONF_SOUTH_TILT_ANGLE): cv.angle
#            ,cv.Required(CONF_TILT_ANGLE_MIN): cv.angle
#            ,cv.Required(CONF_TILT_ANGLE_MAX): cv.angle
#         }
#     )
# )

angle_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:axis-y-rotate-clockwise",
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

energy_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_WATT_HOURS,
    icon="mdi:solar-power-variant-outline",
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    device_class=DEVICE_CLASS_ENERGY,
)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PVTrackerSensor)
           ,cv.Required(CONF_AZIMUTH): cv.use_id(sensor.Sensor)
           ,cv.Required(CONF_ELEVATION): cv.use_id(sensor.Sensor)
           ,cv.Required(CONF_SOUTH_TILT_ANGLE): cv.angle
           ,cv.Required(CONF_TILT_ANGLE_MIN): cv.angle
           ,cv.Required(CONF_TILT_ANGLE_MAX): cv.angle
           ,cv.Optional(CONF_INSTALLED_CAPACITY): cv.positive_float
           ,cv.Required(CONF_PSI_ANGLE): angle_schema
           ,cv.Optional(CONF_ENERGY_IDEAL): energy_schema
           ,cv.Optional(CONF_ENERGY_ACTUAL): energy_schema
           ,cv.Optional(CONF_ENERGY_NOROT): energy_schema
        }
    )
    .extend(cv.polling_component_schema("5s"))
)

async def to_code(config):
    #var = await sensor.new_sensor(config)
    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    azim = await cg.get_variable(config[CONF_AZIMUTH])
    elev = await cg.get_variable(config[CONF_ELEVATION])
    south_tilt = config[CONF_SOUTH_TILT_ANGLE]
    cg.add(var.set_sensor_azimuth(azim))
    cg.add(var.set_sensor_elevation(elev))
    cg.add(var.set_south_tilt_angle(south_tilt))
    cg.add(var.set_tilt_angle_bounds(config[CONF_TILT_ANGLE_MIN], config[CONF_TILT_ANGLE_MAX]))

    psi_sens = await sensor.new_sensor(config[CONF_PSI_ANGLE])
    cg.add(var.set_psi_sensor(psi_sens))

    if CONF_INSTALLED_CAPACITY in config:
        cg.add(var.set_installed_capacity(config[CONF_INSTALLED_CAPACITY]))

    for type in ["actual", "ideal", "norot" ]:
        key = f"energy_{type}"
        if key in config:
            energy_sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_energy_{type}_sensor")(energy_sens))
        




