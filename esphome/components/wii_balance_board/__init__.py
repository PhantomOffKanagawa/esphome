import esphome.codegen as cg
from esphome.components import binary_sensor, esp32, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WEIGHT,
    ICON_BATTERY,
    ICON_BLUETOOTH,
    ICON_SCALE,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_KILOGRAM,
    UNIT_PERCENT,
)

DEPENDENCIES = ["binary_sensor", "sensor", "esp32"]
AUTO_LOAD = ["binary_sensor", "sensor"]

CONF_SYNCING = "syncing"
CONF_WEIGHT = "weight"
CONF_TEMPERATURE = "temperature_sensor"
CONF_REF_TEMPERATURE = "reference_temperature_sensor"
CONF_STDDEV = "standard_deviation"
CONF_LED_PIN = "led_pin"
CONF_BALANCE_UPDATE_INTERVAL = "balance_update_interval"
CONF_OFF_BOARD_TIMEOUT = "off_board_timeout"
CONF_TOP_LEFT = "top_left"
CONF_TOP_RIGHT = "top_right"
CONF_BOTTOM_LEFT = "bottom_left"
CONF_BOTTOM_RIGHT = "bottom_right"
CONF_LIVE_WEIGHT = "live_weight"
CONF_LEFT_PERCENT = "left_percent"
CONF_FRONT_PERCENT = "front_percent"
CONF_COP_X = "center_of_pressure_x"
CONF_COP_Y = "center_of_pressure_y"
CONF_SWAY = "sway"
CONF_ON_BOARD = "on_board"

ICON_SCALE_BALANCE = "mdi:scale-balance"
ICON_TARGET = "mdi:target"


def _load_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOGRAM,
        icon=ICON_SCALE,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _percent_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        icon=ICON_SCALE_BALANCE,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _cm_schema():
    return sensor.sensor_schema(
        unit_of_measurement="cm",
        icon=ICON_TARGET,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )

wii_balance_board_ns = cg.esphome_ns.namespace("wii_balance_board")
WiiBalanceBoard = wii_balance_board_ns.class_("WiiBalanceBoard", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WiiBalanceBoard),
        cv.Optional(CONF_LED_PIN, default=-1): cv.int_,
        cv.Optional(
            CONF_TEMPERATURE,
            default={
                CONF_NAME: "Balance Board Temperature",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(
            CONF_REF_TEMPERATURE,
            default={
                CONF_NAME: "Balance Board Reference Temperature",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(
            CONF_BATTERY_LEVEL,
            default={
                CONF_NAME: "Balance Board Battery Level",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_BATTERY,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_BATTERY,
        ),
        cv.Optional(
            CONF_WEIGHT,
            default={
                CONF_NAME: "Balance Board Weight",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOGRAM,
            icon=ICON_SCALE,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_WEIGHT,
        ),
        cv.Optional(
            CONF_SYNCING,
            default={
                CONF_NAME: "Balance Board Syncing",
            },
        ): binary_sensor.binary_sensor_schema(
            icon=ICON_BLUETOOTH,
        ),
        cv.Optional(CONF_STDDEV, default=0.4): cv.float_range(0, 5),
        cv.Optional(
            CONF_BALANCE_UPDATE_INTERVAL, default="250ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_OFF_BOARD_TIMEOUT, default="15s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_TOP_LEFT): _load_schema(),
        cv.Optional(CONF_TOP_RIGHT): _load_schema(),
        cv.Optional(CONF_BOTTOM_LEFT): _load_schema(),
        cv.Optional(CONF_BOTTOM_RIGHT): _load_schema(),
        cv.Optional(CONF_LIVE_WEIGHT): _load_schema(),
        cv.Optional(CONF_LEFT_PERCENT): _percent_schema(),
        cv.Optional(CONF_FRONT_PERCENT): _percent_schema(),
        cv.Optional(CONF_COP_X): _cm_schema(),
        cv.Optional(CONF_COP_Y): _cm_schema(),
        cv.Optional(CONF_SWAY): _cm_schema(),
        cv.Optional(CONF_ON_BOARD): binary_sensor.binary_sensor_schema(
            icon="mdi:human-male-height"
        ),
    }
)


async def to_code(config):
    # Newer ESPHome (2025.x+) builds Arduino on top of ESP-IDF and excludes the
    # "bt" IDF component unless something requests it.
    if hasattr(esp32, "request_bluetooth"):
        esp32.request_bluetooth()
    if hasattr(esp32, "request_software_coexistence"):
        esp32.request_software_coexistence()
    esp32.add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_CLASSIC_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_BLE_ENABLED", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BTDM", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BLE_ONLY", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY", True)
    # Keep the controller awake so page scans answer a board-initiated reconnect.
    esp32.add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODEM_SLEEP", False)

    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    temperature_sensor = await sensor.new_sensor(config.get(CONF_TEMPERATURE))
    cg.add(var.set_temperature_sensor(temperature_sensor))

    reference_temperature_sensor = await sensor.new_sensor(
        config.get(CONF_REF_TEMPERATURE)
    )
    cg.add(var.set_reference_temperature_sensor(reference_temperature_sensor))

    battery_level = await sensor.new_sensor(config.get(CONF_BATTERY_LEVEL))
    cg.add(var.set_battery_level(battery_level))

    weight = await sensor.new_sensor(config.get(CONF_WEIGHT))
    cg.add(var.set_weight(weight))

    syncing = await binary_sensor.new_binary_sensor(config.get(CONF_SYNCING))
    cg.add(var.set_syncing(syncing))

    if led_pin := config.get(CONF_LED_PIN):
        cg.add(var.set_led_pin(led_pin))

    if stddev := config.get(CONF_STDDEV):
        cg.add(var.set_stddev(stddev))

    cg.add(var.set_balance_update_interval(config[CONF_BALANCE_UPDATE_INTERVAL]))
    cg.add(var.set_off_board_timeout(config[CONF_OFF_BOARD_TIMEOUT]))

    for key, setter in (
        (CONF_TOP_LEFT, var.set_top_left),
        (CONF_TOP_RIGHT, var.set_top_right),
        (CONF_BOTTOM_LEFT, var.set_bottom_left),
        (CONF_BOTTOM_RIGHT, var.set_bottom_right),
        (CONF_LIVE_WEIGHT, var.set_live_weight),
        (CONF_LEFT_PERCENT, var.set_left_percent),
        (CONF_FRONT_PERCENT, var.set_front_percent),
        (CONF_COP_X, var.set_cop_x),
        (CONF_COP_Y, var.set_cop_y),
        (CONF_SWAY, var.set_sway),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))

    if CONF_ON_BOARD in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_ON_BOARD])
        cg.add(var.set_on_board(bs))
